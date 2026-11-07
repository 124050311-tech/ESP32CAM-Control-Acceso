/*
  Evidencia 2 y 3 - ESP32-CAM (AI Thinker)
  - Modo FOTO: PIR o boton de foto -> aviso con buzzer + flash integrado + foto a microSD
  - Modo SEGURIDAD: PIR -> reconocimiento facial -> buzzer distinto para acceso / denegado
  - Servidor web: galeria, vista en vivo, cambio de modo, registro de rostros

  PLACA : AI Thinker ESP32-CAM | PSRAM: Enabled
  PARTICION: Huge APP (3MB No OTA/1MB SPIFFS)
  CORE  : esp32 de Espressif 1.0.6 (en 2.x y 3.x se quito el reconocimiento facial)

  PINES:
    GPIO13 PIR | GPIO12 boton FOTO | GPIO3 boton MODO | GPIO1 buzzer
    GPIO4 flash integrado | GPIO33 LED rojo integrado (indica modo seguridad)
  OJO: GPIO1 y GPIO3 son el puerto serie, por eso NO se usa Serial.
*/
#include "esp_camera.h"
#include "img_converters.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include "FS.h"
#include "SD_MMC.h"
#include "WiFi.h"
#include "WebServer.h"
#include "ESPmDNS.h"
#include "Preferences.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ---------- CONFIGURACION ----------
#define USAR_MODO_AP      1      // 1 = la ESP crea su red (IP 192.168.4.1). 0 = se conecta a tu router
const char* AP_SSID  = "ESP32CAM-ACCESO";
const char* AP_PASS  = "12345678";
const char* WIFI_SSID = "TU_WIFI";      // solo si USAR_MODO_AP = 0 (entra en http://esp32cam.local)
const char* WIFI_PASS = "TU_CLAVE";

#define PIN_PIR        13
#define PIN_BTN_FOTO   12
#define PIN_BTN_MODO    3
#define PIN_BUZZER      1
#define PIN_FLASH       4
#define PIN_LED_INT    33        // LED rojo integrado (enciende con LOW)

#define PIR_COOLDOWN_MS   5000
#define VENTANA_RECONOC_MS 8000  // tiempo para reconocer una cara tras detectar movimiento
#define INTENTOS_DESCONOCIDO 3

// ---------- ESTADO ----------
enum Modo { MODO_FOTO, MODO_SEG };
Modo modo = MODO_FOTO;
WebServer server(80);
Preferences prefs;
uint32_t contador = 0;
bool sdOk = false;
bool pirFoto = false;   // PIR dispara fotos en modo FOTO (apagado por defecto; se activa desde la pagina)
unsigned long ultimaAccion = 0, bloqueoHasta = 0, enrolInicio = 0;
String ultimoResultado = "Listo";

// ---------- RECONOCIMIENTO FACIAL ----------
static mtmn_config_t mtmn_config;
static face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;
bool enrolando = false;
char nombreEnrol[24] = "";
int muestrasFaltan = 0;

// ---------- BUZZER (ACTIVO: el sonido se distingue por cantidad y duracion de pitidos) ----------
void tono(int f, int ms) { (void)f; digitalWrite(PIN_BUZZER, HIGH); delay(ms); digitalWrite(PIN_BUZZER, LOW); }
void pitido(int ms, int pausa = 0) { tono(0, ms); if (pausa) delay(pausa); }
void sonidoAviso()    { pitido(60, 60); pitido(60, 150); }                    // 2 muy cortos: "voy a tomar foto"
void sonidoObturador(){ pitido(200); }                                        // 1 medio: foto lista
void sonidoAcceso()   { pitido(100, 80); pitido(100, 80); pitido(100); }      // 3 rapidos: acceso concedido
void sonidoDenegado() { pitido(1200); }                                       // 1 largo: acceso denegado
void sonidoError()    { for (int i = 0; i < 4; i++) { pitido(150, 100); } }   // 4 cortos: error
void sonidoModo(Modo m) { if (m == MODO_FOTO) pitido(250); else { pitido(250, 150); pitido(250); } } // 1 = foto, 2 = seguridad

// ---------- CAMARA ----------
bool iniciarCamara() {
  camera_config_t c;
  memset(&c, 0, sizeof(c));
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = 5; c.pin_d1 = 18; c.pin_d2 = 19; c.pin_d3 = 21;
  c.pin_d4 = 36; c.pin_d5 = 39; c.pin_d6 = 34; c.pin_d7 = 35;
  c.pin_xclk = 0; c.pin_pclk = 22; c.pin_vsync = 25; c.pin_href = 23;
  c.pin_sscb_sda = 26; c.pin_sscb_scl = 27; c.pin_pwdn = 32; c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_UXGA;      // se reserva memoria para el tamano maximo
  c.jpeg_quality = 10;
  c.fb_count = 2;
  return esp_camera_init(&c) == ESP_OK;
}

void descartar(int n) {
  for (int i = 0; i < n; i++) { camera_fb_t *fb = esp_camera_fb_get(); if (fb) esp_camera_fb_return(fb); }
}

// Foto = UXGA (alta calidad). Seguridad o registro de rostros = QVGA (lo que necesita el reconocimiento).
void ajustarResolucion() {
  sensor_t *s = esp_camera_sensor_get();
  if (modo == MODO_FOTO && !enrolando) { s->set_framesize(s, FRAMESIZE_UXGA); s->set_quality(s, 10); }
  else                                 { s->set_framesize(s, FRAMESIZE_QVGA); s->set_quality(s, 12); }
  delay(150);
  descartar(2);
}

void aplicarModo(Modo m, bool sonar) {
  modo = m;
  digitalWrite(PIN_LED_INT, modo == MODO_SEG ? LOW : HIGH);
  ajustarResolucion();
  if (sonar) sonidoModo(m);
  ultimoResultado = (m == MODO_FOTO) ? "Modo FOTO activo" : "Modo SEGURIDAD activo";
}

// Guarda un JPEG en la SD, con flash opcional. Devuelve la ruta o "" si fallo.
String guardarFoto(const char *pref, bool flash) {
  if (!sdOk) return "";
  if (flash) { digitalWrite(PIN_FLASH, HIGH); delay(200); }
  descartar(2);
  camera_fb_t *fb = esp_camera_fb_get();
  if (flash) digitalWrite(PIN_FLASH, LOW);
  if (!fb) return "";
  contador++; prefs.putUInt("n", contador);
  char ruta[48];
  snprintf(ruta, sizeof(ruta), "/fotos/%s_%04u.jpg", pref, (unsigned)contador);
  File f = SD_MMC.open(ruta, FILE_WRITE);
  if (f) { f.write(fb->buf, fb->len); f.close(); }
  esp_camera_fb_return(fb);
  return String(ruta);
}

void tomarFotoConAviso(const char *pref) {
  sonidoAviso();
  String r = guardarFoto(pref, true);
  if (r == "") { sonidoError(); ultimoResultado = "Error al guardar foto (revisa la SD)"; }
  else { sonidoObturador(); ultimoResultado = "Foto guardada: " + r; }
  ultimaAccion = millis();
}

// ---------- ANALISIS DE ROSTRO ----------
// Devuelve: -1 error, 0 sin cara, 1 cara desconocida, 2 cara reconocida (nombre), 3 muestra de registro tomada
int analizarRostro(String &nombre) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return -1;
  dl_matrix3du_t *img = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  if (!img) { esp_camera_fb_return(fb); return -1; }
  fmt2rgb888(fb->buf, fb->len, fb->format, img->item);
  esp_camera_fb_return(fb);

  int res = 0;
  box_array_t *boxes = face_detect(img, &mtmn_config);
  if (boxes) {
    if (align_face(boxes, img, aligned_face) == ESP_OK) {
      dl_matrix3d_t *face_id = get_face_id(aligned_face);   // en el core 1.0.6 se necesita este paso
      if (face_id) {
        if (enrolando) {
          int8_t left = enroll_face_id_to_flash_with_name(&st_face_list, face_id, nombreEnrol);
          muestrasFaltan = left;
          res = 3;
          if (left <= 0) {
            enrolando = false;
            ultimoResultado = String("Rostro registrado: ") + nombreEnrol;
          }
        } else {
          face_id_node *f = recognize_face_with_name(&st_face_list, face_id);
          if (f) { nombre = String(f->id_name); res = 2; } else res = 1;
        }
        dl_matrix3d_free(face_id);
      }
    }
    free(boxes->score); free(boxes->box); free(boxes->landmark); free(boxes);
  }
  dl_matrix3du_free(img);
  return res;
}

void cicloSeguridad() {
  if (millis() < bloqueoHasta) return;
  if (digitalRead(PIN_PIR) != HIGH) return;

  tono(2000, 60);                       // "te estoy viendo": mira a la camara
  unsigned long t0 = millis();
  int desconocidos = 0;
  while (millis() - t0 < VENTANA_RECONOC_MS) {
    server.handleClient();
    String n;
    int r = analizarRostro(n);
    if (r == 2) {
      ultimoResultado = "Acceso CONCEDIDO: " + n;
      sonidoAcceso();
      break;
    }
    if (r == 1 && ++desconocidos >= INTENTOS_DESCONOCIDO) {
      String ruta = guardarFoto("intruso", true);
      ultimoResultado = "Acceso DENEGADO. Foto: " + ruta;
      sonidoDenegado();
      break;
    }
  }
  bloqueoHasta = millis() + 6000;
}

// ---------- BOTONES ----------
bool boton(int pin) {
  if (digitalRead(pin) == LOW) {
    delay(40);
    if (digitalRead(pin) == LOW) {
      unsigned long t = millis();
      while (digitalRead(pin) == LOW && millis() - t < 1500) delay(10);
      return true;
    }
  }
  return false;
}

// ---------- WEB ----------
const char PAGINA[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>ESP32-CAM Acceso</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:auto;padding:12px;max-width:720px}
button,input{padding:10px;margin:3px;border-radius:6px;border:0;font-size:15px}
button{background:#2d7ff9;color:#fff}.rojo{background:#d33}
#cam{width:100%;max-width:640px;border-radius:8px;background:#000;min-height:120px}
.fila{background:#222;padding:8px;border-radius:6px;margin:4px 0;font-size:14px}
a{color:#7cb8ff;margin-right:10px}
</style></head><body>
<h2>Control de acceso ESP32-CAM</h2>
<p>Modo: <b id="modo">-</b> &nbsp;|&nbsp; Rostros registrados: <b id="ros">0</b><br><span id="ult"></span></p>
<button onclick="f('/modo?m=foto')">Modo FOTO</button>
<button onclick="f('/modo?m=seg')">Modo SEGURIDAD</button>
<button onclick="f('/foto')">Tomar foto</button>
<button id="pirb" onclick="f('/pir')">PIR en modo foto: OFF</button><br>
<img id="cam">
<h3>Rostros</h3>
<input id="nom" placeholder="Nombre"><button onclick="enr()">Registrar rostro</button>
<button class="rojo" onclick="if(confirm('Borrar todos los rostros?'))f('/borrarrostros')">Borrar rostros</button>
<p style="font-size:13px;color:#aaa">Al registrar, mira a la camara de frente: se necesitan 3 muestras (suena un pitido por cada una).</p>
<h3>Galeria</h3><button onclick="lista()">Actualizar</button><div id="gal"></div>
<script>
const $=id=>document.getElementById(id);
function f(u){return fetch(u).then(r=>r.text()).then(()=>{est();})}
function enr(){const n=$('nom').value.trim();if(!n){alert('Escribe un nombre');return}f('/enrolar?n='+encodeURIComponent(n))}
function est(){fetch('/estado').then(r=>r.json()).then(d=>{
 $('modo').textContent=d.modo;$('pirb').textContent='PIR en modo foto: '+(d.pir?'ON':'OFF');$('ros').textContent=d.rostros;
 $('ult').textContent=(d.enrolando?'Registrando... faltan '+d.faltan+' muestras. ':'')+d.ultimo;}).catch(()=>{})}
function cam(){$('cam').src='/cam?t='+Date.now()}
$('cam').onload=()=>setTimeout(cam,400);$('cam').onerror=()=>setTimeout(cam,2000);
function lista(){fetch('/lista').then(r=>r.json()).then(a=>{
 $('gal').innerHTML=a.reverse().map(n=>'<div class="fila">'+n+'<br><a href="/ver?f='+n+'" target="_blank">Ver</a><a href="/ver?f='+n+'&d=1">Descargar</a><a href="#" onclick="del(\''+n+'\');return false">Borrar</a></div>').join('')||'Sin fotos'})}
function del(n){if(confirm('Borrar '+n+'?'))fetch('/borrar?f='+n).then(()=>lista())}
est();setInterval(est,2000);cam();lista();
</script></body></html>
)rawliteral";

bool nombreValido(const String &n) {
  return n.length() > 0 && n.indexOf('/') < 0 && n.indexOf('\\') < 0 && n.indexOf("..") < 0;
}

void rutasWeb() {
  server.on("/", []() { server.send_P(200, "text/html", PAGINA); });

  server.on("/estado", []() {
    String j = "{\"modo\":\"" + String(modo == MODO_FOTO ? "FOTO" : "SEGURIDAD") + "\"";
    j += ",\"rostros\":" + String((int)st_face_list.count);
    j += ",\"pir\":" + String(pirFoto ? "true" : "false");
    j += ",\"enrolando\":" + String(enrolando ? "true" : "false");
    j += ",\"faltan\":" + String(muestrasFaltan > 0 ? muestrasFaltan : 3);
    j += ",\"ultimo\":\"" + ultimoResultado + "\"}";
    server.send(200, "application/json", j);
  });

  server.on("/modo", []() {
    aplicarModo(server.arg("m") == "seg" ? MODO_SEG : MODO_FOTO, true);
    server.send(200, "text/plain", "ok");
  });

  server.on("/pir", []() {
    pirFoto = !pirFoto;
    ultimoResultado = pirFoto ? "PIR activado en modo FOTO" : "PIR desactivado en modo FOTO";
    server.send(200, "text/plain", "ok");
  });

  server.on("/foto", []() {
    if (modo == MODO_FOTO) tomarFotoConAviso("foto");
    else { String r = guardarFoto("manual", true); ultimoResultado = "Foto: " + r; }
    server.send(200, "text/plain", "ok");
  });

  server.on("/cam", []() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { server.send(500, "text/plain", "sin imagen"); return; }
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
  });

  server.on("/lista", []() {
    String j = "[";
    File dir = SD_MMC.open("/fotos");
    if (dir) {
      File f = dir.openNextFile();
      bool primero = true;
      while (f) {
        if (!f.isDirectory()) {
          String n = String(f.name());
          n = n.substring(n.lastIndexOf('/') + 1);
          if (!primero) j += ",";
          j += "\"" + n + "\""; primero = false;
        }
        f = dir.openNextFile();
      }
    }
    j += "]";
    server.send(200, "application/json", j);
  });

  server.on("/ver", []() {
    String n = server.arg("f");
    if (!nombreValido(n)) { server.send(400, "text/plain", "nombre invalido"); return; }
    File f = SD_MMC.open("/fotos/" + n);
    if (!f) { server.send(404, "text/plain", "no existe"); return; }
    if (server.arg("d") == "1") server.sendHeader("Content-Disposition", "attachment; filename=" + n);
    server.streamFile(f, "image/jpeg");
    f.close();
  });

  server.on("/borrar", []() {
    String n = server.arg("f");
    if (!nombreValido(n)) { server.send(400, "text/plain", "nombre invalido"); return; }
    SD_MMC.remove("/fotos/" + n);
    server.send(200, "text/plain", "ok");
  });

  server.on("/enrolar", []() {
    String n = server.arg("n");
    String limpio = "";
    for (unsigned i = 0; i < n.length() && limpio.length() < 20; i++)
      if (isalnum(n[i]) || n[i] == '_' || n[i] == '-') limpio += n[i];
    if (limpio == "") { server.send(400, "text/plain", "nombre invalido"); return; }
    limpio.toCharArray(nombreEnrol, sizeof(nombreEnrol));
    muestrasFaltan = 3;
    enrolando = true;
    enrolInicio = millis();
    ajustarResolucion();
    ultimoResultado = "Registrando a " + limpio;
    server.send(200, "text/plain", "ok");
  });

  server.on("/borrarrostros", []() {
    delete_face_all_in_flash_with_name(&st_face_list);
    ultimoResultado = "Rostros borrados";
    server.send(200, "text/plain", "ok");
  });
}

// ---------- SETUP / LOOP ----------
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);   // evita reinicios por caida de voltaje (usa buena fuente igual)

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_BTN_FOTO, INPUT_PULLUP);
  pinMode(PIN_BTN_MODO, INPUT_PULLUP);
  pinMode(PIN_LED_INT, OUTPUT); digitalWrite(PIN_LED_INT, HIGH);

  pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);

  // microSD en modo 1-bit (libera GPIO 4, 12, 13)
  sdOk = SD_MMC.begin("/sdcard", true);
  if (sdOk) SD_MMC.mkdir("/fotos");
  pinMode(PIN_FLASH, OUTPUT); digitalWrite(PIN_FLASH, LOW);
  if (!sdOk) { tono(300, 300); delay(150); tono(300, 300); delay(150); tono(300, 300); }  // 3 pitidos largos = sin SD

  if (!iniciarCamara()) { sonidoError(); delay(1000); ESP.restart(); }

  prefs.begin("acceso", false);
  contador = prefs.getUInt("n", 0);

  // Reconocimiento facial: hasta 7 rostros, 3 muestras por rostro
  mtmn_config = mtmn_init_config();
  face_id_name_init(&st_face_list, 7, 3);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);

#if USAR_MODO_AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) delay(300);
  if (WiFi.status() == WL_CONNECTED) MDNS.begin("esp32cam");
  else { WiFi.mode(WIFI_AP); WiFi.softAP(AP_SSID, AP_PASS); }
#endif

  rutasWeb();
  server.begin();
  aplicarModo(MODO_FOTO, false);
  sonidoAcceso();                       // melodia de "sistema listo"
}

void loop() {
  server.handleClient();

  if (enrolando) {                      // registro de rostro en curso
    String n;
    int r = analizarRostro(n);
    if (r == 3) {
      if (enrolando) tono(1800, 60);
      else { ajustarResolucion(); sonidoAcceso(); }
    }
    if (enrolando && millis() - enrolInicio > 60000) {
      enrolando = false; ajustarResolucion();
      ultimoResultado = "Registro cancelado (no se detecto rostro)";
      sonidoError();
    }
    return;
  }

  if (boton(PIN_BTN_MODO)) aplicarModo(modo == MODO_FOTO ? MODO_SEG : MODO_FOTO, true);

  if (modo == MODO_FOTO) {
    if (boton(PIN_BTN_FOTO)) tomarFotoConAviso("foto");
    else if (pirFoto && digitalRead(PIN_PIR) == HIGH && millis() - ultimaAccion > PIR_COOLDOWN_MS) tomarFotoConAviso("pir");
  } else {
    cicloSeguridad();
  }
}
