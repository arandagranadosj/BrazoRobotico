// Control del brazo MT8005 con sliders por WiFi y memoria de posicion
//
// El UNO R4 crea su propia red WiFi. Conectar el celular o la compu a la red
// "Brazo_Robotico" (clave robotica123) y abrir http://192.168.4.1
// Ahi hay un slider por junta, un slider de velocidad y botones.
//
// Memoria de posicion: los servos no le dicen al Arduino donde estan. Por eso el
// programa guarda en la EEPROM del UNO la ultima posicion a la que mando cada junta,
// cada vez que el brazo se queda quieto. Al prender, manda esa misma posicion, asi
// el servo no se mueve si el brazo sigue donde quedo. Para que al apagar el brazo no
// se caiga por su peso, antes de apagar se manda a la posicion de reposo, una postura
// que se sostiene sola, y se guarda con el boton "Guardar reposo".
//
// Control desde la compu por USB: abrir interfaz/Control_Brazo.html en Chrome o Edge y tocar
// Conectar. La pagina habla con el Arduino por el mismo cable USB-C (Web Serial).
// Cerrar antes el monitor serial de PlatformIO, solo un programa puede usar el puerto.
//
// Interfaz del proyecto por WiFi: con la compu en la red "Brazo_Robotico" abrir
// http://192.168.4.1/robot  (la pagina la guarda el Arduino, se genera sola al compilar
// a partir de interfaz/Interfaz_Robot.html con scripts/gen_interfaz.py). Por WiFi los comandos llegan
// a /k?t=comando y se atienden igual que por el cable.
//
// Monitor serial a 115200 (respaldo, escribir y Enter):
//   j1 30  mueve J1 a 30 grados     v 15  velocidad 15 grados/s
//   c centro   r reposo   g guardar reposo   e prender   x soltar   ?  estado
//   h1 off esc hol   calibracion de J1 (offset en grados, escala, juego de engranes en grados)
//   p q1 q2 q3 q4 q5   punto de trayectoria (lo usa Interfaz_Robot.html por USB): si cada junta
//                      esta a 10 grados o menos, la sigue a velocidad constante en el
//                      tiempo que paso desde el punto anterior, si no, va con rampa
//   Q_tid_idx_dt_q1.._q5_q1..   lote de puntos (Interfaz_Robot.html por WiFi), se guardan y se
//                      reproducen cada dt ms aunque el WiFi tarde
//   F  detener: vacia los puntos guardados y se queda donde esta
//   t3 1512   centro del canal 3 (J3a) en us, de 1300 a 1700, se guarda en la EEPROM (canales 1 a 6: J1 J2 J3a J3b J4 J5)
//   bw o texto   guarda un trozo de la calibracion de la interfaz desde la posicion o
//   bf largo sello suma   la da por completa si la suma de los bytes coincide
//   br o   devuelve el trozo desde o en una linea "B o texto" (por WiFi es /b?o=...)
//   aire 1 / aire 0   la herramienta agarra o suelta (sin efecto mientras HERRAMIENTA_CONECTADA sea false)

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <WiFiS3.h>
#include "pagina.h"
#include "interfaz_gz.h"
#include "geometria.h"                                     // limites, reposo y pasada de J1, generado desde geometria.json

// ---- copiar aqui los CENTRO que quedaron en Centrar_Servos ----
//                  J1    J2    J3a   J3b   J4    J5
int CENTRO[6] = { 1500, 1500, 1500, 1500, 1500, 1500 };   // us, valores de fabrica, los ajustados con el comando t quedan en la EEPROM

const char *JN[5]      = { "J1 base", "J2 hombro", "J3 codo", "J4 muneca", "J5 giro" };
const float K = 0.09;                                      // grados por us
const float CERO_J1 = 10.0;                                // angulo del servo de J1 (desde CENTRO) donde la base queda viendo al frente, ese es su 0
// El 0 de J1 con que se guardaron las posiciones queda en la EEPROM. Si CERO_J1 cambia, al arrancar se corren
// la posicion guardada para que el brazo no salte y el reposo de J1 queda en 0, recto.
struct Cero { uint32_t marca; float j1; };
const uint32_t MARCA_CERO = 0xCE200001;
const int DIR_CERO = 160;
// Centro de cada canal en us, ajustable desde la interfaz (comando t). Se guarda aparte en la EEPROM.
// Sirve sobre todo para el codo, donde los dos MG996R en espejo se pelean si sus centros no coinciden.
struct Centros { uint32_t marca; int16_t c[6]; };
const uint32_t MARCA_CEN = 0xCE7A0001;
const int DIR_CEN = 200;
// Si el brazo queda quieto en reposo este tiempo, se sueltan los servos para que no calienten ni gasten.
// El reposo se sostiene solo. Iniciar o Ir a reposo en la interfaz los vuelven a prender.
const unsigned long DESCANSO_MS = 180000;
// Calibracion de la interfaz (offsets, ganancias, puntos medidos en el MDF). El firmware no la interpreta,
// la guarda como texto para que cualquier compu o navegador que se conecte la reciba igual.
// La interfaz la manda en trozos (bw) y al final la cierra con su largo, su sello de tiempo y la suma de sus bytes (bf).
// Si se corta a medias la suma no coincide y se toma como que no hay, la interfaz la vuelve a mandar.
struct CabCI { uint32_t marca; uint32_t largo; uint32_t sello; uint32_t suma; };
const uint32_t MARCA_CI = 0xC1A10001;
const int DIR_CI = 256, DIR_CID = DIR_CI + sizeof(CabCI);
const uint32_t MAX_CI = 6000, TROZO_CI = 640;
uint32_t selloCI = 0, largoCI = 0;                         // sello 0: no hay una calibracion completa guardada
bool descanso = false;
bool recuperado = false;                                   // al arrancar se recuperaron los pulsos que seguian en la PCA9685
const int   US_MIN = 500, US_MAX = 2500;
const char *RED = "Brazo_Robotico", *CLAVE = "robotica123";
const uint8_t CANAL_WIFI = 1;                              // si se cae seguido, probar 6 u 11, los canales que no se pisan
char fwWiFi[16] = "?";                                      // version del firmware del modulo WiFi, se lee una vez al arrancar
const char *causaReinicio = "?";                           // por que arranco, se lee de los registros del RA4M1 al arrancar
const int   PIN_RELE = 6;
const bool  RELE_ACTIVO_LOW = true;
// Herramienta neumatica. Mientras HERRAMIENTA_CONECTADA sea false los comandos "aire" se aceptan y se reportan,
// pero el rele no se mueve: asi la interfaz y la tarea se prueban completas sin la bomba. Al conectarla, poner true.
// La valvula de alivio es opcional (-1 sin valvula): al soltar se abre VALVULA_MS para que el objeto caiga enseguida.
const bool  HERRAMIENTA_CONECTADA = false;
const int   PIN_VALVULA = -1;
const bool  VALVULA_ACTIVA_LOW = true;
const unsigned long VALVULA_MS = 400;
const unsigned long AIRE_MAX_MS = 90000;                   // la bomba nunca queda prendida mas de 90 s seguidos
bool aire = false;                                         // la herramienta esta agarrando
unsigned long tAire = 0, tValvula = 0;
const int PROTOCOLO = 4;                                   // version de los comandos, la interfaz avisa si espera otra

const uint8_t PCA = 0x40, PRESCALE = 131;                  // 50 Hz con oscilador de 27 MHz
const float   US_POR_TICK = (PRESCALE + 1) * 1e6f / 27e6f;

struct Memoria { uint32_t marca; float pos[5]; float rep[5]; float vel; };
const uint32_t MARCA = 0xB2A50002;
Memoria mem;

// calibracion de cada junta, se guarda aparte en la EEPROM para no borrar la posicion guardada
// angulo que recibe el servo = esc*pos + off + dir*hol/2, con dir el sentido del ultimo movimiento
// hol es el juego de los engranes, el servo se pasa medio juego hacia donde venia girando
struct Cal { uint32_t marca; float off[5]; float esc[5]; float hol[5]; };
const uint32_t MARCA_CAL = 0xCA1B0001;
const int DIR_CAL = 64;
Cal cal;
int8_t dirJ[5] = { 0, 0, 0, 0, 0 };
// J1 siempre termina girando hacia el mismo lado, asi los engranes quedan apoyados del mismo lado y no a medias.
// Si el objetivo pide llegar al reves, primero se pasa J1_PASO grados y despues regresa al objetivo.
float finJ1 = NAN;                          // objetivo final de J1 mientras hace la pasada

float pos[5], meta[5], w[5];               // posicion mandada, objetivo, velocidad actual
float wp[5];                               // velocidad de seguimiento de puntos p
bool  seguir[5] = { false, false, false, false, false };
unsigned long tP = 0;
float vel = 15;                            // grados/s
bool  fuerza = false, guardado = true;
unsigned long tUlt = 0, tQuieto = 0;
WiFiServer server(80);
char linea[1000]; size_t lLin = 0;                       // linea que llega por USB
bool reportar = false;                     // la pagina por USB pidio estado continuo
unsigned long tRep = 0;
void guardar();
void vigilarHerramienta();
void vaciarBufer();
bool procesar(String c);

// ---------------- PCA9685 ----------------
bool escribir(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
int leer(uint8_t reg) {
  Wire.beginTransmission(PCA); Wire.write(reg);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom((uint8_t)PCA, (size_t)1) != 1) return -1;
  return Wire.read();
}
void canalUs(uint8_t c, float us) {
  int u = constrain((int)lround(us), US_MIN, US_MAX);
  uint16_t t = (uint16_t)lround(u / US_POR_TICK);
  uint8_t b = 0x06 + 4 * c;
  escribir(b, 0); escribir(b + 1, 0); escribir(b + 2, t & 0xFF); escribir(b + 3, (t >> 8) & 0x0F);
}
void canalOff(uint8_t c) { escribir(0x06 + 4 * c + 3, 0x10); }
// Si la PCA9685 siguio encendida mientras el Arduino se reiniciaba (caida de voltaje, boton de reset o subir el codigo),
// todavia tiene el pulso de cada servo. Se leen y se pasan a angulos, asi el brazo sigue sosteniendose donde estaba
// y no salta a la ultima posicion guardada. Si la PCA esta dormida, sin configurar o con algun canal apagado, no se usa.
bool recuperarPCA() {
  int m1 = leer(0x00), pre = leer(0xFE);
  if (m1 < 0 || (m1 & 0x10) || pre != PRESCALE) return false;
  float u[6];
  for (uint8_t c = 0; c < 6; c++) {
    uint8_t b = 0x06 + 4 * c;
    int onL = leer(b), onH = leer(b + 1), offL = leer(b + 2), offH = leer(b + 3);
    if (onL < 0 || onH < 0 || offL < 0 || offH < 0) return false;
    if ((offH & 0x10) || (onH & 0x1F) || onL) return false;   // canal apagado o con un pulso que no mando este programa
    u[c] = (((offH & 0x0F) << 8) | offL) * US_POR_TICK;
    if (u[c] < US_MIN - 10 || u[c] > US_MAX + 10) return false;
  }
  if (fabs((u[2] - CENTRO[2]) + (u[3] - CENTRO[3])) > 15) return false;   // los dos servos del codo no cuadran
  const uint8_t canal[5] = { 0, 1, 2, 4, 5 };
  for (uint8_t j = 0; j < 5; j++) {
    float d = u[canal[j]] - CENTRO[canal[j]] - (j == 0 ? CERO_J1 / K : 0.0f);
    pos[j] = constrain((d * K - cal.off[j]) / cal.esc[j], LIM_MIN[j], LIM_MAX[j]);
    meta[j] = pos[j]; w[j] = 0; seguir[j] = false; dirJ[j] = 0;
  }
  fuerza = true; descanso = false; finJ1 = NAN; guardado = false;
  return true;
}
void iniciarPCA() {
  escribir(0x00, 0x10); escribir(0xFE, PRESCALE); escribir(0x01, 0x04);
  escribir(0xFD, 0x10); escribir(0x00, 0x00); delay(2);
}

// ---------------- juntas ----------------
void mandar(uint8_t j) {
  if (!fuerza) return;
  float d = (cal.esc[j] * pos[j] + cal.off[j] + dirJ[j] * cal.hol[j] * 0.5f) / K;
  switch (j) {
    case 0: canalUs(0, CENTRO[0] + d + CERO_J1 / K); break;
    case 1: canalUs(1, CENTRO[1] + d); break;
    case 2: canalUs(2, CENTRO[2] + d); canalUs(3, CENTRO[3] - d); break;   // codo en espejo
    case 3: canalUs(4, CENTRO[4] + d); break;
    case 4: canalUs(5, CENTRO[5] + d); break;
  }
}
void prender() {                           // manda la posicion guardada, una junta a la vez
  descanso = false;
  fuerza = true; finJ1 = NAN; vaciarBufer();
  for (uint8_t j = 0; j < 5; j++) { meta[j] = pos[j]; w[j] = 0; seguir[j] = false; mandar(j); delay(150); }
}
void soltar() { fuerza = false; finJ1 = NAN; vaciarBufer(); escribir(0xFD, 0x10); for (uint8_t j = 0; j < 5; j++) { meta[j] = pos[j]; w[j] = 0; seguir[j] = false; } }
void ir(uint8_t j, float g, bool pasada = true) {
  seguir[j] = false; g = constrain(g, LIM_MIN[j], LIM_MAX[j]);
  if (j == 0) {
    finJ1 = NAN;
    float m = g - J1_LADO * J1_PASO;
    if (pasada && (g - pos[0]) * J1_LADO < -0.5f && m >= LIM_MIN[0] && m <= LIM_MAX[0]) { meta[0] = m; finJ1 = g; return; }
  }
  meta[j] = g;
}

// punto de trayectoria: cada junta llega al punto justo cuando deberia llegar el siguiente,
// asi el movimiento sale continuo aunque los puntos lleguen cada 40 ms (USB) o cada 100 ms (WiFi)
void punto(const float q[5]) {
  if (!fuerza) return;
  unsigned long t = millis();
  float per = constrain((t - tP) / 1000.0f, 0.02f, 0.25f);
  tP = t;
  for (uint8_t j = 0; j < 5; j++) {
    float g = constrain(q[j], LIM_MIN[j], LIM_MAX[j]), d = fabs(g - pos[j]);
    if (j == 0) finJ1 = NAN;
    if (d <= 10.0f) { meta[j] = g; wp[j] = d / per; w[j] = 0; seguir[j] = true; }
    else ir(j, g, false);
  }
}

// ---------------- bufer de puntos por WiFi ----------------
// Por WiFi la interfaz manda la trayectoria de a varios puntos por pedido: Q_tid_idx_dt_q1_q2_q3_q4_q5_q1_...
// tid identifica la trayectoria, idx es el numero del primer punto y dt los ms entre puntos.
// El brazo los reproduce cada dt aunque el WiFi tarde. Si un pedido se repite, los puntos que ya tiene se ignoran,
// y si el WiFi se cae el brazo termina lo que tiene y espera quieto en el ultimo punto hasta que lleguen mas.
const uint8_t NB = 64;
float bq[NB][5];
uint8_t bIni = 0, bCnt = 0;
uint32_t bTid = 0, bSig = 0;                // trayectoria actual y numero del siguiente punto que espera
uint16_t bDt = 50;
unsigned long tPop = 0;
void vaciarBufer() { bCnt = 0; bIni = 0; }
// se lee directo del texto con strtod, sin crear Strings. Con cientos de pedidos los String sueltos
// fragmentan la memoria del UNO R4 y despues de un rato el programa se puede trabar
void lote(const char *c) {
  if (!fuerza) return;
  static float v[3 + 5 * 16]; int n = 0;
  const char *t = c + 2;
  while (n < 3 + 5 * 16 && *t) {
    char *fin; float x = strtod(t, &fin);
    if (fin == t) break;
    v[n++] = x; t = fin;
    if (*t == '_') t++; else break;
  }
  if (n < 8) return;
  uint32_t tid = (uint32_t)v[0], idx = (uint32_t)v[1];
  if (tid != bTid) { bTid = tid; bSig = 0; vaciarBufer(); }
  bDt = constrain((int)v[2], 20, 250);
  int np = (n - 3) / 5;
  for (int k = 0; k < np; k++) {
    uint32_t g = idx + k;
    if (g < bSig) continue;                 // ya lo tenia, el pedido se repitio
    if (g > bSig || bCnt >= NB) break;      // falta un punto anterior o el bufer esta lleno, la interfaz reenvia desde bSig
    uint8_t p = (bIni + bCnt) % NB;
    for (uint8_t j = 0; j < 5; j++) bq[p][j] = v[3 + 5 * k + j];
    bCnt++; bSig++;
  }
}
void reproducir() {
  if (!bCnt || !fuerza) return;
  unsigned long t = millis();
  if (t - tPop < bDt) return;
  tPop = t;
  punto(bq[bIni]);
  bIni = (bIni + 1) % NB; bCnt--;
}

// perfil trapezoidal: acelera en 0.5 s hasta vel y frena a tiempo para llegar sin golpe
void actualizar() {
  vigilarHerramienta();
  reproducir();
  unsigned long t = millis();
  if (t - tUlt < 20) return;
  float dt = (t - tUlt) / 1000.0f; tUlt = t;
  float A = vel * 2;
  bool quieto = true;
  for (uint8_t j = 0; j < 5; j++) {
    float e = meta[j] - pos[j];
    if (fabs(e) < 0.01f) {
      w[j] = 0;
      if (j == 0 && !isnan(finJ1)) { meta[0] = finJ1; finJ1 = NAN; quieto = false; }   // termina la pasada de J1
      continue;
    }
    quieto = false;
    if (seguir[j]) {                                       // seguimiento de puntos p
      float paso = max(wp[j], 1.0f) * dt;
      dirJ[j] = e > 0 ? 1 : -1;
      if (paso >= fabs(e)) { pos[j] = meta[j]; seguir[j] = false; } else pos[j] += (e > 0 ? paso : -paso);
      mandar(j); continue;
    }
    w[j] = min(min(w[j] + A * dt, vel), sqrtf(2 * A * fabs(e)));
    float paso = w[j] * dt;
    dirJ[j] = e > 0 ? 1 : -1;
    if (paso >= fabs(e)) { pos[j] = meta[j]; w[j] = 0; } else pos[j] += (e > 0 ? paso : -paso);
    mandar(j);
  }
  if (!quieto) { tQuieto = t; guardado = false; }
  if (!guardado && t - tQuieto > 1500) guardar();
  if (fuerza && !bCnt && t - tQuieto > DESCANSO_MS) {      // descanso: quieto en reposo varios minutos
    bool rep = true;
    for (uint8_t j = 0; j < 5; j++) if (fabs(pos[j] - mem.rep[j]) > 0.5f) rep = false;
    if (rep) { soltar(); descanso = true; Serial.println(F("Servos en descanso, el brazo quedo quieto en reposo.")); }
  }
}

// ---------------- memoria ----------------
void guardar() {
  bool cambio = fabs(mem.vel - vel) > 0.5f;
  for (uint8_t j = 0; j < 5; j++) if (fabs(mem.pos[j] - pos[j]) > 0.3f) cambio = true;
  if (cambio) {
    for (uint8_t j = 0; j < 5; j++) mem.pos[j] = pos[j];
    mem.vel = vel; mem.marca = MARCA;
    EEPROM.put(0, mem);
  }
  guardado = true;
}
void cargar() {
  EEPROM.get(0, mem);
  bool ok = mem.marca == MARCA && mem.vel >= 1 && mem.vel <= 60;
  for (uint8_t j = 0; j < 5 && ok; j++)
    if (!(mem.pos[j] >= LIM_MIN[j] && mem.pos[j] <= LIM_MAX[j])) ok = false;
  if (!ok) {
    mem.marca = MARCA; mem.vel = 15;
    for (uint8_t j = 0; j < 5; j++) { mem.pos[j] = REPOSO_DEF[j]; mem.rep[j] = REPOSO_DEF[j]; }
    EEPROM.put(0, mem);
    Serial.println(F("Sin posicion guardada: arranco en la posicion de reposo."));
  }
  Cero z; EEPROM.get(DIR_CERO, z);
  float antes = z.marca == MARCA_CERO ? z.j1 : 5.0f;        // antes de guardarlo el firmware usaba 5 grados
  if (fabs(antes - CERO_J1) > 0.01f) {                       // 0 nuevo: misma posicion fisica, reposo de J1 recto
    mem.pos[0] = constrain(mem.pos[0] + antes - CERO_J1, LIM_MIN[0], LIM_MAX[0]);
    mem.rep[0] = 0;
    EEPROM.put(0, mem);
    z.marca = MARCA_CERO; z.j1 = CERO_J1; EEPROM.put(DIR_CERO, z);
    Serial.println(F("0 de J1 nuevo: posicion guardada corrida y reposo de J1 en 0."));
  }
  for (uint8_t j = 0; j < 5; j++) { pos[j] = meta[j] = mem.pos[j]; w[j] = 0; }
  vel = mem.vel;
}
void cargarCal() {
  EEPROM.get(DIR_CAL, cal);
  bool ok = cal.marca == MARCA_CAL;
  for (uint8_t j = 0; j < 5 && ok; j++)
    if (!(fabs(cal.off[j]) <= 20 && cal.esc[j] >= 0.8f && cal.esc[j] <= 1.2f && cal.hol[j] >= 0 && cal.hol[j] <= 10)) ok = false;
  if (!ok) {
    cal.marca = MARCA_CAL;
    for (uint8_t j = 0; j < 5; j++) { cal.off[j] = 0; cal.esc[j] = 1; cal.hol[j] = 0; }
    EEPROM.put(DIR_CAL, cal);
  }
}

// ---------------- herramienta ----------------
void salida(int pin, bool activo, bool activoLow) { if (pin >= 0) digitalWrite(pin, activo != activoLow ? HIGH : LOW); }
void herramienta(bool agarra) {
  aire = agarra; tAire = millis();
  if (!HERRAMIENTA_CONECTADA) return;
  salida(PIN_RELE, agarra, RELE_ACTIVO_LOW);
  if (!agarra && PIN_VALVULA >= 0) { salida(PIN_VALVULA, true, VALVULA_ACTIVA_LOW); tValvula = millis(); }
}
void vigilarHerramienta() {
  if (aire && millis() - tAire > AIRE_MAX_MS) { herramienta(false); Serial.println(F("Bomba apagada: llevaba demasiado tiempo prendida.")); }
  if (tValvula && millis() - tValvula > VALVULA_MS) { salida(PIN_VALVULA, false, VALVULA_ACTIVA_LOW); tValvula = 0; }
}

// ---------------- calibracion de la interfaz ----------------
uint32_t sumaCI(uint32_t n) { uint32_t s = 0; for (uint32_t i = 0; i < n; i++) s += EEPROM.read(DIR_CID + i); return s; }
void cargarCI() {
  CabCI c; EEPROM.get(DIR_CI, c);
  if (c.marca == MARCA_CI && c.largo <= MAX_CI && sumaCI(c.largo) == c.suma) { selloCI = c.sello; largoCI = c.largo; }
  else { selloCI = 0; largoCI = 0; }
}
// se escribe de a 256 bytes con put, que borra y graba cada bloque de la flash una sola vez
struct Pieza { uint8_t b[256]; };
void escribirCI(uint32_t o, const char *t, uint32_t n) {
  static Pieza pz;
  for (uint32_t i = 0; i < n; ) {
    uint32_t k = min((uint32_t)sizeof(pz.b), n - i);
    EEPROM.get(DIR_CID + o + i, pz); memcpy(pz.b, t + i, k); EEPROM.put(DIR_CID + o + i, pz);
    i += k; actualizar();
  }
}
// trozo de la calibracion desde o, en out
const char *leerCI(uint32_t o, char *out, size_t tam) {
  uint32_t n = 0;
  if (selloCI && o < largoCI) { n = min(largoCI - o, min(TROZO_CI, (uint32_t)tam - 1)); for (uint32_t i = 0; i < n; i++) out[i] = EEPROM.read(DIR_CID + o + i); }
  out[n] = 0; return out;
}
// bw, bf y br se leen directo del texto, sin String: los trozos son de cientos de caracteres y los String grandes
// fragmentan la memoria del UNO R4. Devuelve true si el comando pide responder con el estado.
bool esComandoCI(const char *c) { return c[0] == 'b' && (c[1] == 'w' || c[1] == 'f' || c[1] == 'r') && c[2] == ' '; }
bool comandoCI(const char *c) {
  char *t;
  if (c[1] == 'w') {                                       // bw o texto
    uint32_t o = strtoul(c + 3, &t, 10);
    if (*t == ' ') {
      t++; uint32_t n = strlen(t);
      while (n && (t[n - 1] == ' ' || t[n - 1] == '\r')) n--;
      if (o + n <= MAX_CI && n <= TROZO_CI) { selloCI = 0; largoCI = 0; escribirCI(o, t, n); }
    }
    return true;
  }
  if (c[1] == 'f') {                                       // bf largo sello suma
    uint32_t largo = strtoul(c + 3, &t, 10), sello = strtoul(t, &t, 10), suma = strtoul(t, &t, 10);
    if (largo <= MAX_CI && sello && sumaCI(largo) == suma) {
      CabCI cab = { MARCA_CI, largo, sello, suma }; EEPROM.put(DIR_CI, cab);
      selloCI = sello; largoCI = largo;
    }
    return true;
  }
  static char trozo[TROZO_CI + 1];                         // br o: el trozo en su propia linea, por el cable
  uint32_t o = strtoul(c + 3, NULL, 10);
  Serial.print(F("B ")); Serial.print(o); Serial.print(' '); Serial.println(leerCI(o, trozo, sizeof(trozo)));
  return false;
}

// ---------------- comandos ----------------
void comando(const String &c) {
  if (c == "centro") for (uint8_t j = 0; j < 5; j++) ir(j, 0);
  else if (c == "reposo") for (uint8_t j = 0; j < 5; j++) ir(j, mem.rep[j]);
  else if (c == "guardar") {                                // con la pasada de J1 en curso se guarda su objetivo final, no el de la pasada
    for (uint8_t j = 0; j < 5; j++) mem.rep[j] = (j == 0 && !isnan(finJ1)) ? finJ1 : meta[j];
    EEPROM.put(0, mem);
  }
  else if (c == "prender") { if (!fuerza) prender(); }
  else if (c == "soltar") soltar();
}
// estado en un bufer fijo, se arma igual por USB y por WiFi sin pedir memoria nueva cada vez
char ej[760];
char *ponI(char *p, long v) { if (v < 0) { *p++ = '-'; v = -v; } char t[12]; int k = 0; do { t[k++] = '0' + v % 10; v /= 10; } while (v); while (k) *p++ = t[--k]; return p; }
char *ponF(char *p, float v, uint8_t dec) {
  if (isnan(v) || isinf(v)) v = 0;
  long esc = 1; for (uint8_t i = 0; i < dec; i++) esc *= 10;
  long n = lroundf(v * esc);
  if (n < 0) { *p++ = '-'; n = -n; }
  p = ponI(p, n / esc);
  if (dec) { *p++ = '.'; long f = n % esc; for (long d = esc / 10; d >= 1; d /= 10) *p++ = '0' + (f / d) % 10; }
  return p;
}
char *ponS(char *p, const char *t) { while (*t) *p++ = *t++; return p; }
char *ponV(char *p, const char *k, const float *v, uint8_t dec, bool j1fin = false) {
  p = ponS(p, k); *p++ = '[';
  for (uint8_t j = 0; j < 5; j++) { p = ponF(p, (j1fin && j == 0 && !isnan(finJ1)) ? finJ1 : v[j], dec); *p++ = j < 4 ? ',' : ']'; }
  return p;
}
const char *estadoJSON() {
  char *p = ej;
  p = ponV(p, "{\"p\":", pos, 1);
  p = ponV(p, ",\"m\":", meta, 1, true);                    // con la pasada de J1 se reporta el objetivo final
  p = ponV(p, ",\"r\":", mem.rep, 1);
  p = ponV(p, ",\"lo\":", LIM_MIN, 0);
  p = ponV(p, ",\"hi\":", LIM_MAX, 0);
  p = ponV(p, ",\"co\":", cal.off, 2);
  p = ponV(p, ",\"ce\":", cal.esc, 4);
  p = ponV(p, ",\"ch\":", cal.hol, 2);
  p = ponS(p, ",\"z1\":"); p = ponF(p, CERO_J1, 1);
  p = ponS(p, ",\"b\":"); p = ponI(p, bCnt);
  p = ponS(p, ",\"n\":"); p = ponI(p, bSig);
  p = ponS(p, ",\"t\":"); p = ponI(p, bTid);
  p = ponS(p, ",\"up\":"); p = ponI(p, millis() / 100);    // decimas de segundo desde que arranco, si baja es que se reinicio
  p = ponS(p, ",\"rc\":\""); p = ponS(p, causaReinicio);    // por que arranco la ultima vez
  p = ponS(p, "\",\"fw\":\""); p = ponS(p, fwWiFi); p = ponS(p, "\",\"fwn\":\""); p = ponS(p, WIFI_FIRMWARE_LATEST_VERSION);
  p = ponS(p, "\",\"v\":"); p = ponF(p, vel, 0);
  p = ponS(p, ",\"on\":"); *p++ = fuerza ? '1' : '0';
  p = ponS(p, ",\"g\":"); *p++ = guardado ? '1' : '0';
  p = ponS(p, ",\"dz\":"); *p++ = descanso ? '1' : '0';
  p = ponS(p, ",\"rp\":"); *p++ = recuperado ? '1' : '0';
  p = ponS(p, ",\"pv\":"); p = ponI(p, PROTOCOLO);
  p = ponS(p, ",\"gv\":\""); p = ponS(p, GEO_HASH); *p++ = '"';
  p = ponS(p, ",\"ai\":"); *p++ = aire ? '1' : '0';
  p = ponS(p, ",\"hc\":"); *p++ = HERRAMIENTA_CONECTADA ? '1' : '0';
  p = ponS(p, ",\"bs\":"); p = ponI(p, (long)selloCI);  // sello de la calibracion de la interfaz guardada, 0 si no hay
  p = ponS(p, ",\"ct\":["); for (uint8_t c = 0; c < 6; c++) { p = ponI(p, CENTRO[c]); *p++ = c < 5 ? ',' : ']'; }
  *p++ = '}'; *p = 0;
  return ej;
}
// valor de k en la consulta q, decodificado en out (+ a espacio y %XX a su caracter). Vacio si no esta
const char *parametro(const char *q, const char *k, char *out, size_t tam) {
  size_t lk = strlen(k); out[0] = 0;
  for (const char *t = q; t && *t; ) {
    if (!strncmp(t, k, lk) && t[lk] == '=') {
      t += lk + 1; size_t i = 0;
      while (*t && *t != '&' && i + 1 < tam) {
        char ch = *t++;
        if (ch == '+') ch = ' ';
        else if (ch == '%' && isxdigit((unsigned char)t[0]) && isxdigit((unsigned char)t[1])) { char h[3] = { t[0], t[1], 0 }; ch = (char)strtol(h, NULL, 16); t += 2; }
        out[i++] = ch;
      }
      out[i] = 0; return out;
    }
    t = strchr(t, '&'); if (t) t++;
  }
  return out;
}

// ---------------- servidor web ----------------
char req[1000], val[960];                                  // pedido y valor decodificado, fijos para no fragmentar la memoria
void atenderWeb() {
  WiFiClient cl = server.available();
  if (!cl) return;
  size_t lr = 0; bool completa = false;
  unsigned long t0 = millis();
  // primera linea del pedido. Los lotes de puntos traen varios cientos de caracteres y el modulo WiFi los entrega
  // de a poco, por eso se espera hasta 1.5 s moviendo los servos mientras tanto
  while (cl.connected() && millis() - t0 < 1500) {
    int n = cl.available();
    if (n > 0) {
      while (n-- > 0) { int ch = cl.read(); if (ch < 0) break; if (ch == '\n') { completa = true; break; } if (ch != '\r' && lr + 1 < sizeof(req)) req[lr++] = ch; }
      if (completa) break;
    } else actualizar();
  }
  req[lr] = 0;
  { uint8_t basura[64]; unsigned long t1 = millis(); while (cl.available() && millis() - t1 < 200) cl.read(basura, sizeof(basura)); }   // descarta los encabezados
  char *a = strchr(req, ' '), *b = a ? strchr(a + 1, ' ') : NULL;
  if (!completa || !a || !b) {                              // linea incompleta: que la interfaz lo vuelva a pedir
    cl.print(F("HTTP/1.1 408 Request Timeout\r\nConnection: close\r\n\r\n")); cl.stop(); return;
  }
  *b = 0; char *ruta = a + 1, *q = strchr(ruta, '?');
  if (q) *q++ = 0; else q = ruta + strlen(ruta);
  const char *cuerpo = "{}"; uint8_t tipo = 0;              // 0 json, 1 html, 2 gz, 3 texto
  if (!strcmp(ruta, "/m")) {
    int j = atoi(parametro(q, "j", val, sizeof(val)));
    if (j >= 0 && j < 5 && fuerza) ir(j, atof(parametro(q, "v", val, sizeof(val))));
  }
  else if (!strcmp(ruta, "/v")) { vel = constrain(atof(parametro(q, "v", val, sizeof(val))), 2.0f, 60.0f); guardado = false; }
  else if (!strcmp(ruta, "/c")) comando(String(parametro(q, "c", val, sizeof(val))));
  else if (!strcmp(ruta, "/e")) cuerpo = estadoJSON();
  else if (!strcmp(ruta, "/k")) {
    parametro(q, "t", val, sizeof(val));
    bool r;
    if (val[0] == 'Q' && val[1] == '_') { lote(val); r = true; }   // lotes sin String, son la mayoria de los pedidos
    else if (esComandoCI(val)) r = comandoCI(val);
    else r = procesar(String(val));
    if (r) cuerpo = estadoJSON();
  }
  else if (!strcmp(ruta, "/b")) { static char trozo[TROZO_CI + 1]; cuerpo = leerCI(strtoul(parametro(q, "o", val, sizeof(val)), NULL, 10), trozo, sizeof(trozo)); tipo = 3; }
  else if (!strcmp(ruta, "/")) tipo = 1;
  else if (!strcmp(ruta, "/robot")) tipo = 2;
  else { cl.print(F("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n")); cl.stop(); return; }
  cl.print(F("HTTP/1.1 200 OK\r\nContent-Type: "));
  if (tipo == 2) cl.print(F("text/html; charset=utf-8\r\nContent-Encoding: gzip"));
  else if (tipo == 1) cl.print(F("text/html"));
  else if (tipo == 3) cl.print(F("text/plain"));
  else cl.print(F("application/json"));
  if (tipo == 2) { cl.print(F("\r\nContent-Length: ")); cl.print(INTERFAZ_GZ_LEN); }
  cl.print(F("\r\nAccess-Control-Allow-Origin: *\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"));
  const uint8_t *dat = nullptr; size_t n = 0;
  if (tipo == 1) { dat = (const uint8_t *)PAGINA; n = strlen(PAGINA); }
  else if (tipo == 2) { dat = INTERFAZ_GZ; n = INTERFAZ_GZ_LEN; }
  if (dat) for (size_t i = 0; i < n; i += 512) { cl.write(dat + i, min((size_t)512, n - i)); actualizar(); }
  else cl.write((const uint8_t *)cuerpo, strlen(cuerpo));
  cl.flush();
  cl.stop();
}

// comandos de texto, los mismos por el cable USB y por WiFi (/k?t=...)
// devuelve true si el comando pide responder con el estado
bool procesar(String c) {
  c.trim();
  if (c.startsWith("p ")) {                                 // punto de trayectoria
    float q[5]; int n = 0, i = 2;
    while (n < 5 && i < (int)c.length()) {
      int f = c.indexOf(' ', i); if (f < 0) f = c.length();
      q[n++] = c.substring(i, f).toFloat(); i = f + 1;
    }
    if (n == 5) punto(q);
    return false;                                           // sin respuesta, para no saturar el USB
  }
  if (c.startsWith("Q_")) { lote(c.c_str()); return true; }         // lote de puntos, responde con el estado
  if (c == "F") {                                           // detener: vacia el bufer y se queda donde esta
    vaciarBufer(); finJ1 = NAN;
    for (uint8_t j = 0; j < 5; j++) { meta[j] = pos[j]; w[j] = 0; seguir[j] = false; }
    return true;
  }
  if (c.length() >= 2 && c[0] == 'j') {
    int j = c[1] - '1';
    if (j >= 0 && j < 5 && fuerza) ir(j, c.substring(2).toFloat());
  } else if (c.length() >= 2 && c[0] == 'h') {             // h1 off esc hol, calibracion de una junta
    int j = c[1] - '1';
    float v[3]; int n = 0, i = 2;
    while (n < 3 && i < (int)c.length()) {
      while (i < (int)c.length() && c[i] == ' ') i++;
      if (i >= (int)c.length()) break;
      int f = c.indexOf(' ', i); if (f < 0) f = c.length();
      v[n++] = c.substring(i, f).toFloat(); i = f + 1;
    }
    if (j >= 0 && j < 5 && n == 3 && fabs(v[0]) <= 20 && v[1] >= 0.8f && v[1] <= 1.2f && v[2] >= 0 && v[2] <= 10) {
      cal.off[j] = v[0]; cal.esc[j] = v[1]; cal.hol[j] = v[2];
      EEPROM.put(DIR_CAL, cal);
      mandar(j);
    }
  } else if (c.startsWith("v ")) { vel = constrain(c.substring(2).toFloat(), 2.0f, 60.0f); guardado = false; }
  else if (c == "c") comando("centro");
  else if (c == "r") comando("reposo");
  else if (c == "g") comando("guardar");
  else if (c == "e") comando("prender");
  else if (c == "x") comando("soltar");
  else if (c.length() >= 3 && c[0] == 't' && c[1] >= '1' && c[1] <= '6') {   // t3 1512: centro del canal 3 (J3a) en us
    uint8_t k = c[1] - '1';
    int v = c.substring(2).toInt();
    if (v >= 1300 && v <= 1700) {
      CENTRO[k] = v;
      Centros cc; cc.marca = MARCA_CEN; for (uint8_t i = 0; i < 6; i++) cc.c[i] = CENTRO[i];
      EEPROM.put(DIR_CEN, cc);
      const uint8_t junta[6] = { 0, 1, 2, 2, 3, 4 };
      mandar(junta[k]);
    }
  }
  else if (c == "aire 1") herramienta(true);              // la herramienta agarra
  else if (c == "aire 0") herramienta(false);             // la herramienta suelta
  else if (c == "s1") reportar = true;
  else if (c == "s0") reportar = false;
  return c.length() > 0;
}

void serialRespaldo() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') { if (lLin + 1 < sizeof(linea)) linea[lLin++] = ch; continue; }
    linea[lLin] = 0; lLin = 0;
    bool r;
    if (linea[0] == 'Q' && linea[1] == '_') { lote(linea); r = true; }
    else if (esComandoCI(linea)) r = comandoCI(linea);
    else r = procesar(String(linea));
    if (r) { Serial.print(F("E ")); Serial.println(estadoJSON()); }
  }
}

// Causa del arranque segun los registros de reinicio del RA4M1. "voltaje" casi siempre es la fuente de los servos
// que baja de golpe y arrastra al Arduino, "encendido" es que se conecto o se corto la alimentacion del todo
void leerCausa() {
  uint8_t r0 = R_SYSTEM->RSTSR0, r1 = R_SYSTEM->RSTSR1;
  if (r0 & 0x0E) causaReinicio = "voltaje";                 // LVD0, LVD1 o LVD2: el voltaje bajo del limite
  else if (r0 & 0x01) causaReinicio = "encendido";          // POR
  else if (r1 & 0x03) causaReinicio = "perro";              // watchdog
  else if (r1 & 0x04) causaReinicio = "programa";           // reinicio por software, por ejemplo al subir el codigo
  else causaReinicio = "boton";                             // pin de reset
  R_SYSTEM->PRCR = 0xA502; R_SYSTEM->RSTSR0 = 0; R_SYSTEM->RSTSR1 = 0; R_SYSTEM->PRCR = 0xA500;
}
void setup() {
  leerCausa();
  pinMode(PIN_RELE, OUTPUT);
  digitalWrite(PIN_RELE, RELE_ACTIVO_LOW ? HIGH : LOW);     // bomba apagada
  if (PIN_VALVULA >= 0) { pinMode(PIN_VALVULA, OUTPUT); salida(PIN_VALVULA, false, VALVULA_ACTIVA_LOW); }
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(100000);
  while (leer(0x00) < 0) { Serial.println(F("No detecto la PCA9685. Revisar VCC, GND, SDA y SCL.")); delay(2000); }
  cargar();
  cargarCal();
  cargarCI();
  { Centros cc; EEPROM.get(DIR_CEN, cc);
    if (cc.marca == MARCA_CEN) for (uint8_t i = 0; i < 6; i++) if (cc.c[i] >= 1300 && cc.c[i] <= 1700) CENTRO[i] = cc.c[i]; }
  // Arranque seguro, en este orden:
  // 1. Si la PCA9685 todavia tiene los pulsos, el brazo sigue donde estaba, sin moverse.
  // 2. Si no, y el reinicio fue inesperado (voltaje, perro guardian o boton), el brazo puede estar en cualquier lado
  //    y darle fuerza lo haria saltar a la ultima posicion guardada, asi que queda suelto hasta que se toque Prender.
  // 3. Al encender normal o al subir el codigo arranca con fuerza en la posicion guardada, como antes.
  if (recuperarPCA()) {
    recuperado = true;
    Serial.println(F("La PCA9685 seguia con los pulsos: el brazo sigue sosteniendose donde estaba."));
  } else {
    iniciarPCA();
    if (!strcmp(causaReinicio, "voltaje") || !strcmp(causaReinicio, "perro") || !strcmp(causaReinicio, "boton")) {
      // Prender lo lleva al reposo, la postura a la que se le pide acercarlo a mano antes de prender
      for (uint8_t j = 0; j < 5; j++) { pos[j] = meta[j] = mem.rep[j]; w[j] = 0; }
      Serial.println(F("Arranque despues de un reinicio inesperado: servos sueltos hasta tocar Prender, que lo lleva al reposo."));
    }
    else prender();                                         // de una vez en la posicion guardada
  }
  delay(1500);
  Serial.println(F("\n=== Brazo MT8005, control por sliders ==="));
  Serial.print(F("Arranque: ")); Serial.println(causaReinicio);
  Serial.print(F("Posicion guardada:"));
  for (uint8_t j = 0; j < 5; j++) { Serial.print(' '); Serial.print(pos[j], 1); }
  Serial.println();
  // Se crea una sola vez. No se reintenta: con el firmware viejo del modulo WiFi beginAP puede responder que fallo
  // aunque la red quedo creada, y volver a crearla tumba a la compu que se esta conectando.
  if (WiFi.beginAP(RED, CLAVE, CANAL_WIFI) != WL_AP_LISTENING) Serial.println(F("El modulo WiFi no confirmo la red (con su firmware viejo puede pasar aunque la red funcione)."));
  strncpy(fwWiFi, WiFi.firmwareVersion(), sizeof(fwWiFi) - 1);
  Serial.print(F("Firmware del modulo WiFi: ")); Serial.print(fwWiFi);
  Serial.print(F(", el mas nuevo que conoce esta libreria es ")); Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
  delay(3000);
  server.begin();
  Serial.print(F("Red WiFi: ")); Serial.print(RED); Serial.print(F("  clave: ")); Serial.println(CLAVE);
  Serial.print(F("Sliders: http://")); Serial.println(WiFi.localIP());
  Serial.print(F("Interfaz del proyecto: http://")); Serial.print(WiFi.localIP()); Serial.println(F("/robot"));
  // Sin perro guardian (WDT): el del UNO R4 dura como maximo 5.6 s y la libreria WiFiS3 espera hasta 10 s la respuesta
  // del modulo WiFi, por ejemplo mientras una compu se conecta a la red. El perro reiniciaba el Arduino a media conexion,
  // la red se caia y Windows pedia la clave otra vez.
}

void loop() {
  actualizar();
  atenderWeb();
  serialRespaldo();
  if (reportar && millis() - tRep > 250) { tRep = millis(); Serial.print(F("E ")); Serial.println(estadoJSON()); }
}
