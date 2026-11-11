// Nodo magnético (KY-024 Hall) -> ESP-NOW -> Gateway ESP32 v4
// Estado de la puerta del salón L418: sensor en el marco, imán en la hoja de la puerta.
// La salida del KY-024 es INVERTIDA: imán cerca (puerta cerrada) = campo alto = lectura BAJA.
#include <Arduino.h>
#include "paquetes.h"
#include "espnow_nodo.h"

// ================= RED =================
uint8_t MAC_GATEWAY[6] = {0x58, 0x2A, 0xBD, 0x72, 0x03, 0xDC}; // MAC STA del ESP32 v4
const char*   SSID_ROUTER   = "TU_RED_WIFI";   // red del gateway (solo para hallar el canal)
const uint8_t CANAL_DEFECTO = 1;
const uint8_t NODO_ID       = 6;

// ================= PINES =================
// Salida analógica (AO) del KY-024. Debe ser un pin de ADC1 (GPIO 32 a 39):
// los pines de ADC2 no funcionan mientras la radio (ESP-NOW) está activa.
#define PIN_AO 34

// ================= UMBRALES (lectura de 12 bits, KY-024 alimentado a 3.3 V) =================
const int UMBRAL_CERRADA = 1200;   // por debajo: imán cerca -> puerta cerrada
const int UMBRAL_ABIERTA = 1800;   // por encima: imán lejos -> puerta abierta
                                   // entre ambos: se mantiene el estado anterior (histéresis)
const unsigned long ANTIRREBOTE_MS = 150;  // el nuevo estado debe sostenerse este tiempo
const unsigned long MUESTREO_MS    = 50;   // 20 lecturas por segundo
const int MUESTRAS_PROMEDIO        = 8;    // cada lectura promedia 8 conversiones del ADC

// ================= ENVÍO Y ANOMALÍAS =================
const unsigned long PERIODO_MS            = 10000;              // mensaje periódico
const unsigned long MIN_ENTRE_ENVIOS      = 200;                // envío rápido tras un cambio
const unsigned long ABIERTA_PROLONGADA_MS = 10UL * 60UL * 1000UL; // 10 min abierta
// Fallo: lectura pegada al máximo (cortocircuito o pull-up sin sensor)
// o ruido excesivo (pin flotando por un cable suelto) durante varios segundos seguidos.
const int RAW_EXTREMO_ALTO = 4075;
const int RUIDO_MAX        = 1000;   // variación máx-mín aceptable dentro de 1 s
const int SEG_PARA_FALLO   = 3;

// 1: imprime la lectura cada 100 ms (para comprobar o ajustar los umbrales)
#define DEBUG_RAW 0

// ================= ESTADO =================
bool abierta = false;
bool estadoIniciado = false;
bool hayCandidato = false;
bool candidatoAbierta = false;
unsigned long tCandidato = 0, tAbierta = 0;

uint16_t aperturasVent = 0;
unsigned long msAbiertaVent = 0;
int  rawActual = 0;
bool hayEvento = false;

int  rawMinSeg = 4095, rawMaxSeg = 0, segAnomalos = 0;
bool fallo = false;

unsigned long tMuestra = 0, tUltEnvio = 0, tSegundo = 0;

// ================= LECTURA =================
int leerRaw() {
  long s = 0;
  for (int i = 0; i < MUESTRAS_PROMEDIO; i++) s += analogRead(PIN_AO);
  return s / MUESTRAS_PROMEDIO;
}

// Histéresis + antirrebote: solo cambia si la lectura cruza el umbral contrario
// y se mantiene del otro lado durante ANTIRREBOTE_MS.
void actualizarEstado(int raw, unsigned long ahora) {
  if (!estadoIniciado) {   // primera lectura: se decide por el punto medio
    abierta = raw > (UMBRAL_CERRADA + UMBRAL_ABIERTA) / 2;
    tAbierta = ahora;
    estadoIniciado = true;
    return;
  }
  int objetivo = -1;
  if (raw < UMBRAL_CERRADA)      objetivo = 0;
  else if (raw > UMBRAL_ABIERTA) objetivo = 1;

  if (objetivo < 0 || objetivo == (int)abierta) { hayCandidato = false; return; }
  if (!hayCandidato || candidatoAbierta != (bool)objetivo) {
    hayCandidato = true;
    candidatoAbierta = objetivo;
    tCandidato = ahora;
    return;
  }
  if (ahora - tCandidato < ANTIRREBOTE_MS) return;

  abierta = candidatoAbierta;      // cambio confirmado
  hayCandidato = false;
  hayEvento = true;
  if (abierta) {
    aperturasVent++;
    tAbierta = ahora;
  }
}

// Revisa una vez por segundo si la señal es físicamente creíble.
void revisarFallo(int raw, unsigned long ahora) {
  rawMinSeg = min(rawMinSeg, raw);
  rawMaxSeg = max(rawMaxSeg, raw);
  if (ahora - tSegundo < 1000) return;
  tSegundo = ahora;
  bool anomalo = (rawMaxSeg - rawMinSeg > RUIDO_MAX) || (rawMinSeg >= RAW_EXTREMO_ALTO);
  segAnomalos = anomalo ? segAnomalos + 1 : 0;
  bool antes = fallo;
  fallo = segAnomalos >= SEG_PARA_FALLO;
  if (fallo != antes) hayEvento = true;
  rawMinSeg = 4095;
  rawMaxSeg = 0;
}

// ================= ENVÍO =================
void enviar(unsigned long ahora) {
  uint8_t an = 0;
  if (fallo) an |= MAG_BIT_FALLO;
  if (abierta && ahora - tAbierta > ABIERTA_PROLONGADA_MS) an |= MAG_BIT_ABIERTA_PROLONGADA;

  PaqueteMagnetico p = {};
  llenarCabecera(p.h, TIPO_MAGNETICO, NODO_ID);
  p.puerta_abierta = abierta ? 1 : 0;
  p.aperturas      = aperturasVent;
  p.seg_abierta    = msAbiertaVent / 1000.0f;
  p.campo_raw      = (uint16_t)rawActual;
  p.anomalias      = an;
  espnowEnviar(&p, sizeof(p));

  Serial.printf("%lu,%u,%u,%.1f,%d,0x%02X,%d\n",
                ahora, p.puerta_abierta, aperturasVent, p.seg_abierta, rawActual, an,
                g_ultimoOk ? 1 : 0);

  aperturasVent = 0;
  msAbiertaVent = 0;
  hayEvento = false;
  tUltEnvio = ahora;
}

// ================= SETUP / LOOP =================
void setup() {
  Serial.begin(115200);
  delay(500);
  analogReadResolution(12);              // 0-4095
  analogSetAttenuation(ADC_11db);        // rango de entrada ~0-3.1 V

  if (!espnowIniciar(MAC_GATEWAY, SSID_ROUTER, CANAL_DEFECTO))
    Serial.println("# ERROR: no se pudo iniciar ESP-NOW");
  Serial.printf("# Nodo magnetico (puerta) id=%d  MAC=%s  canal=%d\n",
                NODO_ID, WiFi.macAddress().c_str(), g_canal);
  Serial.printf("# Umbrales: cerrada < %d, abierta > %d\n", UMBRAL_CERRADA, UMBRAL_ABIERTA);
  Serial.println("ms,puerta_abierta,aperturas,seg_abierta,campo_raw,anomalias,envio_ok");

  tMuestra = tUltEnvio = tSegundo = millis();
}

void loop() {
  unsigned long ahora = millis();
  if (ahora - tMuestra < MUESTREO_MS) return;
  unsigned long dt = ahora - tMuestra;
  tMuestra = ahora;

  rawActual = leerRaw();
  revisarFallo(rawActual, ahora);
  if (estadoIniciado && abierta) msAbiertaVent += dt;   // tiempo abierta antes de esta lectura
  if (!fallo) actualizarEstado(rawActual, ahora);       // con fallo se congela el último estado

#if DEBUG_RAW
  static unsigned long tDbg = 0;
  if (ahora - tDbg >= 100) { tDbg = ahora; Serial.printf("# raw=%d estado=%s\n", rawActual, abierta ? "ABIERTA" : "CERRADA"); }
#endif

  bool porPeriodo = ahora - tUltEnvio >= PERIODO_MS;
  bool porEvento  = hayEvento && ahora - tUltEnvio >= MIN_ENTRE_ENVIOS;
  if (porPeriodo || porEvento) enviar(ahora);
}
