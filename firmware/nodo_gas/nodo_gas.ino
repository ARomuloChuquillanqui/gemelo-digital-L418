// Nodo gas (MQ-2) -> ESP-NOW -> Gateway ESP32 v4
// Índice relativo de gases combustibles y humo en el salón L418 (techo).
// IMPORTANTE: el MQ-2 trabaja a 5 V. Sus salidas AO y DO pasan por un DIVISOR antes del ESP32.
// Primer uso: dejar el sensor encendido 24 h (precalentamiento del datasheet) y luego
// escribir CAL en el Monitor Serie, con el salón ventilado, para medir R0 en aire limpio.
#include <Arduino.h>
#include <Preferences.h>         // memoria flash del ESP32 (viene con el núcleo)
#include "paquetes.h"
#include "espnow_nodo.h"

// ================= RED =================
uint8_t MAC_GATEWAY[6] = {0x58, 0x2A, 0xBD, 0x72, 0x03, 0xDC}; // MAC STA del ESP32 v4
const char*   SSID_ROUTER   = "********";   // red del gateway (solo para hallar el canal)
const uint8_t CANAL_DEFECTO = 1;
const uint8_t NODO_ID       = 3;

// ================= PINES (ADC1: los de ADC2 no funcionan con la radio activa) =================
#define PIN_AO 35   // AO del MQ-2 a través del divisor
#define PIN_DO 32   // DO del MQ-2 a través del divisor (se lee como voltaje)

// ================= CIRCUITO =================
const float V_C     = 5.00;   // V; voltaje real que alimenta al módulo (medir con multímetro)
const float R_L     = 20.0;   // kΩ; resistencia de carga del módulo (SMD "203" = 20k, "102" = 1k)
const float DIV_R1  = 10.0;   // kΩ; entre la salida del módulo y el pin del ESP32
const float DIV_R2  = 15.0;   // kΩ; entre el pin del ESP32 y GND (5 V -> 3.0 V)
const float FACTOR_DIV = (DIV_R1 + DIV_R2) / DIV_R2;
const float R0_FIJO = 0.0;    // kΩ; si se pone un valor > 0, se usa en lugar del calibrado

// ================= TIEMPOS =================
const unsigned long CALENTAMIENTO_MS = 180000;  // 3 min tras cada encendido
const unsigned long CALIB_R0_MS      = 60000;   // 1 min promediando Rs en aire limpio
const unsigned long PERIODO_MS       = 2000;    // una lectura y un envío cada 2 s
const int MUESTRAS_PROMEDIO = 16;

// ================= UMBRALES =================
const float UMBRAL_GAS     = 0.60;  // rs_r0 por debajo -> posible gas
const float UMBRAL_LIMPIO  = 0.70;  // rs_r0 por encima -> se apaga la alarma (histéresis)
const float UMBRAL_CRITICO        = 0.17;  // rs_r0 por debajo -> CRITICO (~200 ppm GLP, inicio del rango del datasheet)
const float UMBRAL_CRITICO_SALIDA = 0.20;  // rs_r0 por encima -> sale de CRITICO (histéresis)

// Curva del datasheet para GLP: ppm = A * (Rs/R0_ds)^B, con R0_ds = Rs_aire_limpio / 9.83
const float GLP_A = 613.9;
const float GLP_B = -2.074;
const float RATIO_AIRE_LIMPIO = 9.83;
const float PPM_MAX = 10000.0;             // límite superior del rango del datasheet
const int   CONFIRMACIONES = 3;     // lecturas seguidas para cambiar de estado (~6 s)
const float V_MIN_VALIDO   = 0.02;  // V; por debajo: sensor desconectado o sin alimentación
const float V_DO_ALARMA    = 1.0;   // V (escala del módulo); DO por debajo = LOW = gas
const int   FALLOS_PARA_ALARMA = 3;

// ================= ESTADO =================
Preferences prefs;
float r0 = 0;                       // kΩ
bool  calibrandoR0 = false;
double sumaR0 = 0;
int   nR0 = 0;
unsigned long tInicioCalib = 0;

uint8_t nivelGas = GAS_NIVEL_NORMAL;
uint8_t candidato = GAS_NIVEL_NORMAL;
int     contCambio = 0;
int  fallosSeguidos = 0;
unsigned long tUltEnvio = 0;

// ================= LECTURAS =================
float leerVoltios(int pin) {        // voltaje en la escala del módulo (antes del divisor)
  uint32_t mv = 0;
  for (int i = 0; i < MUESTRAS_PROMEDIO; i++) mv += analogReadMilliVolts(pin);
  return (mv / (float)MUESTRAS_PROMEDIO) / 1000.0f * FACTOR_DIV;
}

float calcularRs(float vout) {      // kΩ
  return R_L * (V_C - vout) / vout;
}

// ================= R0 =================
void iniciarCalibracion() {
  calibrandoR0 = true;
  sumaR0 = 0;
  nR0 = 0;
  tInicioCalib = millis();
  Serial.println("# Calibrando R0 durante 1 min: el aire debe estar limpio (salon ventilado).");
}

void acumularCalibracion(float rs) {
  sumaR0 += rs;
  nR0++;
  if (millis() - tInicioCalib < CALIB_R0_MS) return;
  r0 = sumaR0 / nR0;
  prefs.putFloat("r0", r0);
  calibrandoR0 = false;
  Serial.printf("# R0 calibrado y guardado: %.2f kOhm\n", r0);
}

// Comandos por Monitor Serie: CAL (recalibrar R0) y R0 (ver valor actual)
void revisarComandos() {
  static char buf[8];
  static int n = 0;
  while (Serial.available()) {
    char c = toupper(Serial.read());
    if (c == '\n' || c == '\r') {
      buf[n] = 0;
      if (!strcmp(buf, "CAL")) {
        if (millis() < CALENTAMIENTO_MS) Serial.println("# Espera a que termine el calentamiento.");
        else iniciarCalibracion();
      } else if (!strcmp(buf, "R0")) {
        Serial.printf("# R0 actual: %.2f kOhm\n", r0);
      }
      n = 0;
    } else if (n < 7) {
      buf[n++] = c;
    }
  }
}

// ================= GAS =================
// Nivel al que correspondería la lectura, con histéresis según el nivel actual
uint8_t nivelObjetivo(float rsr0) {
  if (nivelGas == GAS_NIVEL_NORMAL)
    return rsr0 < UMBRAL_CRITICO ? GAS_NIVEL_CRITICO : rsr0 < UMBRAL_GAS ? GAS_NIVEL_ALERTA : GAS_NIVEL_NORMAL;
  if (nivelGas == GAS_NIVEL_ALERTA)
    return rsr0 < UMBRAL_CRITICO ? GAS_NIVEL_CRITICO : rsr0 > UMBRAL_LIMPIO ? GAS_NIVEL_NORMAL : GAS_NIVEL_ALERTA;
  return rsr0 > UMBRAL_LIMPIO ? GAS_NIVEL_NORMAL : rsr0 > UMBRAL_CRITICO_SALIDA ? GAS_NIVEL_ALERTA : GAS_NIVEL_CRITICO;
}
// El cambio de nivel se confirma con CONFIRMACIONES lecturas seguidas
void evaluarGas(float rsr0) {
  uint8_t obj = nivelObjetivo(rsr0);
  if (obj == nivelGas) { contCambio = 0; return; }
  if (obj == candidato) contCambio++;
  else { candidato = obj; contCambio = 1; }
  if (contCambio >= CONFIRMACIONES) { nivelGas = obj; contCambio = 0; }
}

// Estimación referencial de GLP con la curva del datasheet (válida solo para GLP)
float estimarPpmGLP(float rsr0) {
  if (isnan(rsr0) || rsr0 <= 0) return NAN;
  return fminf(GLP_A * powf(RATIO_AIRE_LIMPIO * rsr0, GLP_B), PPM_MAX);
}
// ================= SETUP / LOOP =================
void setup() {
  Serial.begin(115200);
  delay(500);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);     // entrada de ~0 a 3.1 V

  prefs.begin("mq2", false);
  r0 = (R0_FIJO > 0) ? R0_FIJO : prefs.getFloat("r0", 0);

  if (!espnowIniciar(MAC_GATEWAY, SSID_ROUTER, CANAL_DEFECTO))
    Serial.println("# ERROR: no se pudo iniciar ESP-NOW");
  Serial.printf("# Nodo gas (MQ-2) id=%d  MAC=%s  canal=%d\n",
                NODO_ID, WiFi.macAddress().c_str(), g_canal);
  if (r0 > 0) Serial.printf("# R0 = %.2f kOhm (guardado). Escribe CAL para recalibrar.\n", r0);
  else        Serial.println("# Sin R0: se calibrara automaticamente al terminar el calentamiento.");
  Serial.println("# Calentando 3 min: no se evaluan anomalias.");
  Serial.println("ms,v_ao,rs_kohm,rs_r0,ppm_glp_est,nivel,do_alarma,calentando,anomalias,envio_ok");
}

void loop() {
  revisarComandos();
  unsigned long ahora = millis();
  if (ahora - tUltEnvio < PERIODO_MS) return;
  tUltEnvio = ahora;

  float vout = leerVoltios(PIN_AO);
  bool  doAlarma = leerVoltios(PIN_DO) < V_DO_ALARMA;
  bool  calentando = ahora < CALENTAMIENTO_MS;
  bool  valido = vout >= V_MIN_VALIDO && vout < V_C;
  uint8_t an = 0;
  float rs = NAN, rsr0 = NAN, ppm = NAN;
  if (valido) fallosSeguidos = 0;
  else if (++fallosSeguidos >= FALLOS_PARA_ALARMA) an |= GAS_BIT_FALLO;

  if (valido && !calentando) {
    rs = calcularRs(vout);
    if (r0 <= 0 && !calibrandoR0) iniciarCalibracion();
    if (calibrandoR0) acumularCalibracion(rs);
    if (r0 > 0 && !calibrandoR0) {
            rsr0 = rs / r0;
      ppm  = estimarPpmGLP(rsr0);
      evaluarGas(rsr0);
      if (nivelGas >= GAS_NIVEL_ALERTA)  an |= GAS_BIT_GAS_DETECTADO;
      if (nivelGas == GAS_NIVEL_CRITICO) an |= GAS_BIT_CRITICO;
    }
  } else {
    contCambio = 0;
  }

  bool listo = !calentando && !calibrandoR0 && r0 > 0;
  uint8_t nivelTx = !valido ? GAS_NIVEL_SIN_DATO : !listo ? GAS_NIVEL_CALENTANDO : nivelGas;

  PaqueteGas p = {};
  llenarCabecera(p.h, TIPO_GAS, NODO_ID);
  p.v_ao       = vout;
  p.rs_r0      = rsr0;
  p.ppm_glp_est = ppm;
  p.nivel      = nivelTx;
  p.do_alarma  = doAlarma ? 1 : 0;
  p.calentando = (calentando || calibrandoR0 || r0 <= 0) ? 1 : 0;
  p.anomalias  = an;
  espnowEnviar(&p, sizeof(p));
    Serial.printf("%lu,%.3f,%.2f,%.3f,%.0f,%u,%u,%u,0x%02X,%d\n", ahora, vout, rs, rsr0, ppm,
                nivelTx, p.do_alarma, p.calentando, an, g_ultimoOk ? 1 : 0);
}
