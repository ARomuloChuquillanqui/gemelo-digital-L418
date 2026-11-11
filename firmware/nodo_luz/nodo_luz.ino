// Nodo luz (GY-302 BH1750) -> ESP-NOW -> Gateway ESP32 v4
// Iluminancia del salón L418 (techo, centro) y estimación sobre la mesa.
#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>              // Librería "BH1750" de Christopher Laws
#include "paquetes.h"
#include "espnow_nodo.h"

// ================= RED =================
uint8_t MAC_GATEWAY[6] = {0x58, 0x2A, 0xBD, 0x72, 0x03, 0xDC}; // MAC STA del ESP32 v4
const char*   SSID_ROUTER   = "*******";   // red del gateway (solo para hallar el canal)
const uint8_t CANAL_DEFECTO = 1;
const uint8_t NODO_ID       = 4;

// ================= PINES I2C =================
#define PIN_SDA 21
#define PIN_SCL 22

// ================= CALIBRACIÓN Y UMBRALES =================
const float FACTOR_MESA   = 0.482;  // k = E_mesa / E_techo (medido con luxómetro en la mesa)
const float ILUM_BAJA     = 300.0;  // lux sobre la mesa: por debajo se marca ILUMINACION_BAJA
const float HIST_ILUM     = 30.0;   // la alarma se apaga al superar 330 lx
// Cambio brusco de luces: compara el promedio de 3 s con el de los 3 s anteriores
const float CAMBIO_ABS    = 50.0;   // lux mínimos de diferencia
const float CAMBIO_REL    = 0.30;   // y al menos 30 % del nivel mayor
const float ESTABLE_REL   = 0.20;   // cada bloque de 3 s debe ser estable (variación < 20 %)
const unsigned long ESPERA_CAMBIO_MS = 6000;   // no detectar dos veces el mismo cambio

const unsigned long MUESTREO_MS = 1000;   // una lectura por segundo
const unsigned long PERIODO_MS  = 10000;  // un envío cada 10 s (y de inmediato si cambian las luces)
const int FALLOS_PARA_ALARMA = 3;

// ================= ESTADO =================
BH1750 sensor;
bool    sensorOk = false;
uint8_t direccion = 0;

float   ult[6];          // últimas 6 lecturas (para detectar cambios bruscos)
int     nUlt = 0;
unsigned long tUltCambio = 0;

double  sumaVent = 0;
int     nVent = 0;
float   minVent = 1e9, maxVent = -1;
int8_t  cambioVent = 0;
bool    hayEvento = false;
bool    iluminacionBaja = false;
int     fallosSeguidos = 0;
bool    fallo = false;
unsigned long tMuestra = 0, tUltEnvio = 0, tUltReintento = 0;

// ================= SENSOR =================
bool iniciarSensor() {
  const uint8_t dirs[2] = {0x23, 0x5C};   // ADDR a GND = 0x23, ADDR a 3.3 V = 0x5C
  for (uint8_t d : dirs) {
    if (sensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, d, &Wire)) {
      direccion = d;
      return true;
    }
  }
  return false;
}

// ================= CAMBIO BRUSCO =================
// Las luces cambian en menos de 1 s y luego se mantienen; las nubes cambian de forma gradual.
int8_t detectarCambio(float lux, unsigned long ahora) {
  if (nUlt < 6) ult[nUlt++] = lux;
  else { for (int i = 0; i < 5; i++) ult[i] = ult[i + 1]; ult[5] = lux; }
  if (nUlt < 6 || ahora - tUltCambio < ESPERA_CAMBIO_MS) return 0;

  float a = 0, b = 0, aMin = 1e9, aMax = 0, bMin = 1e9, bMax = 0;
  for (int i = 0; i < 3; i++) {
    a += ult[i];     aMin = fminf(aMin, ult[i]);     aMax = fmaxf(aMax, ult[i]);
    b += ult[i + 3]; bMin = fminf(bMin, ult[i + 3]); bMax = fmaxf(bMax, ult[i + 3]);
  }
  a /= 3; b /= 3;
  bool estables = (aMax - aMin <= ESTABLE_REL * fmaxf(a, 1)) && (bMax - bMin <= ESTABLE_REL * fmaxf(b, 1));
  float dif = b - a;
  if (estables && fabsf(dif) > CAMBIO_ABS && fabsf(dif) > CAMBIO_REL * fmaxf(a, b)) {
    tUltCambio = ahora;
    return dif > 0 ? +1 : -1;
  }
  return 0;
}

// ================= ENVÍO =================
void enviar(unsigned long ahora) {
  PaqueteLuz p = {};
  llenarCabecera(p.h, TIPO_LUZ, NODO_ID);
  uint8_t an = fallo ? LUZ_BIT_FALLO : 0;

  if (nVent > 0) {
    float lux = sumaVent / nVent;
    float mesa = lux * FACTOR_MESA;
    if (!iluminacionBaja) iluminacionBaja = mesa < ILUM_BAJA;
    else                  iluminacionBaja = mesa < ILUM_BAJA + HIST_ILUM;
    if (iluminacionBaja) an |= LUZ_BIT_ILUMINACION_BAJA;
    p.lux = lux;
    p.lux_min = minVent;
    p.lux_max = maxVent;
    p.lux_mesa_est = mesa;
  } else {
    p.lux = p.lux_min = p.lux_max = p.lux_mesa_est = NAN;   // sin lecturas válidas
  }
  p.luces_cambio = cambioVent;
  p.anomalias = an;
  espnowEnviar(&p, sizeof(p));

  Serial.printf("%lu,%.1f,%.1f,%.1f,%.1f,%d,0x%02X,%d\n", ahora, p.lux, p.lux_min, p.lux_max,
                p.lux_mesa_est, p.luces_cambio, an, g_ultimoOk ? 1 : 0);

  sumaVent = 0; nVent = 0; minVent = 1e9; maxVent = -1;
  cambioVent = 0;
  hayEvento = false;
  tUltEnvio = ahora;
}

// ================= SETUP / LOOP =================
void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(PIN_SDA, PIN_SCL);

  if (!espnowIniciar(MAC_GATEWAY, SSID_ROUTER, CANAL_DEFECTO))
    Serial.println("# ERROR: no se pudo iniciar ESP-NOW");
  Serial.printf("# Nodo luz id=%d  MAC=%s  canal=%d\n", NODO_ID, WiFi.macAddress().c_str(), g_canal);

  sensorOk = iniciarSensor();
  if (sensorOk) Serial.printf("# BH1750 encontrado en 0x%02X\n", direccion);
  else          Serial.println("# ERROR: no se detecta el BH1750 (revisar SDA=21, SCL=22, VCC)");
  Serial.println("ms,lux,lux_min,lux_max,lux_mesa_est,luces_cambio,anomalias,envio_ok");

  tMuestra = tUltEnvio = millis();
}

void loop() {
  unsigned long ahora = millis();

  if (ahora - tMuestra >= MUESTREO_MS) {
    tMuestra = ahora;
    if (!sensorOk && ahora - tUltReintento > 10000) {
      tUltReintento = ahora;
      sensorOk = iniciarSensor();
    }
    float lux = sensorOk ? sensor.readLightLevel() : -1;   // negativo = error de lectura
    if (lux >= 0) {
      fallosSeguidos = 0;
      fallo = false;
      sumaVent += lux; nVent++;
      minVent = fminf(minVent, lux);
      maxVent = fmaxf(maxVent, lux);
      int8_t c = detectarCambio(lux, ahora);
      if (c != 0) { cambioVent = c; hayEvento = true; }
    } else if (++fallosSeguidos >= FALLOS_PARA_ALARMA) {
      if (!fallo) hayEvento = true;
      fallo = true;
      sensorOk = false;     // forzar reinicio del sensor
      nUlt = 0;
    }
  }

  if (ahora - tUltEnvio >= PERIODO_MS || (hayEvento && ahora - tUltEnvio >= 200)) enviar(ahora);
}
