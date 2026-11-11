// Nodo ambiente (GY-BME280) -> ESP-NOW -> Gateway ESP32 v4
// Temperatura, humedad relativa y presión del salón L418 (techo, centro).
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>     // Librería "Adafruit BME280"
#include "paquetes.h"
#include "espnow_nodo.h"

// ================= RED =================
uint8_t MAC_GATEWAY[6] = {0x58, 0x2A, 0xBD, 0x72, 0x03, 0xDC}; // MAC STA del ESP32 v4
const char*   SSID_ROUTER   = "TU_RED_WIFI";   // red del gateway (solo para hallar el canal)
const uint8_t CANAL_DEFECTO = 1;
const uint8_t NODO_ID       = 7;

// ================= PINES I2C =================
#define PIN_SDA 21
#define PIN_SCL 22

// ================= UMBRALES =================
const float CONFORT_T_MIN = 20.0;   // °C
const float CONFORT_T_MAX = 26.0;   // °C
const float HIST_T        = 0.5;    // la alarma se apaga 0.5 °C dentro del rango
const float HUM_ALTA      = 70.0;   // % HR
const float HIST_HUM      = 5.0;    // la alarma se apaga por debajo de 65 %
const float CAMBIO_T_MAX  = 1.5;    // °C de variación máxima en la ventana
const unsigned long VENTANA_CAMBIO_MS = 5UL * 60UL * 1000UL;   // 5 min
const unsigned long PERIODO_MS        = 10000;                 // una lectura y un envío cada 10 s
const int FALLOS_PARA_ALARMA = 3;

// ================= ESTADO =================
const int N_HIST = VENTANA_CAMBIO_MS / PERIODO_MS;   // 30 lecturas = 5 min
float histT[N_HIST];
int   histIdx = 0, histN = 0;

Adafruit_BME280 bme;
bool    sensorOk = false;
uint8_t direccion = 0;
bool    fueraConfort = false, humAlta = false;
int     fallosSeguidos = 0;
unsigned long tUltEnvio = 0, tUltReintento = 0;

// ================= SENSOR =================
// Lee el registro de identificación para diagnosticar módulos equivocados.
uint8_t leerChipId(uint8_t dir) {
  Wire.beginTransmission(dir);
  Wire.write(0xD0);
  if (Wire.endTransmission() != 0) return 0;
  if (Wire.requestFrom(dir, (uint8_t)1) != 1) return 0;
  return Wire.read();
}

bool iniciarSensor() {
  const uint8_t dirs[2] = {0x76, 0x77};    // SDO a GND = 0x76, SDO a 3.3 V = 0x77
  for (uint8_t d : dirs) {
    if (bme.begin(d, &Wire)) {
      direccion = d;
      // Modo forzado: mide solo cuando se le pide, así el sensor casi no se autocalienta.
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X1,    // temperatura
                      Adafruit_BME280::SAMPLING_X1,    // presión
                      Adafruit_BME280::SAMPLING_X1,    // humedad
                      Adafruit_BME280::FILTER_OFF);
      return true;
    }
    if (leerChipId(d) == 0x58)
      Serial.printf("# AVISO: en 0x%02X hay un BMP280 (NO mide humedad), no un BME280\n", d);
  }
  return false;
}

bool leer(float &t, float &h, float &p) {
  if (!bme.takeForcedMeasurement()) return false;
  t = bme.readTemperature();
  h = bme.readHumidity();
  p = bme.readPressure() / 100.0F;
  if (isnan(t) || isnan(h) || isnan(p)) return false;
  if (t < -40 || t > 85 || h < 0 || h > 100 || p < 300 || p > 1100) return false;   // fuera de rango físico
  return true;
}

// ================= ANOMALÍAS =================
uint8_t evaluar(float t, float h) {
  uint8_t an = 0;

  // Confort térmico con histéresis (evita que la alarma parpadee en el límite)
  if (!fueraConfort) fueraConfort = (t < CONFORT_T_MIN) || (t > CONFORT_T_MAX);
  else               fueraConfort = (t < CONFORT_T_MIN + HIST_T) || (t > CONFORT_T_MAX - HIST_T);
  if (fueraConfort) an |= AMB_BIT_TEMP_FUERA_CONFORT;

  if (!humAlta) humAlta = h > HUM_ALTA;
  else          humAlta = h > HUM_ALTA - HIST_HUM;
  if (humAlta) an |= AMB_BIT_HUMEDAD_ALTA;

  // Cambio rápido: comparación con la lectura de hace 5 minutos
  if (histN == N_HIST && fabsf(t - histT[histIdx]) > CAMBIO_T_MAX) an |= AMB_BIT_CAMBIO_RAPIDO_TEMP;
  histT[histIdx] = t;
  histIdx = (histIdx + 1) % N_HIST;
  if (histN < N_HIST) histN++;

  return an;
}

// ================= ENVÍO =================
void enviar(float t, float h, float p, uint8_t an) {
  PaqueteAmbiente pk = {};
  llenarCabecera(pk.h, TIPO_AMBIENTE, NODO_ID);
  pk.temp_c    = t;
  pk.hum_pct   = h;
  pk.pres_hpa  = p;
  pk.anomalias = an;
  espnowEnviar(&pk, sizeof(pk));

  Serial.printf("%lu,%.2f,%.1f,%.2f,0x%02X,%d\n", millis(), t, h, p, an, g_ultimoOk ? 1 : 0);
}

// ================= SETUP / LOOP =================
void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(PIN_SDA, PIN_SCL);

  if (!espnowIniciar(MAC_GATEWAY, SSID_ROUTER, CANAL_DEFECTO))
    Serial.println("# ERROR: no se pudo iniciar ESP-NOW");
  Serial.printf("# Nodo ambiente id=%d  MAC=%s  canal=%d\n",
                NODO_ID, WiFi.macAddress().c_str(), g_canal);

  sensorOk = iniciarSensor();
  if (sensorOk) Serial.printf("# BME280 encontrado en 0x%02X\n", direccion);
  else          Serial.println("# ERROR: no se detecta el BME280 (revisar SDA=21, SCL=22, 3.3V)");
  Serial.println("ms,temp_c,hum_pct,pres_hpa,anomalias,envio_ok");

  tUltEnvio = millis() - PERIODO_MS;   // primera lectura inmediata
}

void loop() {
  unsigned long ahora = millis();
  if (ahora - tUltEnvio < PERIODO_MS) return;
  tUltEnvio = ahora;

  if (!sensorOk && ahora - tUltReintento > 10000) {
    tUltReintento = ahora;
    sensorOk = iniciarSensor();
  }

  float t, h, p;
  if (sensorOk && leer(t, h, p)) {
    fallosSeguidos = 0;
    enviar(t, h, p, evaluar(t, h));
  } else {
    fallosSeguidos++;
    uint8_t an = (fallosSeguidos >= FALLOS_PARA_ALARMA || !sensorOk) ? AMB_BIT_FALLO : 0;
    enviar(NAN, NAN, NAN, an);
    if (fallosSeguidos >= FALLOS_PARA_ALARMA) sensorOk = false;   // reintentar iniciarlo
  }
}
