// Nodo CO2 (MH-Z1911A) -> ESP-NOW -> Gateway ESP32 v4
#include <Arduino.h>
#include "paquetes.h"
#include "espnow_nodo.h"

// ---------- RED ----------
uint8_t MAC_GATEWAY[6] = {0x58, 0x2A, 0xBD, 0x72, 0x03, 0xDC}; // MAC STA del ESP32 v4
const char*   SSID_ROUTER   = "**********";   // red del gateway (solo para hallar el canal)
const uint8_t CANAL_DEFECTO = 1;
const uint8_t NODO_ID       = 1;

// ---------- PINES (UART2) ----------
#define RX2_PIN 16   // al TX del sensor
#define TX2_PIN 17   // al RX del sensor

// ---------- UMBRALES ----------
const int  CO2_ALERTA_PPM  = 1000;
const int  CO2_CRITICO_PPM = 1500;
const int  CO2_HISTERESIS  = 100;
const int  CONFIRMACIONES  = 3;
const int  CO2_MIN_VALIDO  = 300;
const int  CO2_MAX_VALIDO  = 10000;
const int  PICO_SUBIDA_PPM = 300;
const int  VENTANA         = 6;
const int  FALLOS_PARA_ALARMA = 5;
const unsigned long PERIODO_MS       = 5000;
const unsigned long CALENTAMIENTO_MS = 180000;

// ---------- ESTADO ----------
byte cmdLeer[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
unsigned long tUltimo = 0;
int nivel = 0;
int candPrev = -1;
int contCand = 0;
int fallosSeguidos = 0;
int hist[VENTANA];
int histIdx = 0, histN = 0;

// Devuelve ppm (o -1 / -2 en error) y la temperatura interna aproximada.
int leerCO2(int8_t &tempC) {
  byte r[9];
  while (Serial2.available()) Serial2.read();
  Serial2.write(cmdLeer, 9);
  if (Serial2.readBytes(r, 9) != 9) return -1;
  if (r[0] != 0xFF || r[1] != 0x86) return -2;
  byte sum = 0;
  for (int i = 1; i < 8; i++) sum += r[i];
  if ((byte)(0xFF - sum + 1) != r[8]) return -2;
  tempC = (int8_t)(r[4] - 40);
  return r[2] * 256 + r[3];
}

int calcularNivel(int ppm, int actual) {
  if (ppm >= CO2_CRITICO_PPM) return 2;
  if (ppm >= CO2_ALERTA_PPM)  return (actual == 2 && ppm >= CO2_CRITICO_PPM - CO2_HISTERESIS) ? 2 : 1;
  if (actual >= 1 && ppm >= CO2_ALERTA_PPM - CO2_HISTERESIS) return 1;
  return 0;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  Serial2.setTimeout(1000);
  delay(500);

  if (!espnowIniciar(MAC_GATEWAY, SSID_ROUTER, CANAL_DEFECTO))
    Serial.println("# ERROR: no se pudo iniciar ESP-NOW");

  Serial.printf("# Nodo CO2 id=%d  MAC=%s  canal=%d\n",
                NODO_ID, WiFi.macAddress().c_str(), g_canal);
  Serial.println("# Primeros 3 min: calentamiento, no se evaluan anomalias.");
  Serial.println("ms,co2_ppm,delta_30s,temp_c,nivel,anomalias,envio_ok");
}

void loop() {
  if (millis() - tUltimo < PERIODO_MS) return;
  tUltimo = millis();

  int8_t  tempC      = TEMP_SIN_DATO;
  int     ppm        = leerCO2(tempC);
  bool    valido     = (ppm >= CO2_MIN_VALIDO && ppm <= CO2_MAX_VALIDO);
  bool    calentando = millis() < CALENTAMIENTO_MS;
  uint8_t an         = 0;
  uint8_t nivelTx    = CO2_NIVEL_SIN_DATO;
  int     delta30    = 0;

  if (valido) fallosSeguidos = 0; else fallosSeguidos++;
  if (fallosSeguidos >= FALLOS_PARA_ALARMA) an |= CO2_BIT_FALLO_SENSOR;

  if (calentando) {
    nivelTx = CO2_NIVEL_CALENTANDO;
  } else if (valido) {
    if (histN == VENTANA) delta30 = ppm - hist[histIdx];
    hist[histIdx] = ppm;
    histIdx = (histIdx + 1) % VENTANA;
    if (histN < VENTANA) histN++;
    if (delta30 >= PICO_SUBIDA_PPM) an |= CO2_BIT_PICO_RAPIDO;

    int cand = calcularNivel(ppm, nivel);
    if (cand == nivel) {
      candPrev = -1; contCand = 0;
    } else {
      if (cand == candPrev) contCand++; else { candPrev = cand; contCand = 1; }
      if (contCand >= CONFIRMACIONES) { nivel = cand; candPrev = -1; contCand = 0; }
    }
    if (nivel == 1)      an |= CO2_BIT_ALERTA_VENT;
    else if (nivel == 2) an |= CO2_BIT_CRITICO;
    nivelTx = (uint8_t)nivel;
  }

  // ---------- Envío ----------
  PaqueteCO2 p = {};
  llenarCabecera(p.h, TIPO_CO2, NODO_ID);
  p.co2_ppm    = (int16_t)ppm;
  p.delta_30s  = (int16_t)delta30;
  p.temp_c     = (ppm > 0) ? tempC : TEMP_SIN_DATO;
  p.calentando = calentando ? 1 : 0;
  p.nivel      = nivelTx;
  p.anomalias  = an;
  espnowEnviar(&p, sizeof(p));

  // Depuración por USB (envio_ok corresponde al envío anterior)
  Serial.printf("%lu,%d,%d,%d,%u,0x%02X,%d\n",
                millis(), ppm, delta30, p.temp_c, nivelTx, an, g_ultimoOk ? 1 : 0);
}
