// Nodo audio (INMP441) -> ESP-NOW -> Gateway ESP32 v4
// Mantener silencio durante los primeros 5 segundos.
#include <Arduino.h>
#include "driver/i2s.h"
#include "paquetes.h"
#include "espnow_nodo.h"

// ---------- RED ----------
uint8_t MAC_GATEWAY[6] = {0x58, 0x2A, 0xBD, 0x72, 0x03, 0xDC}; // MAC STA del ESP32 v4
const char*   SSID_ROUTER   = "******";
const uint8_t CANAL_DEFECTO = 1;
const uint8_t NODO_ID       = 2;

// ---------- CALIBRACIÓN ----------
// INMP441: -26 dBFS @ 94 dB SPL  ->  dB SPL = dBFS + 120
// Ajustar comparando con un sonómetro.
const float OFFSET_SPL_DB = 120.0f;

// Pines I2S
#define PIN_SCK 26
#define PIN_WS  25
#define PIN_SD  33
#define I2S_PORT I2S_NUM_0

// Parámetros de audio
const int FS = 16000;
const int N = 512;
const float SALTO_DB = 20.0;
const float PISO_FUERTE_DB = -45.0;
const float CREST_IMPACTO_DB = 15.0;
const int MAX_BLOQ_IMPACTO = 8;
const int BLOQ_SOSTENIDO = 16;
const float TONAL_MIN = 0.35;
const float PISO_TONAL_DB = -55.0;
const int TONAL_VENTANAS = 30;
const int BLOQUES_ARRANQUE = 150;

// Variables de análisis
int32_t bufI2S[N];
float bloque[N], bloqueMax[N], re[N], im[N], pw[N / 2];
float baseDb = -60;
bool baseInit = false;
long bloquesTotal = 0;

unsigned long tVentana = 0, tUltValido = 0, tFalloPrint = 0;
int bloquesVent = 0, fuertes = 0, sostenidoVent = 0, tonalCont = 0;
float potAcum = 0, picoVent = -200, maxRms = -200;
float maxCrest = 0, freqPrev = 0;

// FFT radix-2 in-place
void fft(float *r, float *i_, int n) {
  for (int i = 1, j = 0; i < n; i++) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float t = r[i]; r[i] = r[j]; r[j] = t;
      t = i_[i]; i_[i] = i_[j]; i_[j] = t;
    }
  }
  for (int len = 2; len <= n; len <<= 1) {
    float ang = -2.0f * PI / len;
    float wr = cosf(ang), wi = sinf(ang);
    for (int i = 0; i < n; i += len) {
      float cr = 1, ci = 0;
      for (int k = 0; k < len / 2; k++) {
        int a = i + k, b = i + k + len / 2;
        float tr = r[b] * cr - i_[b] * ci;
        float ti = r[b] * ci + i_[b] * cr;
        r[b] = r[a] - tr;  i_[b] = i_[a] - ti;
        r[a] += tr;        i_[a] += ti;
        float ncr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = ncr;
      }
    }
  }
}

// Si no llegan muestras por 3 s, envía un paquete de fallo cada segundo.
void avisarFallo() {
  if (millis() - tUltValido > 3000 && millis() - tFalloPrint >= 1000) {
    tFalloPrint = millis();

    PaqueteAudio p = {};
    llenarCabecera(p.h, TIPO_AUDIO, NODO_ID);
    p.leq_db = p.lmax_db = p.pico_db = p.base_db = NAN;
    p.freq_dom_hz = p.tonal = NAN;
    p.anomalias = AUD_BIT_FALLO;
    espnowEnviar(&p, sizeof(p));

    Serial.printf("%lu,nan,nan,nan,nan,nan,nan,0,0,0x%02X,%d\n",
                  millis(), AUD_BIT_FALLO, g_ultimoOk ? 1 : 0);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!espnowIniciar(MAC_GATEWAY, SSID_ROUTER, CANAL_DEFECTO))
    Serial.println("# ERROR: no se pudo iniciar ESP-NOW");

  Serial.printf("# Nodo audio id=%d  MAC=%s  canal=%d\n",
                NODO_ID, WiFi.macAddress().c_str(), g_canal);

  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = FS;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = PIN_SCK;
  pins.ws_io_num = PIN_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = PIN_SD;

  if (i2s_driver_install(I2S_PORT, &cfg, 0, NULL) != ESP_OK ||
      i2s_set_pin(I2S_PORT, &pins) != ESP_OK) {
    Serial.println("# ERROR: no se pudo iniciar I2S.");
    while (1) delay(1000);
  }

  tVentana = tUltValido = millis();
  Serial.println("# Calibrando ruido de fondo: mantener silencio 5 s.");
  Serial.println("ms,leq_db,lmax_db,pico_db,base_db,freq_hz,tonal,"
                 "bloques_fuertes,calibrando,anomalias,envio_ok");
}

void loop() {
  size_t bytes = 0;
  i2s_read(I2S_PORT, bufI2S, sizeof(bufI2S), &bytes, pdMS_TO_TICKS(200));

  if ((int)(bytes / 4) < N) { avisarFallo(); return; }
  tUltValido = millis();

  // RMS y pico del bloque (dBFS)
  double s = 0;
  for (int i = 0; i < N; i++) {
    bloque[i] = (bufI2S[i] >> 8) / 8388608.0f;
    s += bloque[i];
  }
  float media = s / N, pot = 0, pico = 0;
  for (int i = 0; i < N; i++) {
    bloque[i] -= media;
    pot += bloque[i] * bloque[i];
    if (fabsf(bloque[i]) > pico) pico = fabsf(bloque[i]);
  }
  float rms    = sqrtf(pot / N);
  float rmsDb  = rms  > 1e-7f ? 20.0f * log10f(rms)  : -120.0f;
  float picoDb = pico > 1e-7f ? 20.0f * log10f(pico) : -120.0f;
  float crest  = picoDb - rmsDb;

  // Ruido de fondo y sonidos fuertes
  if (!baseInit) { baseDb = rmsDb; baseInit = true; }
  bool fuerte = (rmsDb > baseDb + SALTO_DB) && (rmsDb > PISO_FUERTE_DB);
  if (!fuerte) baseDb += 0.01f * (rmsDb - baseDb);

  bloquesTotal++;
  bloquesVent++;
  potAcum += rms * rms;
  if (picoDb > picoVent) picoVent = picoDb;
  if (fuerte) { fuertes++; if (crest > maxCrest) maxCrest = crest; }
  if (rmsDb > maxRms) { maxRms = rmsDb; memcpy(bloqueMax, bloque, sizeof(bloque)); }

  // Reporte cada segundo
  if (millis() - tVentana < 1000) return;
  tVentana = millis();

  float rmsVentDb = 10.0f * log10f(potAcum / bloquesVent + 1e-14f);
  bool listo = bloquesTotal > BLOQUES_ARRANQUE;
  uint8_t an = 0;

  if (listo) {
    if (fuertes >= 1 && fuertes <= MAX_BLOQ_IMPACTO && maxCrest >= CREST_IMPACTO_DB)
      an |= AUD_BIT_IMPACTO;
    if (fuertes >= BLOQ_SOSTENIDO) { an |= AUD_BIT_SOSTENIDO; sostenidoVent++; }
    else sostenidoVent = 0;
    if (sostenidoVent > 30) { baseDb = rmsVentDb; sostenidoVent = 0; }
  }

  // Espectro del bloque más fuerte (ventana de Hann)
  for (int i = 0; i < N; i++) {
    re[i] = bloqueMax[i] * (0.5f - 0.5f * cosf(2.0f * PI * i / (N - 1)));
    im[i] = 0;
  }
  fft(re, im, N);

  float total = 0, mx = 0;
  int kmax = 2;
  for (int k = 2; k < N / 2 - 1; k++) {
    pw[k] = re[k] * re[k] + im[k] * im[k];
    total += pw[k];
    if (pw[k] > mx) { mx = pw[k]; kmax = k; }
  }
  float tonal = total > 0 ? (pw[kmax - 1] + pw[kmax] + pw[kmax + 1]) / total : 0;
  float freq  = kmax * (float)FS / N;

  if (listo && tonal >= TONAL_MIN && rmsVentDb > PISO_TONAL_DB &&
      fabsf(freq - freqPrev) <= 62.5f) tonalCont++;
  else tonalCont = 0;
  freqPrev = freq;
  if (tonalCont >= TONAL_VENTANAS) an |= AUD_BIT_MECANICO;

  // ---------- Envío ----------
  PaqueteAudio p = {};
  llenarCabecera(p.h, TIPO_AUDIO, NODO_ID);
  p.leq_db          = rmsVentDb + OFFSET_SPL_DB;
  p.lmax_db         = maxRms    + OFFSET_SPL_DB;
  p.pico_db         = picoVent  + OFFSET_SPL_DB;
  p.base_db         = baseDb    + OFFSET_SPL_DB;
  p.freq_dom_hz     = freq;
  p.tonal           = tonal;
  p.bloques_fuertes = (uint8_t)min(fuertes, 255);
  p.calibrando      = listo ? 0 : 1;
  p.anomalias       = an;
  espnowEnviar(&p, sizeof(p));

  Serial.printf("%lu,%.1f,%.1f,%.1f,%.1f,%.0f,%.2f,%d,%d,0x%02X,%d\n",
                millis(), p.leq_db, p.lmax_db, p.pico_db, p.base_db,
                freq, tonal, fuertes, p.calibrando, an, g_ultimoOk ? 1 : 0);

  // Reiniciar acumuladores
  bloquesVent = 0; fuertes = 0; potAcum = 0;
  picoVent = -200; maxRms = -200; maxCrest = 0;
}
