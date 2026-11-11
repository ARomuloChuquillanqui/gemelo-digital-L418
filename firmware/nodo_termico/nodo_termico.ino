// Nodo térmico (MLX90640) -> ESP-NOW -> Gateway ESP32 v4
// Conteo de personas en la puerta del salón L418.
// Montaje: marco superior de la puerta (2.3 m), mirando hacia abajo.
// Al encender, mantener la puerta DESPEJADA unos 3 s (calibración del fondo).
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>     // Librería "Adafruit MLX90640"
#include "paquetes.h"
#include "espnow_nodo.h"

// ================= RED =================
uint8_t MAC_GATEWAY[6] = {0x58, 0x2A, 0xBD, 0x72, 0x03, 0xDC}; // MAC STA del ESP32 v4
const char*   SSID_ROUTER   = "TU_RED_WIFI";   // red del gateway (solo para hallar el canal)
const uint8_t CANAL_DEFECTO = 1;
const uint8_t NODO_ID       = 5;

// ================= PINES I2C =================
#define PIN_SDA 21
#define PIN_SCL 22

// ================= ORIENTACIÓN =================
// true : las personas avanzan a lo largo del eje de 32 píxeles (55°)
// false: avanzan a lo largo del eje de 24 píxeles (35°)
#define PASO_EN_EJE_X  true
// +1 si avanzar en la coordenada de paso significa ENTRAR al salón.
// Si al probar cuenta "salida" cuando alguien entra, cambiar a -1.
const int SENTIDO_ENTRADA = +1;
// 1: imprime cada segundo un mapa de calor en el Monitor Serie (para instalar y orientar)
#define DEBUG_MAPA 0

// ================= PARÁMETROS (calculados para 2.3 m) =================
// Cabeza a ~0.6 m del sensor: cada píxel mide ~2 cm y una cabeza ocupa ~9 x 11 píxeles.
const float DELTA_MIN       = 1.5;    // °C mínimos sobre el fondo para marcar un píxel caliente
const float K_SIGMA         = 4.0;    // umbral = máx(DELTA_MIN, K_SIGMA * ruido del sensor)
const float DELTA_PICO      = 2.5;    // °C sobre el fondo para que un pico cuente como cabeza
const int   AREA_MIN        = 25;     // píxeles mínimos de una región válida
const int   AREA_MIN_PARTE  = 15;     // píxeles mínimos de cada persona al separar una fusión
const float R_PICO          = 10.0;   // distancia mínima entre dos cabezas (píxeles, ~20 cm)
const float GATE            = 12.0;   // desplazamiento máximo de una persona entre cuadros (píxeles)
const int   MAX_PERDIDOS    = 2;      // cuadros sin ver a alguien antes de cerrar su trayectoria
const int   VISTOS_MIN      = 2;      // cuadros mínimos para que una trayectoria sea válida
const float FRAC_LINEA_A    = 0.35;   // línea virtual A (fracción del eje de paso)
const float FRAC_LINEA_B    = 0.65;   // línea virtual B
const float FRAC_DESPL_MIN  = 0.35;   // desplazamiento mínimo para el conteo de respaldo
const float ALFA_FONDO      = 0.02;   // adaptación del fondo donde no hay nadie (~6 s)
const float ALFA_FONDO_OCUP = 0.001;  // adaptación muy lenta bajo objetos (absorbe objetos olvidados)
const int   N_CALIB         = 24;     // cuadros de calibración (~3 s a 8 cuadros/s)
const unsigned long PERIODO_MS        = 5000;
const unsigned long MIN_ENTRE_ENVIOS  = 300;     // envío rápido tras un cruce
const unsigned long OBJ_ESTATICO_MS   = 60000;   // algo caliente quieto más de 1 min
const unsigned long RESET_INACTIVO_MS = 5UL * 3600UL * 1000UL; // 5 h sin cruces -> ocupación = 0
const int   FALLOS_PARA_ALARMA = 5;

// ================= ESTADO =================
const int COLS = 32, FILAS = 24, NPIX = COLS * FILAS;
const int   L_PASO  = PASO_EN_EJE_X ? COLS : FILAS;
const float LINEA_A = FRAC_LINEA_A * (L_PASO - 1);
const float LINEA_B = FRAC_LINEA_B * (L_PASO - 1);
const float CENTRO  = 0.5f * (L_PASO - 1);

Adafruit_MLX90640 mlx;
bool sensorOk = false;

float   T[NPIX];        // cuadro actual (°C)
float   fondo[NPIX];    // modelo del fondo (°C)
float   m2[NPIX];       // acumulador de varianza (solo calibración)
float   dT[NPIX];       // T - fondo
int16_t etiqueta[NPIX]; // -1 = fondo; >= 0 = región caliente
int16_t cola[NPIX];     // píxeles de la región en análisis
float   umbral = DELTA_MIN;
int     nCalib = 0;

struct Deteccion { float p, q; bool fusion; };
const int MAX_DET = 6;
Deteccion det[MAX_DET];
int nDet = 0;

struct Pista {
  bool activa;
  float p, q;            // posición actual (p = eje de paso, q = transversal)
  float p0, pmin, pmax;  // posición inicial y extremos recorridos en p
  uint8_t vistos, perdidos;
  bool contada, dudosa;
  unsigned long t0;
};
const int MAX_PISTAS = 6;
Pista pistas[MAX_PISTAS];

// Acumuladores de la ventana de envío
int      ocupacion   = 0;
uint16_t entradasVent = 0, salidasVent = 0;
uint8_t  dudososVent = 0, maxEnVista = 0, anomVent = 0;
float    tMaxVent = -100;
bool     hayEvento = false;
unsigned long tUltEnvio = 0, tUltActividad = 0, tUltReintento = 0;
int      fallosSeguidos = 0;

inline float coordPaso(int c, int f)  { return PASO_EN_EJE_X ? c : f; }
inline float coordTrans(int c, int f) { return PASO_EN_EJE_X ? f : c; }

float distPix(int i, int j) {
  float dc = (i % COLS) - (j % COLS), df = (i / COLS) - (j / COLS);
  return sqrtf(dc * dc + df * df);
}

// ================= SENSOR =================
bool iniciarSensor() {
  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) return false;
  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_16_HZ);   // ~8 cuadros completos por segundo
  Wire.setClock(1000000);               // I2C rápido; si hay errores de lectura, bajar a 400000
  return true;
}

bool leerCuadro() {
  if (mlx.getFrame(T) != 0) return false;
  for (int i = 0; i < NPIX; i++)
    if (isnan(T[i]) || T[i] < -40 || T[i] > 300) return false;
  return true;
}

// ================= FONDO =================
// Calibración: media y ruido de cada píxel con la puerta despejada (algoritmo de Welford).
void calibrar() {
  nCalib++;
  for (int i = 0; i < NPIX; i++) {
    float d = T[i] - fondo[i];
    fondo[i] += d / nCalib;
    m2[i] += d * (T[i] - fondo[i]);
  }
  if (nCalib == N_CALIB) {
    double s = 0;
    for (int i = 0; i < NPIX; i++) s += sqrtf(m2[i] / (N_CALIB - 1));
    float sigma = s / NPIX;
    umbral = fmaxf(DELTA_MIN, K_SIGMA * sigma);
    Serial.printf("# Calibracion lista: ruido=%.2f C, umbral=%.2f C\n", sigma, umbral);
  }
}

// El fondo se adapta lento donde no hay nadie y casi nada bajo las personas.
void actualizarFondo() {
  for (int i = 0; i < NPIX; i++) {
    bool ocupado = etiqueta[i] != -1;
    if (!ocupado) {   // un píxel de margen alrededor de cada región
      int c = i % COLS, f = i / COLS;
      if ((c > 0 && etiqueta[i - 1] != -1) || (c < COLS - 1 && etiqueta[i + 1] != -1) ||
          (f > 0 && etiqueta[i - COLS] != -1) || (f < FILAS - 1 && etiqueta[i + COLS] != -1))
        ocupado = true;
    }
    float a = ocupado ? ALFA_FONDO_OCUP : ALFA_FONDO;
    fondo[i] += a * (T[i] - fondo[i]);
  }
}

// ================= DETECCIÓN =================
void agregarDeteccion(float p, float q, bool fusion) {
  if (nDet < MAX_DET) det[nDet++] = {p, q, fusion};
}

// Analiza una región caliente (píxeles en cola[0..n)) y la convierte en 0, 1 o más personas.
void analizarRegion(int n) {
  if (n < AREA_MIN) return;                         // muy pequeña: ruido
  float maxd = 0;
  for (int k = 0; k < n; k++) maxd = fmaxf(maxd, dT[cola[k]]);
  if (maxd < DELTA_PICO) return;                    // poco contraste: no es una persona

  // 1) Picos locales de temperatura = posibles cabezas (ordenados de mayor a menor)
  const int MAXP = 8;
  int picos[MAXP];
  int np = 0;
  for (int k = 0; k < n; k++) {
    int i = cola[k];
    if (dT[i] < DELTA_PICO) continue;
    int c = i % COLS, f = i / COLS;
    bool esPico = true;
    for (int df = -1; df <= 1 && esPico; df++)
      for (int dc = -1; dc <= 1; dc++) {
        int cc = c + dc, ff = f + df;
        if ((!dc && !df) || cc < 0 || cc >= COLS || ff < 0 || ff >= FILAS) continue;
        if (dT[ff * COLS + cc] > dT[i]) { esPico = false; break; }
      }
    if (!esPico) continue;
    if (np < MAXP) picos[np++] = i;
    else if (dT[i] > dT[picos[MAXP - 1]]) picos[MAXP - 1] = i;
    else continue;
    for (int a = np - 1; a > 0 && dT[picos[a]] > dT[picos[a - 1]]; a--) {
      int t = picos[a]; picos[a] = picos[a - 1]; picos[a - 1] = t;
    }
  }

  // 2) Supresión de no máximos: dos cabezas deben estar separadas al menos R_PICO
  int acept[MAX_DET];
  int na = 0;
  for (int a = 0; a < np && na < MAX_DET; a++) {
    bool lejos = true;
    for (int b = 0; b < na; b++)
      if (distPix(picos[a], acept[b]) < R_PICO) { lejos = false; break; }
    if (lejos) acept[na++] = picos[a];
  }

  // 3) Repartir los píxeles de la región entre las cabezas y calcular centros de calor
  float sw[MAX_DET] = {0}, sp[MAX_DET] = {0}, sq[MAX_DET] = {0};
  int area[MAX_DET] = {0};
  float swT = 0, spT = 0, sqT = 0;
  for (int k = 0; k < n; k++) {
    int i = cola[k], c = i % COLS, f = i / COLS;
    int mejor = 0;
    for (int a = 1; a < na; a++)
      if (distPix(i, acept[a]) < distPix(i, acept[mejor])) mejor = a;
    float w = dT[i];
    sw[mejor] += w; sp[mejor] += w * coordPaso(c, f); sq[mejor] += w * coordTrans(c, f);
    area[mejor]++;
    swT += w; spT += w * coordPaso(c, f); sqT += w * coordTrans(c, f);
  }
  int partes = 0;
  for (int a = 0; a < na; a++) if (area[a] >= AREA_MIN_PARTE) partes++;

  if (partes <= 1) {
    agregarDeteccion(spT / swT, sqT / swT, false);  // una sola persona
  } else {
    for (int a = 0; a < na; a++)                    // fusión: varias personas juntas
      if (area[a] >= AREA_MIN_PARTE) agregarDeteccion(sp[a] / sw[a], sq[a] / sw[a], true);
  }
}

// Agrupa los píxeles calientes en regiones conexas (vecindad de 8).
void detectar() {
  nDet = 0;
  for (int i = 0; i < NPIX; i++) { dT[i] = T[i] - fondo[i]; etiqueta[i] = -1; }
  int16_t region = 0;
  for (int s = 0; s < NPIX; s++) {
    if (etiqueta[s] != -1 || dT[s] <= umbral) continue;
    int cab = 0, fin = 0;
    cola[fin++] = s;
    etiqueta[s] = region;
    while (cab < fin) {
      int i = cola[cab++], c = i % COLS, f = i / COLS;
      for (int df = -1; df <= 1; df++)
        for (int dc = -1; dc <= 1; dc++) {
          int cc = c + dc, ff = f + df;
          if ((!dc && !df) || cc < 0 || cc >= COLS || ff < 0 || ff >= FILAS) continue;
          int j = ff * COLS + cc;
          if (etiqueta[j] == -1 && dT[j] > umbral) { etiqueta[j] = region; cola[fin++] = j; }
        }
    }
    analizarRegion(fin);
    region++;
  }
}

// ================= CONTEO =================
void registrarCruce(Pista &t, int dirPaso, bool respaldo) {
  t.contada = true;
  bool dudoso = respaldo || t.dudosa;
  if (dirPaso * SENTIDO_ENTRADA > 0) {
    entradasVent++;
    ocupacion++;
  } else {
    salidasVent++;
    if (ocupacion > 0) ocupacion--;
    else { dudoso = true; anomVent |= TER_BIT_CONTEO_CORREGIDO; }  // evitaría ocupación negativa
  }
  if (dudoso && dudososVent < 255) dudososVent++;
  hayEvento = true;
  tUltActividad = millis();
}

// Regla principal: la trayectoria nació antes de la línea A y llegó más allá de la B (o al revés).
void evaluarCruce(Pista &t) {
  if (t.contada || t.vistos < VISTOS_MIN) return;
  if (t.p0 <= LINEA_A && t.p >= LINEA_B)      registrarCruce(t, +1, false);
  else if (t.p0 >= LINEA_B && t.p <= LINEA_A) registrarCruce(t, -1, false);
}

// Al perder a alguien: si no se contó pero cruzó el centro con suficiente recorrido,
// se cuenta como cruce de respaldo (dudoso). Un retroceso no se cuenta.
void cerrarPista(Pista &t) {
  if (!t.contada && t.vistos >= VISTOS_MIN) {
    float despl = t.p - t.p0;
    bool cruzoCentro = t.pmin < CENTRO && t.pmax > CENTRO;
    if (cruzoCentro && fabsf(despl) >= FRAC_DESPL_MIN * (L_PASO - 1))
      registrarCruce(t, despl > 0 ? +1 : -1, true);
  }
  t.activa = false;
}

// Asocia cada detección con la trayectoria más cercana (emparejamiento voraz).
void seguir() {
  bool usada[MAX_DET] = {false};
  bool actualizada[MAX_PISTAS] = {false};

  while (true) {
    float mejor = GATE;
    int bp = -1, bd = -1;
    for (int p = 0; p < MAX_PISTAS; p++) {
      if (!pistas[p].activa || actualizada[p]) continue;
      for (int d = 0; d < nDet; d++) {
        if (usada[d]) continue;
        float dp = det[d].p - pistas[p].p, dq = det[d].q - pistas[p].q;
        float dist = sqrtf(dp * dp + dq * dq);
        if (dist < mejor) { mejor = dist; bp = p; bd = d; }
      }
    }
    if (bp < 0) break;
    Pista &t = pistas[bp];
    t.p = det[bd].p; t.q = det[bd].q;
    t.pmin = fminf(t.pmin, t.p); t.pmax = fmaxf(t.pmax, t.p);
    if (t.vistos < 255) t.vistos++;
    t.perdidos = 0;
    if (det[bd].fusion) t.dudosa = true;
    usada[bd] = true;
    actualizada[bp] = true;
    evaluarCruce(t);
  }

  for (int p = 0; p < MAX_PISTAS; p++)
    if (pistas[p].activa && !actualizada[p] && ++pistas[p].perdidos > MAX_PERDIDOS)
      cerrarPista(pistas[p]);

  for (int d = 0; d < nDet; d++) {
    if (usada[d]) continue;
    for (int p = 0; p < MAX_PISTAS; p++) {
      if (pistas[p].activa) continue;
      pistas[p] = {true, det[d].p, det[d].q, det[d].p, det[d].p, det[d].p,
                   1, 0, false, det[d].fusion, millis()};
      break;
    }
  }
}

// ================= DEPURACIÓN =================
void imprimirMapa() {
  static unsigned long t = 0;
  if (millis() - t < 1000) return;
  t = millis();
  int la = (int)roundf(LINEA_A), lb = (int)roundf(LINEA_B);
  Serial.printf("# ---- mapa: '#' cabeza, '+' caliente, lineas A/B | personas=%d ----\n", nDet);
  for (int f = 0; f < FILAS; f++) {
    char linea[COLS + 1];
    for (int c = 0; c < COLS; c++) {
      float d = dT[f * COLS + c];
      char ch = d > DELTA_PICO + 2 ? '#' : d > umbral ? '+' : d > umbral * 0.5f ? '.' : ' ';
      int pos = PASO_EN_EJE_X ? c : f;
      if (ch == ' ' && (pos == la || pos == lb)) ch = PASO_EN_EJE_X ? '|' : '-';
      linea[c] = ch;
    }
    linea[COLS] = 0;
    Serial.print("# ");
    Serial.println(linea);
  }
}

// ================= CICLO =================
void procesarCuadro() {
  float tmax = -100;
  for (int i = 0; i < NPIX; i++) tmax = fmaxf(tmax, T[i]);
  if (tmax > tMaxVent) tMaxVent = tmax;

  if (nCalib < N_CALIB) { calibrar(); return; }

  detectar();
  seguir();
  if (nDet > maxEnVista) maxEnVista = nDet;

  for (int p = 0; p < MAX_PISTAS; p++)
    if (pistas[p].activa && millis() - pistas[p].t0 > OBJ_ESTATICO_MS)
      anomVent |= TER_BIT_OBJETO_ESTATICO;

  actualizarFondo();
#if DEBUG_MAPA
  imprimirMapa();
#endif
}

void enviar() {
  float tf = 0;
  for (int i = 0; i < NPIX; i++) tf += fondo[i];
  tf /= NPIX;

  PaqueteTermico p = {};
  llenarCabecera(p.h, TIPO_TERMICO, NODO_ID);
  p.entradas          = entradasVent;
  p.salidas           = salidasVent;
  p.ocupacion_est     = (int16_t)ocupacion;
  p.personas_en_vista = maxEnVista;
  p.cruces_dudosos    = dudososVent;
  p.t_fondo           = nCalib > 0 ? tf : NAN;
  p.t_max             = tMaxVent > -100 ? tMaxVent : NAN;
  p.calibrando        = nCalib < N_CALIB ? 1 : 0;
  p.anomalias         = anomVent;
  espnowEnviar(&p, sizeof(p));

  Serial.printf("%lu,%u,%u,%d,%u,%u,%.2f,%.2f,%u,0x%02X,%d\n",
                millis(), entradasVent, salidasVent, ocupacion, maxEnVista, dudososVent,
                p.t_fondo, p.t_max, p.calibrando, anomVent, g_ultimoOk ? 1 : 0);

  entradasVent = salidasVent = 0;
  dudososVent = maxEnVista = anomVent = 0;
  tMaxVent = -100;
  hayEvento = false;
  tUltEnvio = millis();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(PIN_SDA, PIN_SCL);

  if (!espnowIniciar(MAC_GATEWAY, SSID_ROUTER, CANAL_DEFECTO))
    Serial.println("# ERROR: no se pudo iniciar ESP-NOW");
  Serial.printf("# Nodo termico id=%d  MAC=%s  canal=%d\n",
                NODO_ID, WiFi.macAddress().c_str(), g_canal);

  sensorOk = iniciarSensor();
  if (!sensorOk) Serial.println("# ERROR: no se detecta el MLX90640 (revisar SDA=21, SCL=22, 3.3V)");
  Serial.println("# Calibrando fondo: mantener la puerta despejada ~3 s.");
  Serial.println("ms,entradas,salidas,ocupacion_est,personas_en_vista,cruces_dudosos,"
                 "t_fondo,t_max,calibrando,anomalias,envio_ok");
  tUltEnvio = tUltActividad = millis();
}

void loop() {
  if (!sensorOk) {
    anomVent |= TER_BIT_FALLO;
    if (millis() - tUltReintento > 10000) {
      tUltReintento = millis();
      sensorOk = iniciarSensor();
    }
    delay(100);
  } else if (leerCuadro()) {       // bloquea hasta tener un cuadro (~125 ms)
    fallosSeguidos = 0;
    procesarCuadro();
  } else if (++fallosSeguidos >= FALLOS_PARA_ALARMA) {
    anomVent |= TER_BIT_FALLO;
  }

  // Deriva: sin cruces durante mucho tiempo, el salón se asume vacío
  if (ocupacion > 0 && millis() - tUltActividad > RESET_INACTIVO_MS) {
    ocupacion = 0;
    anomVent |= TER_BIT_CONTEO_CORREGIDO;
    tUltActividad = millis();
  }

  bool porPeriodo = millis() - tUltEnvio >= PERIODO_MS;
  bool porEvento  = hayEvento && millis() - tUltEnvio >= MIN_ENTRE_ENVIOS;
  if (porPeriodo || porEvento) enviar();
}
