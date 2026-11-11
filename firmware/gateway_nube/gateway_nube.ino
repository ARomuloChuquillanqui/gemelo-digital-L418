// Gateway ESP32 v4 (NUBE): ESP-NOW (nodos) -> JSON -> MQTT con TLS
// Versión para el servidor de producción (servidor_iot_prod): Mosquitto con usuario,
// contraseña y TLS en el puerto 8883. Decodifica los 7 tipos de nodo de paquetes.h.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <time.h>
#include <sys/time.h>
#include <stdarg.h>
#include "paquetes.h"

// ================= CONFIGURACIÓN =================
#define USAR_MQTT 1   // 0: solo Serial   1: publica en Mosquitto
#define USAR_TLS  1   // 1: servidor en la nube (8883, cifrado)   0: servidor local (1883)

const char* WIFI_SSID = "*******";   // MISMO valor que SSID_ROUTER en los nodos (2.4 GHz)
const char* WIFI_PASS = "TU_CLAVE_WIFI";    // NO subir la clave real al repositorio
// Debe coincidir con el usuario MQTT creado en preparar.sh (por defecto "grupo1"):
// el ACL solo deja publicar a cada usuario en utec/gemelo/<su usuario>/...
const char* GRUPO     = "grupo1";

#if USAR_MQTT
#include <PubSubClient.h>                // Librería "PubSubClient" de Nick O'Leary

#if USAR_TLS
#include <WiFiClientSecure.h>
const char*    MQTT_HOST = "TU-NOMBRE.REGION.cloudapp.azure.com"; // DOMINIO del .env de servidor_iot_prod
const uint16_t MQTT_PORT = 8883;
const char*    MQTT_USER = GRUPO;                  // usuario = grupo (lo exige el ACL)
const char*    MQTT_PASS = "CLAVE_MQTT_GRUPO1";    // MQTT_GRUPO1_PASS del .env

// Pega aquí el contenido COMPLETO de ca/ca.crt que imprime preparar.sh
const char* CA_CERT = R"PEM(
-----BEGIN CERTIFICATE-----
PEGA_AQUI_EL_CERTIFICADO
-----END CERTIFICATE-----
)PEM";
WiFiClientSecure wifiCli;
#else
const char*    MQTT_HOST = "IP_DEL_SERVIDOR";      // IPv4 de la laptop con Mosquitto
const uint16_t MQTT_PORT = 1883;
const char*    MQTT_USER = "";
const char*    MQTT_PASS = "";
WiFiClient wifiCli;
#endif

PubSubClient mqtt(wifiCli);
#endif

// ================= COLA DE RECEPCIÓN =================
// El callback de ESP-NOW debe ser corto: solo copia el paquete a una cola.
struct Mensaje {
  uint8_t mac[6];
  uint8_t datos[64];
  int     len;
  int64_t ts_ms;      // hora UTC de recepción en ms (0 si no hay NTP aún)
};
QueueHandle_t cola;
volatile uint32_t descartados = 0;

// ================= ESTADÍSTICAS POR NODO =================
uint32_t ultimoSeq[256];
bool     visto[256];
uint32_t perdidos[256];

// ================= TEXTOS =================
const char* const NIVEL_CO2[] = {"NORMAL", "ALERTA", "CRITICO", "CALENTANDO", "SIN_DATO"};
const char* const AN_CO2[] = {"FALLO_SENSOR", "CO2_PICO_RAPIDO", "CO2_ALERTA_VENTILACION", "CO2_CRITICO"};
const char* const AN_AUD[] = {"FALLO_AUDIO", "RUIDO_IMPACTO", "RUIDO_FUERTE_SOSTENIDO", "RUIDO_MECANICO_CONSTANTE"};
const char* const AN_TER[] = {"FALLO_SENSOR", "CONTEO_CORREGIDO", "OBJETO_ESTATICO"};
const char* const AN_MAG[] = {"FALLO_SENSOR", "PUERTA_ABIERTA_PROLONGADA"};
const char* const AN_AMB[] = {"FALLO_SENSOR", "TEMP_FUERA_CONFORT", "HUMEDAD_ALTA", "CAMBIO_RAPIDO_TEMP"};
const char* const AN_LUZ[] = {"FALLO_SENSOR", "ILUMINACION_BAJA"};
const char* const AN_GAS[] = {"FALLO_SENSOR", "GAS_DETECTADO", "GAS_CRITICO"};
const char* const NIVEL_GAS[] = {"NORMAL", "ALERTA", "CRITICO", "CALENTANDO", "SIN_DATO"};

// ================= JSON MÍNIMO =================
struct Json {
  char b[512];
  size_t n = 0;
  bool primero = true;
  void add(const char* fmt, ...) {
    if (n >= sizeof(b)) return;
    va_list a; va_start(a, fmt);
    n += vsnprintf(b + n, sizeof(b) - n, fmt, a);
    va_end(a);
  }
  void clave(const char* k) { add(primero ? "\"%s\":" : ",\"%s\":", k); primero = false; }
  void num(const char* k, long long v) { clave(k); add("%lld", v); }
  void flo(const char* k, float v, int d) { clave(k); if (isnan(v)) add("null"); else add("%.*f", d, v); }
  void str(const char* k, const char* v) { clave(k); add("\"%s\"", v); }
  void nul(const char* k) { clave(k); add("null"); }
};

// Anomalías como texto "A|B" (o "NINGUNA"): fácil de guardar en InfluxDB
void anomalias(Json& j, uint8_t bits, const char* const nombres[], int nb) {
  j.clave("anomalias");
  j.add("\"");
  bool primera = true;
  for (int i = 0; i < nb; i++) {
    if (bits & (1 << i)) { j.add(primera ? "%s" : "|%s", nombres[i]); primera = false; }
  }
  if (primera) j.add("NINGUNA");
  j.add("\"");
  j.num("anom_bits", bits);
}

int64_t ahoraMs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  if (tv.tv_sec < 1700000000) return 0;   // NTP aún no sincronizado
  return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

// ================= CALLBACK ESP-NOW =================
#if ESP_IDF_VERSION_MAJOR >= 5
void alRecibir(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
#else
void alRecibir(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len <= 0 || len > (int)sizeof(Mensaje::datos)) { descartados++; return; }
  Mensaje m;
  memcpy(m.mac, mac, 6);
  memcpy(m.datos, data, len);
  m.len   = len;
  m.ts_ms = ahoraMs();
  if (xQueueSend(cola, &m, 0) != pdTRUE) descartados++;
}

// ================= PROCESAR PAQUETE =================
void publicar(const char* topic, const char* json) {
  Serial.printf("%s %s\n", topic, json);
#if USAR_MQTT
  if (mqtt.connected()) mqtt.publish(topic, json);
#endif
}

void procesar(const Mensaje& m) {
  if (m.len < (int)sizeof(Cabecera)) { Serial.println("# paquete demasiado corto"); return; }

  Cabecera h;
  memcpy(&h, m.datos, sizeof(h));
  if (h.version != PROTO_VERSION) {
    Serial.printf("# version de protocolo %u desconocida\n", h.version);
    return;
  }

  // Paquetes perdidos (si seq retrocede, el nodo se reinició)
  uint8_t id = h.nodo_id;
  if (visto[id] && h.seq > ultimoSeq[id] + 1) perdidos[id] += h.seq - ultimoSeq[id] - 1;
  visto[id] = true;
  ultimoSeq[id] = h.seq;

  char mac[18];
  snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
           m.mac[0], m.mac[1], m.mac[2], m.mac[3], m.mac[4], m.mac[5]);

  Json j;
  j.add("{");
  j.num("nodo", id);
  j.str("mac", mac);
  j.num("seq", h.seq);
  if (m.ts_ms) j.num("ts", m.ts_ms); else j.nul("ts");
  j.num("uptime_ms", h.uptime_ms);
  j.num("perdidos", perdidos[id]);

  const char* tipo;

  if (h.tipo == TIPO_CO2 && m.len == (int)sizeof(PaqueteCO2)) {
    PaqueteCO2 p;
    memcpy(&p, m.datos, sizeof(p));
    tipo = "co2";
    j.str("tipo", tipo);
    j.num("co2_ppm", p.co2_ppm);
    j.num("delta_30s", p.delta_30s);
    if (p.temp_c == TEMP_SIN_DATO) j.nul("temp_c"); else j.num("temp_c", p.temp_c);
    j.num("calentando", p.calentando);
    j.str("nivel", p.nivel < 5 ? NIVEL_CO2[p.nivel] : "DESCONOCIDO");
    anomalias(j, p.anomalias, AN_CO2, 4);

  } else if (h.tipo == TIPO_AUDIO && m.len == (int)sizeof(PaqueteAudio)) {
    PaqueteAudio p;
    memcpy(&p, m.datos, sizeof(p));
    tipo = "audio";
    j.str("tipo", tipo);
    j.flo("leq_db", p.leq_db, 1);
    j.flo("lmax_db", p.lmax_db, 1);
    j.flo("pico_db", p.pico_db, 1);
    j.flo("base_db", p.base_db, 1);
    j.flo("freq_dom_hz", p.freq_dom_hz, 0);
    j.flo("tonal", p.tonal, 2);
    j.num("bloques_fuertes", p.bloques_fuertes);
    j.num("calibrando", p.calibrando);
    anomalias(j, p.anomalias, AN_AUD, 4);

  } else if (h.tipo == TIPO_TERMICO && m.len == (int)sizeof(PaqueteTermico)) {
    PaqueteTermico p;
    memcpy(&p, m.datos, sizeof(p));
    tipo = "termico";
    j.str("tipo", tipo);
    j.num("entradas", p.entradas);
    j.num("salidas", p.salidas);
    j.num("ocupacion_est", p.ocupacion_est);
    j.num("personas_en_vista", p.personas_en_vista);
    j.num("cruces_dudosos", p.cruces_dudosos);
    j.flo("t_fondo", p.t_fondo, 2);
    j.flo("t_max", p.t_max, 2);
    j.num("calibrando", p.calibrando);
    anomalias(j, p.anomalias, AN_TER, 3);

  } else if (h.tipo == TIPO_MAGNETICO && m.len == (int)sizeof(PaqueteMagnetico)) {
    PaqueteMagnetico p;
    memcpy(&p, m.datos, sizeof(p));
    tipo = "magnetico";
    j.str("tipo", tipo);
    j.num("puerta_abierta", p.puerta_abierta);
    j.num("aperturas", p.aperturas);
    j.flo("seg_abierta", p.seg_abierta, 1);
    j.num("campo_raw", p.campo_raw);
    anomalias(j, p.anomalias, AN_MAG, 2);

  } else if (h.tipo == TIPO_AMBIENTE && m.len == (int)sizeof(PaqueteAmbiente)) {
    PaqueteAmbiente p;
    memcpy(&p, m.datos, sizeof(p));
    tipo = "ambiente";
    j.str("tipo", tipo);
    j.flo("temp_c", p.temp_c, 2);
    j.flo("hum_pct", p.hum_pct, 1);
    j.flo("pres_hpa", p.pres_hpa, 2);
    anomalias(j, p.anomalias, AN_AMB, 4);

  } else if (h.tipo == TIPO_LUZ && m.len == (int)sizeof(PaqueteLuz)) {
    PaqueteLuz p;
    memcpy(&p, m.datos, sizeof(p));
    tipo = "luz";
    j.str("tipo", tipo);
    j.flo("lux", p.lux, 1);
    j.flo("lux_min", p.lux_min, 1);
    j.flo("lux_max", p.lux_max, 1);
    j.flo("lux_mesa_est", p.lux_mesa_est, 1);
    j.num("luces_cambio", p.luces_cambio);
    anomalias(j, p.anomalias, AN_LUZ, 2);

  } else if (h.tipo == TIPO_GAS && m.len == (int)sizeof(PaqueteGas)) {
    PaqueteGas p;
    memcpy(&p, m.datos, sizeof(p));
    tipo = "gas";
    j.str("tipo", tipo);
    j.flo("v_ao", p.v_ao, 3);
    j.flo("rs_r0", p.rs_r0, 3);
    j.flo("ppm_glp_est", p.ppm_glp_est, 0);
    j.str("nivel", p.nivel < 5 ? NIVEL_GAS[p.nivel] : "DESCONOCIDO");
    j.num("do_alarma", p.do_alarma);
    j.num("calentando", p.calentando);
    anomalias(j, p.anomalias, AN_GAS, 3);
  } else {
    Serial.printf("# paquete invalido: tipo=%u len=%d (de %s)\n", h.tipo, m.len, mac);
    return;
  }
  j.add("}");

  char topic[96];
  snprintf(topic, sizeof(topic), "utec/gemelo/%s/nodo%u/%s", GRUPO, id, tipo);
  publicar(topic, j.b);
}

// ================= WIFI / MQTT =================
void conectarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // sin ahorro de energía: no perder paquetes ESP-NOW
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("# Conectando a WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - t0 > 15000) {   // 1 = red no encontrada, 4 = rechazada, 6 = clave incorrecta
      Serial.printf("\n# Sin conexion. Estado=%d. Reintentando...\n", WiFi.status());
      t0 = millis();
    }
  }
  Serial.printf("\n# WiFi OK  IP=%s  canal=%d\n",
                WiFi.localIP().toString().c_str(), WiFi.channel());
}

#if USAR_MQTT
void mantenerMQTT() {
  static unsigned long tIntento = 0;
  // Con TLS se necesita la hora real para validar el certificado: esperar a NTP
  if (!mqtt.connected() && WiFi.status() == WL_CONNECTED && ahoraMs() != 0 &&
      millis() - tIntento > 5000) {
    tIntento = millis();
    char idCli[32], topicEstado[80];
    snprintf(idCli, sizeof(idCli), "gw-%s", WiFi.macAddress().c_str());
    snprintf(topicEstado, sizeof(topicEstado), "utec/gemelo/%s/gateway/estado", GRUPO);
    const char* u = MQTT_USER[0] ? MQTT_USER : nullptr;
    const char* pw = MQTT_PASS[0] ? MQTT_PASS : nullptr;
    // "offline" queda publicado automáticamente si el gateway se desconecta
    if (mqtt.connect(idCli, u, pw, topicEstado, 1, true, "offline")) {
      mqtt.publish(topicEstado, "online", true);
      Serial.println("# MQTT conectado");
    } else {
      Serial.printf("# MQTT fallo, estado=%d\n", mqtt.state());
    }
  }
  mqtt.loop();
}
#endif

// ================= SETUP / LOOP =================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("# Gateway ESP32 v4 (nube, MQTT con TLS)");

  cola = xQueueCreate(32, sizeof(Mensaje));
  conectarWiFi();
  configTime(0, 0, "pool.ntp.org", "time.google.com");   // hora UTC

  if (esp_now_init() != ESP_OK) {
    Serial.println("# ERROR: no se pudo iniciar ESP-NOW");
    while (1) delay(1000);
  }
  esp_now_register_recv_cb(alRecibir);

#if USAR_MQTT
#if USAR_TLS
  wifiCli.setCACert(CA_CERT);   // solo acepta el servidor firmado por nuestra CA
#endif
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);   // por defecto 256: el JSON de audio no entraría
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(10);
#endif

  Serial.printf("# Listo. MAC=%s  Esperando nodos...\n", WiFi.macAddress().c_str());
}

void loop() {
#if USAR_MQTT
  mantenerMQTT();
#endif

  Mensaje m;
  while (xQueueReceive(cola, &m, 0) == pdTRUE) procesar(m);

  static unsigned long tEstado = 0;
  if (millis() - tEstado > 30000) {
    tEstado = millis();
    Serial.printf("# estado: WiFi=%s canal=%d NTP=%s descartados=%lu\n",
                  WiFi.status() == WL_CONNECTED ? "OK" : "NO",
                  WiFi.channel(), ahoraMs() ? "OK" : "NO",
                  (unsigned long)descartados);
  }
  delay(2);
}
