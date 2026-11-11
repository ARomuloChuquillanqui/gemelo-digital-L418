// espnow_nodo.h — Envío ESP-NOW desde un nodo sensor hacia el gateway
#pragma once
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "paquetes.h"

static uint8_t       g_macGateway[6];
static const char*   g_ssid = nullptr;
static uint8_t       g_canal = 1;
static uint32_t      g_seq = 0;
static volatile bool g_ultimoOk = false;
static volatile int  g_fallosSeguidos = 0;

// Confirmación de entrega (ACK de capa MAC).
// La firma del callback cambió en ESP-IDF 5.5 (core Arduino 3.3+).
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void alEnviar(const esp_now_send_info_t *info, esp_now_send_status_t st) {
#else
static void alEnviar(const uint8_t *mac, esp_now_send_status_t st) {
#endif
  g_ultimoOk = (st == ESP_NOW_SEND_SUCCESS);
  if (g_ultimoOk) g_fallosSeguidos = 0;
  else            g_fallosSeguidos++;
}

// Busca el canal de la red WiFi a la que está conectado el gateway.
static uint8_t buscarCanal(const char* ssid, uint8_t porDefecto) {
  int n = WiFi.scanNetworks();
  uint8_t canal = porDefecto;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == ssid) { canal = WiFi.channel(i); break; }
  }
  WiFi.scanDelete();
  return canal;
}

static void fijarCanal(uint8_t canal) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(canal, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  g_canal = canal;
}

static bool espnowIniciar(const uint8_t mac[6], const char* ssid, uint8_t canalDefecto) {
  memcpy(g_macGateway, mac, 6);
  g_ssid = ssid;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();                       // no se conecta al router, solo usa su canal
  fijarCanal(buscarCanal(ssid, canalDefecto));

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_send_cb(alEnviar);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;                        // 0 = canal actual
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

static void llenarCabecera(Cabecera &h, uint8_t tipo, uint8_t nodoId) {
  h.version   = PROTO_VERSION;
  h.tipo      = tipo;
  h.nodo_id   = nodoId;
  h.seq       = g_seq++;
  h.uptime_ms = millis();
}

static void espnowEnviar(const void* datos, size_t len) {
  esp_now_send(g_macGateway, (const uint8_t*)datos, len);

  // Muchos fallos seguidos: quizá el router cambió de canal. Re-escanear (~2 s).
  if (g_fallosSeguidos >= 10 && g_ssid) {
    g_fallosSeguidos = 0;
    fijarCanal(buscarCanal(g_ssid, g_canal));
  }
}
