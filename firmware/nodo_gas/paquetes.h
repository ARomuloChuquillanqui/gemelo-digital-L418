// paquetes.h — Formato común de datos ESP-NOW
// Debe ser IDÉNTICO en todos los nodos y en el gateway.
#pragma once
#include <stdint.h>

#define PROTO_VERSION 1

// Tipo de sensor
enum : uint8_t {
  TIPO_CO2     = 1,
  TIPO_AUDIO   = 2,
  TIPO_GAS     = 3,
  TIPO_LUZ     = 4,
  TIPO_TERMICO = 5,
  TIPO_MAGNETICO = 6,
  TIPO_AMBIENTE  = 7,
};

// Nivel de CO2
enum : uint8_t {
  CO2_NIVEL_NORMAL     = 0,
  CO2_NIVEL_ALERTA     = 1,
  CO2_NIVEL_CRITICO    = 2,
  CO2_NIVEL_CALENTANDO = 3,
  CO2_NIVEL_SIN_DATO   = 4
};

// Nivel de gas (MQ-2)
enum : uint8_t {
  GAS_NIVEL_NORMAL     = 0,
  GAS_NIVEL_ALERTA     = 1,
  GAS_NIVEL_CRITICO    = 2,
  GAS_NIVEL_CALENTANDO = 3,
  GAS_NIVEL_SIN_DATO   = 4
};

// Bits de anomalías CO2
#define CO2_BIT_FALLO_SENSOR  (1 << 0)
#define CO2_BIT_PICO_RAPIDO   (1 << 1)
#define CO2_BIT_ALERTA_VENT   (1 << 2)
#define CO2_BIT_CRITICO       (1 << 3)

// Bits de anomalías audio
#define AUD_BIT_FALLO         (1 << 0)
#define AUD_BIT_IMPACTO       (1 << 1)
#define AUD_BIT_SOSTENIDO     (1 << 2)
#define AUD_BIT_MECANICO      (1 << 3)

// Bits de anomalías térmico (MLX90640)
#define TER_BIT_FALLO             (1 << 0)
#define TER_BIT_CONTEO_CORREGIDO  (1 << 1)
#define TER_BIT_OBJETO_ESTATICO   (1 << 2)

// Bits de anomalías magnético (KY-024, puerta)
#define MAG_BIT_FALLO               (1 << 0)
#define MAG_BIT_ABIERTA_PROLONGADA  (1 << 1)

// Bits de anomalías ambiente (BME280)
#define AMB_BIT_FALLO               (1 << 0)
#define AMB_BIT_TEMP_FUERA_CONFORT  (1 << 1)
#define AMB_BIT_HUMEDAD_ALTA        (1 << 2)
#define AMB_BIT_CAMBIO_RAPIDO_TEMP  (1 << 3)

// Bits de anomalías luz (BH1750)
#define LUZ_BIT_FALLO              (1 << 0)
#define LUZ_BIT_ILUMINACION_BAJA   (1 << 1)

// Bits de anomalías gas (MQ-2)
#define GAS_BIT_FALLO          (1 << 0)
#define GAS_BIT_GAS_DETECTADO  (1 << 1)
#define GAS_BIT_CRITICO        (1 << 2)

#define TEMP_SIN_DATO  (-128)

// Cabecera común (11 bytes)
struct __attribute__((packed)) Cabecera {
  uint8_t  version;     // PROTO_VERSION
  uint8_t  tipo;        // TIPO_CO2, TIPO_AUDIO...
  uint8_t  nodo_id;     // id único en todo el proyecto
  uint32_t seq;         // contador para detectar pérdidas
  uint32_t uptime_ms;   // millis() del nodo
};

// CO2 (19 bytes)
struct __attribute__((packed)) PaqueteCO2 {
  Cabecera h;
  int16_t  co2_ppm;     // -1 sin respuesta, -2 checksum inválido
  int16_t  delta_30s;   // ppm
  int8_t   temp_c;      // temperatura interna aprox. (TEMP_SIN_DATO si no hay)
  uint8_t  calentando;  // 1 durante los 3 min iniciales
  uint8_t  nivel;       // CO2_NIVEL_*
  uint8_t  anomalias;   // CO2_BIT_*
};

// Audio (38 bytes). Niveles en dB SPL estimados.
struct __attribute__((packed)) PaqueteAudio {
  Cabecera h;
  float    leq_db;          // nivel equivalente del segundo
  float    lmax_db;         // máximo RMS de bloque (32 ms)
  float    pico_db;         // pico de muestra
  float    base_db;         // ruido de fondo estimado
  float    freq_dom_hz;     // frecuencia dominante del bloque más fuerte
  float    tonal;           // 0..1, concentración espectral
  uint8_t  bloques_fuertes; // bloques de 32 ms sobre umbral
  uint8_t  calibrando;      // 1 durante los primeros ~5 s
  uint8_t  anomalias;       // AUD_BIT_*
};

// Térmico / conteo de personas (29 bytes)
struct __attribute__((packed)) PaqueteTermico {
  Cabecera h;
  uint16_t entradas;          // cruces hacia el salón desde el envío anterior
  uint16_t salidas;           // cruces hacia afuera desde el envío anterior
  int16_t  ocupacion_est;     // acumulado entradas - salidas (nunca negativo)
  uint8_t  personas_en_vista; // máximo de cabezas vistas a la vez en la ventana
  uint8_t  cruces_dudosos;    // cruces contados con baja confianza en la ventana
  float    t_fondo;           // temperatura media del fondo (°C)
  float    t_max;             // temperatura máxima vista en la ventana (°C)
  uint8_t  calibrando;        // 1 mientras aprende el fondo (~3 s)
  uint8_t  anomalias;         // TER_BIT_*
};

// Puerta / magnético (21 bytes)
struct __attribute__((packed)) PaqueteMagnetico {
  Cabecera h;
  uint8_t  puerta_abierta;    // estado al momento del envío (0 = cerrada, 1 = abierta)
  uint16_t aperturas;         // veces que se abrió desde el envío anterior
  float    seg_abierta;       // segundos que estuvo abierta desde el envío anterior
  uint16_t campo_raw;         // lectura analógica promediada (0-4095)
  uint8_t  anomalias;         // MAG_BIT_*
};

// Ambiente: temperatura, humedad y presión (24 bytes)
struct __attribute__((packed)) PaqueteAmbiente {
  Cabecera h;
  float    temp_c;     // °C (NAN si falla el sensor)
  float    hum_pct;    // % de humedad relativa
  float    pres_hpa;   // hPa
  uint8_t  anomalias;  // AMB_BIT_*
};

// Luz (29 bytes)
struct __attribute__((packed)) PaqueteLuz {
  Cabecera h;
  float    lux;           // promedio de la ventana (lectura en el techo)
  float    lux_min;       // mínimo de la ventana
  float    lux_max;       // máximo de la ventana
  float    lux_mesa_est;  // estimación sobre la mesa = lux * FACTOR_MESA
  int8_t   luces_cambio;  // +1 encendido brusco, -1 apagado brusco, 0 sin cambio
  uint8_t  anomalias;     // LUZ_BIT_*
};

// Gas / humo (22 bytes)
struct __attribute__((packed)) PaqueteGas {
  Cabecera h;
  float    v_ao;        // voltaje de salida del MQ-2 (V, escala del módulo)
  float    rs_r0;       // índice relativo: 1 = aire limpio, menor = presencia de gases (NAN si no está listo)
  uint8_t  do_alarma;   // 1 si la salida digital del comparador indica gas
  uint8_t  calentando;  // 1 durante el calentamiento o la calibración de R0
  uint8_t  anomalias;   // GAS_BIT_*
  float    ppm_glp_est;  // ppm de GLP equivalente, estimación referencial (NAN si no está listo)
  uint8_t  nivel;        // GAS_NIVEL_*
};

static_assert(sizeof(PaqueteCO2)   <= 250, "Paquete CO2 excede ESP-NOW");
static_assert(sizeof(PaqueteGas) <= 250, "Paquete gas excede ESP-NOW");
static_assert(sizeof(PaqueteLuz) <= 250, "Paquete luz excede ESP-NOW");
static_assert(sizeof(PaqueteAmbiente) <= 250, "Paquete ambiente excede ESP-NOW");
static_assert(sizeof(PaqueteMagnetico) <= 250, "Paquete magnetico excede ESP-NOW");
static_assert(sizeof(PaqueteTermico) <= 250, "Paquete termico excede ESP-NOW");
static_assert(sizeof(PaqueteAudio) <= 250, "Paquete audio excede ESP-NOW");
