/*
 * ============================================================
 * LidarScanner.h — Módulo de Interfaz con YDLidar X3
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Plataforma : ESP32-S3
 * Sensor     : YDLidar X3 (YDLIDAR) — UART a 115200 bps
 * Nivel      : 4 — Navegación autónoma (SLAM)
 *
 * Descripción:
 *   Gestiona la comunicación UART con el YDLidar X3 y mantiene
 *   un escaneo circular de 360° actualizado continuamente en un
 *   buffer de puntos indexado por ángulo.
 *
 * Protocolo YDLidar X3 (paquetes de escaneo):
 *   Header    : 0xAA 0x55            (2 bytes)
 *   CT        : tipo de paquete      (1 byte)  0x00=normal, 0x01=inicio
 *   LSN       : cantidad de muestras (1 byte)
 *   FSA       : ángulo inicial Q6    (2 bytes, little-endian)
 *   LSA       : ángulo final Q6      (2 bytes, little-endian)
 *   CS        : checksum XOR         (2 bytes)
 *   Si_i      : distancia Q2 x LSN  (2 bytes por muestra)
 *
 *   Ángulo real  = (valor_Q6 >> 1) / 64.0   [grados]
 *   Distancia mm = valor_Q2 / 4.0            [mm]
 *
 * Conexión física (YDLidar X3 — adaptador 5V/3.3V incluido):
 *   TX del X3  → RX del ESP32-S3 (GPIO_NUM_16)
 *   RX del X3  → TX del ESP32-S3 (GPIO_NUM_17)
 *   DTR/MOTOCTRL → GPIO para encender/apagar el motor
 *   VCC        → 5V
 *   GND        → GND común
 *
 * Uso básico:
 *   LidarScanner lidar(Serial2, GPIO_NUM_4);
 *   lidar.init();
 *   lidar.startScan();
 *   // Desde task dedicado (Core 0):
 *   lidar.update();
 *   // Consultar:
 *   float dist = lidar.getDistanceAt(90.0f, 5.0f);
 * ============================================================
 */

#ifndef LIDAR_SCANNER_H
#define LIDAR_SCANNER_H

#include "Arduino.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <math.h>

// ── Configuración ────────────────────────────────────────────
static constexpr uint16_t LIDAR_BUCKET_COUNT = 360;    // 1 entrada por grado
static constexpr float    LIDAR_MAX_DIST_MM  = 8000.0f; // Rango máx X3: 8 m
static constexpr float    LIDAR_MIN_DIST_MM  = 100.0f;  // Zona ciega: 10 cm
static constexpr uint8_t  LIDAR_MIN_QUALITY  = 8;       // Descartar puntos débiles

// ── Protocolo YDLidar ────────────────────────────────────────
static constexpr uint8_t  YD_HDR_1      = 0xAA;
static constexpr uint8_t  YD_HDR_2      = 0x55;
static constexpr uint8_t  YD_CT_NORMAL  = 0x00;
static constexpr uint8_t  YD_CT_START   = 0x01;
static constexpr uint8_t  YD_CMD_STOP   = 0x65;
static constexpr uint8_t  YD_CMD_SCAN   = 0x60;
static constexpr uint8_t  YD_CMD_RESET  = 0x80;
static constexpr uint8_t  YD_START_FLAG = 0xA5;
static constexpr uint16_t YD_PKT_HEADER = 0x55AA;
static constexpr uint8_t  YD_HDR_SIZE   = 10;   // Bytes de cabecera por paquete

// ── Estructura de un punto ───────────────────────────────────
struct ScanPoint {
    float   angle_deg;    // 0.0 – 359.9°
    float   distance_mm;  // mm (0 = no válido)
    uint8_t quality;      // intensidad de la señal
};

// ── Clase principal ──────────────────────────────────────────
class LidarScanner {
public:
    /**
     * Constructor
     * @param serial    Puerto UART (ej. Serial2)
     * @param pinMotor  GPIO → pin DTR/MOTOCTRL del adaptador del X3
     *                  HIGH = motor encendido, LOW = apagado
     */
    LidarScanner(HardwareSerial& serial, gpio_num_t pinMotor);

    /**
     * Configura UART2 a 115200 bps, GPIO del motor y mutex de acceso.
     * Llamar una vez en setup() antes de startScan().
     */
    void init(gpio_num_t rxPin = GPIO_NUM_16, gpio_num_t txPin = GPIO_NUM_17,
              uint32_t baudRate = 115200);

    /** Enciende el motor y comienza la captura de paquetes. */
    void startScan();

    /** Detiene el motor y el escaneo. */
    void stopScan();

    /**
     * Parsea bytes disponibles en el UART y actualiza el buffer circular.
     * Llamar desde un task FreeRTOS dedicado en Core 0 (máxima frecuencia).
     */
    void update();

    /**
     * Distancia promedio en el ángulo dado ± tolerancia.
     * Thread-safe: usa mutex interno.
     * @param angleDeg  Ángulo objetivo [0-359°]
     * @param tolDeg    Tolerancia angular (±grados)
     * @return  Distancia en mm, o -1.0f si no hay datos válidos
     */
    float getDistanceAt(float angleDeg, float tolDeg = 5.0f) const;

    /**
     * Distancia mínima en un sector angular [startDeg, endDeg].
     * Útil para detectar el obstáculo más cercano en una dirección.
     */
    float getMinInSector(float startDeg, float endDeg) const;

    /**
     * Devuelve una copia completa del buffer de escaneo.
     * @param out   Array destino de tamaño LIDAR_BUCKET_COUNT
     */
    void getScanCopy(ScanPoint out[LIDAR_BUCKET_COUNT]) const;

    /** true si se han recibido al menos 2 revoluciones completas */
    bool isReady() const { return _ready; }

    /** Número de puntos válidos en la última revolución */
    uint16_t getValidPointCount() const { return _validCount; }

private:
    HardwareSerial& _serial;
    gpio_num_t      _pinMotor;

    // Buffer circular indexado por grado (0–359)
    ScanPoint _scan[LIDAR_BUCKET_COUNT];

    // Estado del parser de paquetes
    enum class ParseState : uint8_t {
        WAIT_HDR1, WAIT_HDR2, READ_HEADER, READ_SAMPLES
    };

    ParseState _parseState;
    uint8_t    _hdrBuf[YD_HDR_SIZE];
    uint8_t    _hdrIdx;

    uint8_t    _ct;         // Tipo de paquete
    uint8_t    _lsn;        // Número de muestras
    float      _fsa;        // Ángulo inicial (grados)
    float      _lsa;        // Ángulo final (grados)
    uint16_t   _cs;         // Checksum esperado
    uint16_t   _csCalc;     // Checksum calculado
    uint8_t    _sampleIdx;  // Muestra actual dentro del paquete

    uint8_t    _sampleBuf[2]; // Buffer de 2 bytes por muestra
    uint8_t    _sampleByte;   // Índice dentro de la muestra

    bool       _ready;
    uint16_t   _validCount;
    uint8_t    _revCount;   // Contador de revoluciones completas

    SemaphoreHandle_t _mutex;  // Protege el acceso al buffer _scan

    // ── Métodos internos ─────────────────────────────────────
    void _sendCommand(uint8_t cmd);
    void _processSample(uint8_t sampleIdx, uint16_t rawDist);
    void _interpolateAngles(uint8_t numSamples, float fsa, float lsa,
                            float* anglesOut) const;
    bool _verifyChecksum() const;
    static float _normalizeAngle(float deg);
};

#endif // LIDAR_SCANNER_H
