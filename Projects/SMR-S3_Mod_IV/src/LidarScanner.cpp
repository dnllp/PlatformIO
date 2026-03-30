/*
 * ============================================================
 * LidarScanner.cpp — Implementación del driver YDLidar X3
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 */

#include "LidarScanner.h"
#include "esp_log.h"

static const char* TAG = "LidarScanner";

// ── Constructor ──────────────────────────────────────────────
LidarScanner::LidarScanner(HardwareSerial& serial, gpio_num_t pinMotor)
    : _serial(serial), _pinMotor(pinMotor),
      _parseState(ParseState::WAIT_HDR1),
      _hdrIdx(0), _ct(0), _lsn(0),
      _fsa(0.0f), _lsa(0.0f), _cs(0), _csCalc(0),
      _sampleIdx(0), _sampleByte(0),
      _ready(false), _validCount(0), _revCount(0),
      _mutex(nullptr)
{
    // Inicializar buffer de escaneo a cero
    for (uint16_t i = 0; i < LIDAR_BUCKET_COUNT; i++) {
        _scan[i] = { static_cast<float>(i), 0.0f, 0 };
    }
}

// ── init() ───────────────────────────────────────────────────
void LidarScanner::init(gpio_num_t rxPin, gpio_num_t txPin, uint32_t baudRate) {
    // Configurar UART2 con los pines dados
    _serial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    _serial.setRxBufferSize(4096); // Buffer grande para no perder bytes

    // Configurar pin MOTOCTRL como salida (LOW = motor apagado al inicio)
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << _pinMotor);
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level(_pinMotor, 0);

    // Crear mutex para acceso thread-safe al buffer
    _mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "YDLidar X3 inicializado (RX=%d TX=%d Motor=%d)",
             rxPin, txPin, _pinMotor);
}

// ── startScan() ──────────────────────────────────────────────
void LidarScanner::startScan() {
    _serial.flush();
    gpio_set_level(_pinMotor, 1);     // Encender motor de giro
    vTaskDelay(pdMS_TO_TICKS(500));   // Esperar estabilización del motor
    _sendCommand(YD_CMD_SCAN);        // Enviar comando de escaneo
    _parseState = ParseState::WAIT_HDR1;
    _ready      = false;
    _revCount   = 0;
    ESP_LOGI(TAG, "Escaneo iniciado.");
}

// ── stopScan() ───────────────────────────────────────────────
void LidarScanner::stopScan() {
    _sendCommand(YD_CMD_STOP);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(_pinMotor, 0);
    _serial.flush();
    ESP_LOGI(TAG, "Escaneo detenido.");
}

// ── update() — parser de paquetes ────────────────────────────
void LidarScanner::update() {
    while (_serial.available()) {
        uint8_t byte = static_cast<uint8_t>(_serial.read());

        switch (_parseState) {

            // ── Esperar primer byte del header (0xAA) ────────
            case ParseState::WAIT_HDR1:
                if (byte == YD_HDR_1) _parseState = ParseState::WAIT_HDR2;
                break;

            // ── Esperar segundo byte del header (0x55) ───────
            case ParseState::WAIT_HDR2:
                if (byte == YD_HDR_2) {
                    _hdrIdx    = 0;
                    _parseState = ParseState::READ_HEADER;
                } else {
                    _parseState = ParseState::WAIT_HDR1;
                }
                break;

            // ── Leer los 8 bytes restantes del header ────────
            // CT(1) + LSN(1) + FSA(2) + LSA(2) + CS(2)
            case ParseState::READ_HEADER:
                _hdrBuf[_hdrIdx++] = byte;
                if (_hdrIdx == 8) {
                    _ct  = _hdrBuf[0];
                    _lsn = _hdrBuf[1];

                    uint16_t rawFSA = (uint16_t)(_hdrBuf[2] | (_hdrBuf[3] << 8));
                    uint16_t rawLSA = (uint16_t)(_hdrBuf[4] | (_hdrBuf[5] << 8));
                    _cs = (uint16_t)(_hdrBuf[6] | (_hdrBuf[7] << 8));

                    // Ángulo real = (raw >> 1) / 64.0
                    _fsa = static_cast<float>(rawFSA >> 1) / 64.0f;
                    _lsa = static_cast<float>(rawLSA >> 1) / 64.0f;

                    // Calcular checksum parcial: XOR del header
                    _csCalc = YD_PKT_HEADER ^ ((uint16_t)_ct | ((uint16_t)_lsn << 8))
                              ^ rawFSA ^ rawLSA;

                    _sampleIdx  = 0;
                    _sampleByte = 0;

                    if (_lsn == 0) {
                        _parseState = ParseState::WAIT_HDR1;
                    } else {
                        _parseState = ParseState::READ_SAMPLES;
                    }
                }
                break;

            // ── Leer muestras: 2 bytes por punto ────────────
            case ParseState::READ_SAMPLES:
                _sampleBuf[_sampleByte++] = byte;
                if (_sampleByte == 2) {
                    uint16_t rawDist = (uint16_t)(_sampleBuf[0] | (_sampleBuf[1] << 8));
                    _csCalc ^= rawDist;
                    _processSample(_sampleIdx, rawDist);
                    _sampleIdx++;
                    _sampleByte = 0;
                }
                if (_sampleIdx >= _lsn) {
                    // Paquete completo
                    if (_ct == YD_CT_START) {
                        // Inicio de nueva revolución
                        _revCount++;
                        if (_revCount >= 2) _ready = true;
                    }
                    _parseState = ParseState::WAIT_HDR1;
                }
                break;
        }
    }
}

// ── getDistanceAt() ──────────────────────────────────────────
float LidarScanner::getDistanceAt(float angleDeg, float tolDeg) const {
    angleDeg = _normalizeAngle(angleDeg);

    float sum   = 0.0f;
    int   count = 0;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        for (uint16_t i = 0; i < LIDAR_BUCKET_COUNT; i++) {
            if (_scan[i].distance_mm < LIDAR_MIN_DIST_MM) continue;

            float diff = fabsf(_scan[i].angle_deg - angleDeg);
            if (diff > 180.0f) diff = 360.0f - diff;  // Cruce de 0°/360°

            if (diff <= tolDeg) {
                sum += _scan[i].distance_mm;
                count++;
            }
        }
        xSemaphoreGive(_mutex);
    }

    return (count > 0) ? (sum / count) : -1.0f;
}

// ── getMinInSector() ─────────────────────────────────────────
float LidarScanner::getMinInSector(float startDeg, float endDeg) const {
    startDeg = _normalizeAngle(startDeg);
    endDeg   = _normalizeAngle(endDeg);

    float minDist = LIDAR_MAX_DIST_MM + 1.0f;
    bool  found   = false;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        for (uint16_t i = 0; i < LIDAR_BUCKET_COUNT; i++) {
            float d = _scan[i].distance_mm;
            if (d < LIDAR_MIN_DIST_MM || d > LIDAR_MAX_DIST_MM) continue;

            float a = _scan[i].angle_deg;
            bool inSector;

            if (startDeg <= endDeg) {
                inSector = (a >= startDeg && a <= endDeg);
            } else {
                // Sector cruza el 0° (ej. 350° → 10°)
                inSector = (a >= startDeg || a <= endDeg);
            }

            if (inSector && d < minDist) {
                minDist = d;
                found   = true;
            }
        }
        xSemaphoreGive(_mutex);
    }

    return found ? minDist : -1.0f;
}

// ── getScanCopy() ────────────────────────────────────────────
void LidarScanner::getScanCopy(ScanPoint out[LIDAR_BUCKET_COUNT]) const {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(out, _scan, sizeof(_scan));
        xSemaphoreGive(_mutex);
    }
}

// ── _sendCommand() ───────────────────────────────────────────
void LidarScanner::_sendCommand(uint8_t cmd) {
    uint8_t pkt[2] = { YD_START_FLAG, cmd };
    _serial.write(pkt, 2);
}

// ── _processSample() ─────────────────────────────────────────
void LidarScanner::_processSample(uint8_t idx, uint16_t rawDist) {
    // Distancia real en mm = raw / 4.0 (formato Q2)
    float distMm = static_cast<float>(rawDist) / 4.0f;

    // Interpolar el ángulo de esta muestra dentro del paquete
    float angle;
    if (_lsn == 1) {
        angle = _fsa;
    } else {
        float step = (_lsa - _fsa);
        // Compensar cruce de 360°
        if (step < 0.0f) step += 360.0f;
        angle = _fsa + step * (static_cast<float>(idx) / (_lsn - 1));
    }
    angle = _normalizeAngle(angle);

    // Guardar en el bucket correspondiente (protegido por mutex)
    uint16_t bucket = static_cast<uint16_t>(angle) % LIDAR_BUCKET_COUNT;

    if (xSemaphoreTake(_mutex, 0) == pdTRUE) {  // No bloqueante en ISR path
        if (distMm >= LIDAR_MIN_DIST_MM && distMm <= LIDAR_MAX_DIST_MM) {
            _scan[bucket].angle_deg    = angle;
            _scan[bucket].distance_mm  = distMm;
            _scan[bucket].quality      = LIDAR_MIN_QUALITY;
            _validCount++;
        } else {
            _scan[bucket].distance_mm = 0.0f;
        }
        xSemaphoreGive(_mutex);
    }
}

// ── _normalizeAngle() ────────────────────────────────────────
float LidarScanner::_normalizeAngle(float deg) {
    while (deg < 0.0f)    deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}
