/*
 * ============================================================
 * UltrasonicSensor.cpp — Implementación del Sensor HC-SR04
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 */

#include "UltrasonicSensor.h"
#include "rom/ets_sys.h"   // ets_delay_us()
#include "esp_log.h"

static const char* TAG = "UltrasonicSensor";

// ── Constructor ──────────────────────────────────────────────
UltrasonicSensor::UltrasonicSensor(gpio_num_t pinTrig, gpio_num_t pinEcho)
    : _pinTrig(pinTrig), _pinEcho(pinEcho),
      _echoStart(0), _echoDuration(0),
      _measuring(false), _lastDistanceCm(-1.0f)
{}

// ── init() ───────────────────────────────────────────────────
void UltrasonicSensor::init() {
    // ── Configurar TRIG como salida ──────────────────────────
    gpio_config_t trig_conf = {};
    trig_conf.pin_bit_mask = (1ULL << _pinTrig);
    trig_conf.mode         = GPIO_MODE_OUTPUT;
    trig_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    trig_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    trig_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&trig_conf);
    gpio_set_level(_pinTrig, 0);

    // ── Configurar ECHO como entrada con ISR en ambos flancos ─
    gpio_config_t echo_conf = {};
    echo_conf.pin_bit_mask = (1ULL << _pinEcho);
    echo_conf.mode         = GPIO_MODE_INPUT;
    echo_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    echo_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;   // Evita lecturas falsas
    echo_conf.intr_type    = GPIO_INTR_ANYEDGE;      // Flanco subida Y bajada
    gpio_config(&echo_conf);

    // ── Instalar servicio ISR y registrar handler ────────────
    gpio_install_isr_service(0);
    gpio_isr_handler_add(_pinEcho, _echoISR, this);

    ESP_LOGI(TAG, "HC-SR04 inicializado (TRIG=%d, ECHO=%d)", _pinTrig, _pinEcho);
}

// ── getDistanceCm() — modo bloqueante ────────────────────────
float UltrasonicSensor::getDistanceCm() {
    _echoDuration = 0;
    _measuring    = true;

    // Pulso TRIG de 10 µs
    gpio_set_level(_pinTrig, 1);
    ets_delay_us(US_TRIG_PULSE_US);
    gpio_set_level(_pinTrig, 0);

    // Esperar a que la ISR complete la medición (timeout = 25 ms)
    uint32_t waited = 0;
    while (_measuring && waited < US_TIMEOUT_US) {
        ets_delay_us(100);
        waited += 100;
    }

    if (_measuring) {
        // Timeout: no se recibió eco (objeto fuera de rango o ausente)
        _measuring       = false;
        _lastDistanceCm  = -1.0f;
        return -1.0f;
    }

    float dist = _usToCm(_echoDuration);
    _lastDistanceCm = (dist < US_MIN_DISTANCE_CM || dist > US_MAX_DISTANCE_CM)
                      ? -1.0f : dist;
    return _lastDistanceCm;
}

// ── triggerAsync() — modo no bloqueante ──────────────────────
void UltrasonicSensor::triggerAsync() {
    if (_measuring) return;  // Medición anterior aún en curso

    _echoDuration = 0;
    _measuring    = true;

    gpio_set_level(_pinTrig, 1);
    ets_delay_us(US_TRIG_PULSE_US);
    gpio_set_level(_pinTrig, 0);
    // La ISR actualizará _lastDistanceCm cuando llegue el eco
}

// ── isObstacleDetected() ─────────────────────────────────────
bool UltrasonicSensor::isObstacleDetected(float thresholdCm) const {
    return (_lastDistanceCm > 0.0f && _lastDistanceCm < thresholdCm);
}

// ── _echoISR() — ejecuta en IRAM ─────────────────────────────
void IRAM_ATTR UltrasonicSensor::_echoISR(void* arg) {
    UltrasonicSensor* self = static_cast<UltrasonicSensor*>(arg);

    if (gpio_get_level(self->_pinEcho) == 1) {
        // Flanco de SUBIDA: guardar timestamp de inicio
        self->_echoStart = esp_timer_get_time();
    } else {
        // Flanco de BAJADA: calcular duración del pulso
        self->_echoDuration  = esp_timer_get_time() - self->_echoStart;
        float dist           = self->_usToCm(self->_echoDuration);
        self->_lastDistanceCm = (dist < US_MIN_DISTANCE_CM || dist > US_MAX_DISTANCE_CM)
                                ? -1.0f : dist;
        self->_measuring = false;
    }
}

// ── _usToCm() ────────────────────────────────────────────────
float UltrasonicSensor::_usToCm(int64_t durationUs) {
    return static_cast<float>(durationUs) / US_SOUND_FACTOR;
}
