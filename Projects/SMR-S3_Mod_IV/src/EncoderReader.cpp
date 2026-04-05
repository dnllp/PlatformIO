/*
 * ============================================================
 * EncoderReader.cpp — Implementación del Lector de Encoders
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 */

#include "EncoderReader.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "EncoderReader";

// ── Constructor ──────────────────────────────────────────────
EncoderReader::EncoderReader(gpio_num_t pinA, gpio_num_t pinB, pcnt_unit_t unit)
    : _pinA(pinA), _pinB(pinB), _unit(unit),
      _accumulator(0), _lastCount(0)
{}

// ── init() ───────────────────────────────────────────────────
void EncoderReader::init() {
    // ── Configuración del canal 0: señal A controla el conteo,
    //    señal B es el control de dirección
    pcnt_config_t cfg0 = {};
    cfg0.pulse_gpio_num = _pinA;
    cfg0.ctrl_gpio_num  = _pinB;
    cfg0.unit           = _unit;
    cfg0.channel        = PCNT_CHANNEL_0;
    // Flanco de subida de A: +1 si B=LOW, -1 si B=HIGH
    cfg0.pos_mode       = PCNT_COUNT_INC;
    cfg0.neg_mode       = PCNT_COUNT_DEC;
    // Flanco de bajada de A: -1 si B=LOW, +1 si B=HIGH
    cfg0.lctrl_mode     = PCNT_MODE_REVERSE;
    cfg0.hctrl_mode     = PCNT_MODE_KEEP;
    cfg0.counter_h_lim  = PCNT_HIGH_LIMIT;
    cfg0.counter_l_lim  = PCNT_LOW_LIMIT;
    pcnt_unit_config(&cfg0);

    // ── Configuración del canal 1: señal B controla el conteo,
    //    señal A es el control de dirección (cuadratura completa x4)
    pcnt_config_t cfg1 = {};
    cfg1.pulse_gpio_num = _pinB;
    cfg1.ctrl_gpio_num  = _pinA;
    cfg1.unit           = _unit;
    cfg1.channel        = PCNT_CHANNEL_1;
    cfg1.pos_mode       = PCNT_COUNT_DEC;
    cfg1.neg_mode       = PCNT_COUNT_INC;
    cfg1.lctrl_mode     = PCNT_MODE_KEEP;
    cfg1.hctrl_mode     = PCNT_MODE_REVERSE;
    cfg1.counter_h_lim  = PCNT_HIGH_LIMIT;
    cfg1.counter_l_lim  = PCNT_LOW_LIMIT;
    pcnt_unit_config(&cfg1);

    // ── Filtro antirebote: ignora pulsos menores a ~1 µs
    //    (valor en ciclos del APB clock a 80 MHz: 80 = 1 µs)
    pcnt_set_filter_value(_unit, 100);
    pcnt_filter_enable(_unit);

    // ── Habilitar eventos de overflow para acumulación de 32 bits
    pcnt_event_enable(_unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(_unit, PCNT_EVT_L_LIM);

    // ── Registrar ISR de overflow (pasa 'this' como argumento)
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(_unit, _overflowISR, this);

    // ── Limpiar y arrancar el contador
    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    pcnt_counter_resume(_unit);

    ESP_LOGI(TAG, "Unidad PCNT %d inicializada (pines A=%d, B=%d)",
             _unit, _pinA, _pinB);
}

// ── reset() ──────────────────────────────────────────────────
void EncoderReader::reset() {
    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    _accumulator = 0;
    _lastCount   = 0;
    pcnt_counter_resume(_unit);
}

// ── getCount() ───────────────────────────────────────────────
int32_t EncoderReader::getCount() {
    int16_t raw = 0;
    pcnt_get_counter_value(_unit, &raw);
    return _accumulator + raw;
}

// ── getSpeed() ───────────────────────────────────────────────
float EncoderReader::getSpeed(uint32_t dtMs) {
    if (dtMs == 0) return 0.0f;

    int32_t currentCount = getCount();
    float   delta        = static_cast<float>(currentCount - _lastCount);
    _lastCount           = currentCount;

    // Velocidad = pulsos / tiempo (en segundos)
    return delta / (static_cast<float>(dtMs) / 1000.0f);
}

// ── pulsesToCm() (estático) ──────────────────────────────────
float EncoderReader::pulsesToCm(int32_t pulses, uint16_t pulsesPerRev, float wheelDiamCm) {
    // Circunferencia = π × diámetro
    // Distancia = (pulsos / PPR) × circunferencia
    float circumference = M_PI * wheelDiamCm;
    return (static_cast<float>(pulses) / static_cast<float>(pulsesPerRev)) * circumference;
}

// ── _overflowISR() (ISR, ejecuta en IRAM) ────────────────────
void IRAM_ATTR EncoderReader::_overflowISR(void* arg) {
    EncoderReader* self = static_cast<EncoderReader*>(arg);
    uint32_t status = 0;
    pcnt_get_event_status(self->_unit, &status);

    // Acumular el rango completo según la dirección del overflow
    if (status & PCNT_EVT_H_LIM) {
        self->_accumulator += PCNT_HIGH_LIMIT;
    }
    if (status & PCNT_EVT_L_LIM) {
        self->_accumulator += PCNT_LOW_LIMIT;
    }
}
