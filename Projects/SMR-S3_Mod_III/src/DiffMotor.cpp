/*
 * ============================================================
 * DiffMotor.cpp — Implementación del Módulo de Motores DC
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Control real del L298N usando IN1 e IN2 con PWM en cada pin.
 * ENA/ENB debe estar en HIGH (puente soldado o pin a 5V fijo).
 * ============================================================
 */

#include "DiffMotor.h"
#include <cstdlib>  // abs()

// ── Constructor ──────────────────────────────────────────────
DiffMotor::DiffMotor(gpio_num_t pinIN1, gpio_num_t pinIN2,
                     ledc_channel_t chIN1, ledc_channel_t chIN2)
    : _pinIN1(pinIN1), _pinIN2(pinIN2),
      _chIN1(chIN1),   _chIN2(chIN2),
      _currentSpeed(0)
{}

// ── init() ───────────────────────────────────────────────────
void DiffMotor::init() {
    // Configurar timer LEDC (compartido por todos los canales de motor)
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode      = MOTOR_SPEED_MODE;
    timer_conf.duty_resolution = MOTOR_RESOLUTION;
    timer_conf.timer_num       = MOTOR_TIMER;
    timer_conf.freq_hz         = MOTOR_FREQ_HZ;
    timer_conf.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    // Inicializar ambos canales en LOW (motor frenado)
    _initChannel(_pinIN1, _chIN1);
    _initChannel(_pinIN2, _chIN2);
}

// ── drive() ──────────────────────────────────────────────────
void DiffMotor::drive(int speed) {
    // Limitar al rango válido
    if (speed >  MOTOR_DUTY_MAX) speed =  MOTOR_DUTY_MAX;
    if (speed < -MOTOR_DUTY_MAX) speed = -MOTOR_DUTY_MAX;

    _currentSpeed = speed;

    if (speed > 0) {
        // Avance: IN1 = PWM, IN2 = 0
        _applyDual(_chIN1, _chIN2, static_cast<uint32_t>(speed));
    } else if (speed < 0) {
        // Retroceso: IN1 = 0, IN2 = PWM
        _applyDual(_chIN2, _chIN1, static_cast<uint32_t>(-speed));
    } else {
        // Freno: IN1 = 0, IN2 = 0
        stop();
    }
}

// ── stop() ───────────────────────────────────────────────────
void DiffMotor::stop() {
    _currentSpeed = 0;
    ledc_set_duty(MOTOR_SPEED_MODE, _chIN1, 0);
    ledc_update_duty(MOTOR_SPEED_MODE, _chIN1);
    ledc_set_duty(MOTOR_SPEED_MODE, _chIN2, 0);
    ledc_update_duty(MOTOR_SPEED_MODE, _chIN2);
}

// ── _initChannel() (privado) ─────────────────────────────────
void DiffMotor::_initChannel(gpio_num_t pin, ledc_channel_t ch) {
    ledc_channel_config_t channel_conf = {};
    channel_conf.gpio_num   = pin;
    channel_conf.speed_mode = MOTOR_SPEED_MODE;
    channel_conf.channel    = ch;
    channel_conf.timer_sel  = MOTOR_TIMER;
    channel_conf.duty       = 0;   // Inicia en LOW (freno)
    channel_conf.hpoint     = 0;
    ledc_channel_config(&channel_conf);
}

// ── _applyDual() (privado) ───────────────────────────────────
void DiffMotor::_applyDual(ledc_channel_t chActive, ledc_channel_t chIdle, uint32_t duty) {
    // Pin activo recibe el PWM
    ledc_set_duty(MOTOR_SPEED_MODE, chActive, duty);
    ledc_update_duty(MOTOR_SPEED_MODE, chActive);
    // Pin inactivo se pone en 0 (LOW)
    ledc_set_duty(MOTOR_SPEED_MODE, chIdle, 0);
    ledc_update_duty(MOTOR_SPEED_MODE, chIdle);
}
