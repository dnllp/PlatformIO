/*
 * ============================================================
 * PIDController.cpp — Implementación del Controlador PID
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 */

#include "PIDController.h"
#include <cmath>   // isnan, isinf

// ── Constructor ──────────────────────────────────────────────
PIDController::PIDController(float kp, float ki, float kd,
                             float outMin, float outMax)
    : _kp(kp), _ki(ki), _kd(kd),
      _outMin(outMin), _outMax(outMax),
      _setpoint(0.0f), _integral(0.0f),
      _lastMeasurement(0.0f), _lastError(0.0f)
{}

// ── setSetpoint() ────────────────────────────────────────────
void PIDController::setSetpoint(float setpoint) {
    _setpoint = setpoint;
    // Reiniciar integral al cambiar el objetivo evita arranques bruscos
    _integral = 0.0f;
}

// ── compute() ────────────────────────────────────────────────
float PIDController::compute(float measurement, uint32_t dtMs) {
    if (dtMs == 0) return 0.0f;

    float dt = static_cast<float>(dtMs) / 1000.0f;  // a segundos

    // ── Término Proporcional ─────────────────────────────────
    float error = _setpoint - measurement;
    float termP = _kp * error;

    // ── Término Integral con Anti-Windup ─────────────────────
    _integral += _ki * error * dt;
    _integral  = _clamp(_integral, _outMin, _outMax);  // Anti-windup

    // ── Término Derivativo (sobre la medición, no el error) ──
    // Evita "derivative kick" cuando el setpoint cambia bruscamente
    float dMeasurement = (measurement - _lastMeasurement) / dt;
    float termD        = -_kd * dMeasurement;

    // ── Salida total acotada ─────────────────────────────────
    float output = _clamp(termP + _integral + termD, _outMin, _outMax);

    // ── Guardar estado para el siguiente ciclo ───────────────
    _lastMeasurement = measurement;
    _lastError       = error;

    return output;
}

// ── reset() ──────────────────────────────────────────────────
void PIDController::reset() {
    _integral        = 0.0f;
    _lastMeasurement = 0.0f;
    _lastError       = 0.0f;
}

// ── _clamp() (privado) ───────────────────────────────────────
float PIDController::_clamp(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}
