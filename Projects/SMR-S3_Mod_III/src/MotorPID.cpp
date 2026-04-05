/*
 * ============================================================
 * MotorPID.cpp — Implementación del Motor con PID de Velocidad
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 */

#include "MotorPID.h"

// ── Constructor ──────────────────────────────────────────────
MotorPID::MotorPID(DiffMotor& motor, EncoderReader& encoder, PIDController& pid)
    : _motor(motor), _encoder(encoder), _pid(pid),
      _measuredSpeed(0.0f), _pidOutput(0.0f)
{}

// ── setTargetSpeed() ─────────────────────────────────────────
void MotorPID::setTargetSpeed(float pulsesPerSec) {
    _pid.setSetpoint(pulsesPerSec);
}

// ── update() ─────────────────────────────────────────────────
void MotorPID::update(uint32_t dtMs) {
    // 1. Leer velocidad actual desde el encoder
    _measuredSpeed = _encoder.getSpeed(dtMs);

    // 2. Calcular corrección PID
    _pidOutput = _pid.compute(_measuredSpeed, dtMs);

    // 3. Aplicar la salida al motor
    _motor.drive(static_cast<int>(_pidOutput));
}

// ── stop() ───────────────────────────────────────────────────
void MotorPID::stop() {
    _pid.setSetpoint(0.0f);
    _pid.reset();
    _motor.stop();
    _measuredSpeed = 0.0f;
    _pidOutput     = 0.0f;
}
