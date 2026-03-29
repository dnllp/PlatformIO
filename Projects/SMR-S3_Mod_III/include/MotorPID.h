/*
 * ============================================================
 * MotorPID.h — Motor con Control PID de Velocidad
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Nivel      : 2 — Closed Loop
 *
 * Descripción:
 *   Une DiffMotor + EncoderReader + PIDController en una sola
 *   clase. El método update() debe llamarse periódicamente
 *   (desde un task FreeRTOS) para cerrar el lazo de control.
 *
 *   El setpoint se expresa en pulsos/segundo. Para convertirlo
 *   a una velocidad física usar EncoderReader::pulsesToCm().
 *
 * Uso básico:
 *   MotorPID motorIzq(diffMotor, encoder, pid);
 *   motorIzq.setTargetSpeed(400.0f);   // 400 pulsos/segundo
 *   // Cada 20 ms desde un task:
 *   motorIzq.update(20);
 * ============================================================
 */

#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include "DiffMotor.h"
#include "EncoderReader.h"
#include "PIDController.h"

class MotorPID {
public:
    /**
     * @param motor    Referencia al DiffMotor ya inicializado
     * @param encoder  Referencia al EncoderReader ya inicializado
     * @param pid      Referencia al PIDController configurado
     */
    MotorPID(DiffMotor& motor, EncoderReader& encoder, PIDController& pid);

    /**
     * Establece la velocidad objetivo en pulsos/segundo.
     * Positivo = avance, Negativo = retroceso, 0 = freno.
     */
    void setTargetSpeed(float pulsesPerSec);

    /**
     * Ejecuta un ciclo del lazo de control.
     * Debe llamarse periódicamente con el intervalo real en ms.
     * @param dtMs  Tiempo desde la última llamada (milisegundos)
     */
    void update(uint32_t dtMs);

    /**
     * Detiene el motor y reinicia el PID.
     */
    void stop();

    /** Velocidad medida actual (pulsos/segundo) */
    float getMeasuredSpeed() const { return _measuredSpeed; }

    /** Salida PWM actual calculada por el PID [-8191, 8191] */
    float getPIDOutput()     const { return _pidOutput; }

private:
    DiffMotor&     _motor;
    EncoderReader& _encoder;
    PIDController& _pid;

    float _measuredSpeed;
    float _pidOutput;
};

#endif // MOTOR_PID_H
