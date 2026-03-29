/*
 * ============================================================
 * PIDController.h — Controlador PID Genérico
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Plataforma : ESP32-S3 (cualquier plataforma C++)
 * Nivel      : 2 — Closed Loop
 *
 * Descripción:
 *   Implementa un controlador PID discreto con las siguientes
 *   características de producción:
 *     - Anti-windup: limita la integral al rango de salida
 *     - Derivative-on-measurement: evita el "derivative kick"
 *       al cambiar el setpoint bruscamente
 *     - Salida acotada: clamp configurable [outMin, outMax]
 *     - Todos los parámetros ajustables en tiempo de ejecución
 *
 * Ecuación discreta implementada:
 *   error(k)    = setpoint - medición
 *   P           = Kp × error(k)
 *   I           = I_prev + Ki × error(k) × dt   [con anti-windup]
 *   D           = -Kd × (medición - medición_prev) / dt
 *   salida      = clamp(P + I + D, outMin, outMax)
 *
 * Uso básico:
 *   PIDController pid(2.0f, 0.5f, 0.1f, -8191.0f, 8191.0f);
 *   pid.setSetpoint(300.0f);           // 300 pulsos/segundo
 *   float pwm = pid.compute(velocidad, dtMs);
 * ============================================================
 */
#include <stdint.h>
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    /**
     * Constructor
     * @param kp      Ganancia proporcional
     * @param ki      Ganancia integral
     * @param kd      Ganancia derivativa
     * @param outMin  Límite inferior de la salida (ej. -8191)
     * @param outMax  Límite superior de la salida (ej.  8191)
     */
    PIDController(float kp, float ki, float kd,
                  float outMin, float outMax);

    /**
     * Establece el valor deseado (setpoint).
     * Al cambiar el setpoint se reinicia la integral para evitar
     * transientes bruscos.
     */
    void setSetpoint(float setpoint);

    /**
     * Calcula la salida del PID.
     * @param measurement  Valor medido actual (ej. velocidad en pulsos/s)
     * @param dtMs         Tiempo transcurrido desde el último compute (ms)
     * @return             Salida acotada en [outMin, outMax]
     */
    float compute(float measurement, uint32_t dtMs);

    /**
     * Reinicia el estado interno del PID (integral y derivada).
     * Usar al reanudar el control después de una parada.
     */
    void reset();

    // ── Getters de diagnóstico ───────────────────────────────
    float getSetpoint()   const { return _setpoint; }
    float getLastError()  const { return _lastError; }
    float getIntegral()   const { return _integral; }

    // ── Sintonización en caliente ────────────────────────────
    void setKp(float kp) { _kp = kp; }
    void setKi(float ki) { _ki = ki; }
    void setKd(float kd) { _kd = kd; }

private:
    float _kp, _ki, _kd;
    float _outMin, _outMax;
    float _setpoint;
    float _integral;
    float _lastMeasurement;  // Para derivative-on-measurement
    float _lastError;

    float _clamp(float value, float minVal, float maxVal);
};

#endif // PID_CONTROLLER_H
