/*
 * ============================================================
 * ObstacleAvoider.h — Módulo de Evasión Reactiva de Obstáculos
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Nivel      : 3 — Percepción reactiva del entorno
 *
 * Descripción:
 *   Implementa comportamiento de evasión reactiva usando hasta
 *   3 sensores ultrasónicos (izquierdo, frontal, derecho).
 *   Toma decisiones de movimiento en función de las zonas
 *   de detección y una máquina de estados simple.
 *
 * Máquina de estados:
 *
 *   FORWARD ──(frente libre)──────────────────────────────┐
 *      │                                                  │
 *      └──(obstáculo frente)──► STOP ──► SCAN ──► decide ┘
 *                                          │
 *                              TURN_LEFT ◄─┤─► TURN_RIGHT
 *                                          │
 *                                       BACKWARD (si ambos lados bloqueados)
 *
 * Uso básico:
 *   ObstacleAvoider avoider(robot, sonarLeft, sonarFront, sonarRight);
 *   avoider.setThresholds(15.0f, 25.0f, 30.0f);
 *   // En un task FreeRTOS cada 50 ms:
 *   avoider.update();
 * ============================================================
 */

#ifndef OBSTACLE_AVOIDER_H
#define OBSTACLE_AVOIDER_H

#include "UltrasonicSensor.h"
#include "RobotBase.h"

// ── Estados de la máquina de evasión ─────────────────────────
enum class AvoidState : uint8_t {
    FORWARD,       // Avanza libremente
    STOP,          // Frena al detectar obstáculo frontal
    SCAN,          // Mide los tres sensores para decidir
    TURN_LEFT,     // Gira a la izquierda
    TURN_RIGHT,    // Gira a la derecha
    BACKWARD       // Retrocede (ambos lados bloqueados)
};

class ObstacleAvoider {
public:
    /**
     * Constructor
     * @param robot        Referencia al RobotBase (Módulo 1)
     * @param sonarLeft    Sensor izquierdo (puede ser nullptr si no existe)
     * @param sonarFront   Sensor frontal (obligatorio)
     * @param sonarRight   Sensor derecho (puede ser nullptr si no existe)
     */
    ObstacleAvoider(RobotBase&        robot,
                    UltrasonicSensor* sonarLeft,
                    UltrasonicSensor& sonarFront,
                    UltrasonicSensor* sonarRight);

    /**
     * Configura las distancias de reacción.
     * @param stopCm    Frena si obstáculo frontal < stopCm (ej. 15 cm)
     * @param warnCm    Reduce velocidad si < warnCm    (ej. 25 cm)
     * @param sideBlockCm  Lateral bloqueado si < sideBlockCm (ej. 20 cm)
     */
    void setThresholds(float stopCm, float warnCm, float sideBlockCm);

    /**
     * Velocidad de crucero al avanzar libremente [0-8191].
     */
    void setCruiseSpeed(int speed) { _cruiseSpeed = speed; }

    /**
     * Velocidad de giro al evadir [0-8191].
     */
    void setTurnSpeed(int speed) { _turnSpeed = speed; }

    /**
     * Ejecuta un ciclo de la máquina de evasión.
     * Llamar periódicamente desde un task FreeRTOS (ej. cada 50 ms).
     */
    void update();

    /** Estado actual de la máquina */
    AvoidState getState() const { return _state; }

    /** Nombre del estado actual como cadena (para debug) */
    const char* getStateName() const;

    /** Últimas distancias medidas (cm), -1 si no disponible */
    float getDistLeft()  const { return _distLeft;  }
    float getDistFront() const { return _distFront; }
    float getDistRight() const { return _distRight; }

private:
    RobotBase&        _robot;
    UltrasonicSensor* _sonarLeft;
    UltrasonicSensor& _sonarFront;
    UltrasonicSensor* _sonarRight;

    AvoidState _state;
    int        _cruiseSpeed;
    int        _turnSpeed;

    float _stopCm;
    float _warnCm;
    float _sideBlockCm;

    float _distLeft;
    float _distFront;
    float _distRight;

    uint32_t _stateTimer;  // ms en el estado actual (anti-chatter)

    // Lee los tres sensores disponibles
    void _measureAll();

    // Decide a qué lado girar según las distancias laterales
    AvoidState _decideTurn() const;
};

#endif // OBSTACLE_AVOIDER_H
