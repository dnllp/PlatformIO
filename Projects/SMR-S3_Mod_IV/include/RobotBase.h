/*
 * ============================================================
 * RobotBase.h — Orquestador del Robot Diferencial
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Abstrae el par de motores en comandos de movimiento intuitivos:
 *   forward(), backward(), turnLeft(), turnRight(), stop()
 * ============================================================
 */

#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include "DiffMotor.h"

class RobotBase {
public:
    /**
     * @param leftMotor  Motor izquierdo (ya construido)
     * @param rightMotor Motor derecho (ya construido)
     */
    RobotBase(DiffMotor& leftMotor, DiffMotor& rightMotor);

    /** Inicializa ambos motores. Llamar una vez en setup(). */
    void init();

    /** Avanza a la velocidad dada [0 – 8191] */
    void forward(int speed);

    /** Retrocede a la velocidad dada [0 – 8191] */
    void backward(int speed);

    /** Gira en su propio eje hacia la izquierda */
    void turnLeft(int speed);

    /** Gira en su propio eje hacia la derecha */
    void turnRight(int speed);

    /**
     * Movimiento mezclado: permite control tipo joystick.
     * @param linear   Componente lineal  [-8191, 8191]
     * @param angular  Componente angular [-8191, 8191]
     */
    void arcade(int linear, int angular);

    /** Detiene ambos motores */
    void stop();

private:
    DiffMotor& _left;
    DiffMotor& _right;
};

#endif // ROBOT_BASE_H
