/*
 * ============================================================
 * RobotBase.cpp — Implementación del Orquestador
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 */

#include "RobotBase.h"
#include <algorithm>  // std::clamp
#include <iostream>
#include <cstdint>

static constexpr int MAX_SPEED = 8191;

RobotBase::RobotBase(DiffMotor& leftMotor, DiffMotor& rightMotor)
    : _left(leftMotor), _right(rightMotor)
{}

void RobotBase::init() {
    _left.init();
    _right.init();
}

void RobotBase::forward(int speed) {
    speed = std::clamp(speed, 0, MAX_SPEED);
    _left.drive(speed);
    _right.drive(speed);
}

void RobotBase::backward(int speed) {
    speed = std::clamp(speed, 0, MAX_SPEED);
    _left.drive(-speed);
    _right.drive(-speed);
}

void RobotBase::turnLeft(int speed) {
    speed = std::clamp(speed, 0, MAX_SPEED);
    _left.drive(-speed);   // Rueda izq. hacia atrás
    _right.drive(speed);   // Rueda der. hacia adelante
}

void RobotBase::turnRight(int speed) {
    speed = std::clamp(speed, 0, MAX_SPEED);
    _left.drive(speed);
    _right.drive(-speed);
}

void RobotBase::arcade(int linear, int angular) {
    // Mezcla tipo arcade: izq = linear + angular, der = linear - angular
    int leftSpeed  = std::clamp(linear + angular, -MAX_SPEED, MAX_SPEED);
    int rightSpeed = std::clamp(linear - angular, -MAX_SPEED, MAX_SPEED);
    _left.drive(leftSpeed);
    _right.drive(rightSpeed);
}

void RobotBase::stop() {
    _left.stop();
    _right.stop();
}
