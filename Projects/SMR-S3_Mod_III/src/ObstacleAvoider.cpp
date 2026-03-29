/*
 * ============================================================
 * ObstacleAvoider.cpp — Implementación de la Evasión Reactiva
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 */

#include "ObstacleAvoider.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ObstacleAvoider";

// ── Constructor ──────────────────────────────────────────────
ObstacleAvoider::ObstacleAvoider(RobotBase&        robot,
                                 UltrasonicSensor* sonarLeft,
                                 UltrasonicSensor& sonarFront,
                                 UltrasonicSensor* sonarRight)
    : _robot(robot),
      _sonarLeft(sonarLeft),
      _sonarFront(sonarFront),
      _sonarRight(sonarRight),
      _state(AvoidState::FORWARD),
      _cruiseSpeed(3500),
      _turnSpeed(2500),
      _stopCm(15.0f),
      _warnCm(25.0f),
      _sideBlockCm(20.0f),
      _distLeft(-1.0f),
      _distFront(-1.0f),
      _distRight(-1.0f),
      _stateTimer(0)
{}

// ── setThresholds() ──────────────────────────────────────────
void ObstacleAvoider::setThresholds(float stopCm, float warnCm, float sideBlockCm) {
    _stopCm      = stopCm;
    _warnCm      = warnCm;
    _sideBlockCm = sideBlockCm;
}

// ── update() — núcleo de la máquina de estados ───────────────
void ObstacleAvoider::update() {
    _measureAll();
    _stateTimer += 50;  // Asume período de llamada de ~50 ms

    switch (_state) {

        // ── FORWARD: avanzar libremente ──────────────────────
        case AvoidState::FORWARD: {
            if (_distFront > 0.0f && _distFront < _stopCm) {
                // Obstáculo frontal detectado → frenar
                _robot.stop();
                _state      = AvoidState::STOP;
                _stateTimer = 0;
                ESP_LOGI(TAG, "FORWARD → STOP  (frente=%.1f cm)", _distFront);
            } else if (_distFront > 0.0f && _distFront < _warnCm) {
                // Zona de advertencia → reducir velocidad al 40%
                int slowSpeed = (_cruiseSpeed * 40) / 100;
                _robot.forward(slowSpeed);
            } else {
                _robot.forward(_cruiseSpeed);
            }
            break;
        }

        // ── STOP: pausa breve antes de escanear ──────────────
        case AvoidState::STOP: {
            if (_stateTimer >= 300) {  // 300 ms parado
                _state      = AvoidState::SCAN;
                _stateTimer = 0;
                ESP_LOGI(TAG, "STOP → SCAN");
            }
            break;
        }

        // ── SCAN: medir laterales y decidir dirección ─────────
        case AvoidState::SCAN: {
            AvoidState next = _decideTurn();
            _state          = next;
            _stateTimer     = 0;
            ESP_LOGI(TAG, "SCAN → %s  (izq=%.1f, der=%.1f)",
                     getStateName(), _distLeft, _distRight);
            break;
        }

        // ── TURN_LEFT: girar hasta que el frente quede libre ──
        case AvoidState::TURN_LEFT: {
            _robot.turnLeft(_turnSpeed);
            // Salir del giro cuando el frente esté despejado
            if (_distFront < 0.0f || _distFront > _warnCm) {
                _state      = AvoidState::FORWARD;
                _stateTimer = 0;
                ESP_LOGI(TAG, "TURN_LEFT → FORWARD");
            }
            // Si giró demasiado tiempo sin salida → re-escanear
            if (_stateTimer > 2000) {
                _robot.stop();
                _state      = AvoidState::SCAN;
                _stateTimer = 0;
            }
            break;
        }

        // ── TURN_RIGHT: girar hasta que el frente quede libre ─
        case AvoidState::TURN_RIGHT: {
            _robot.turnRight(_turnSpeed);
            if (_distFront < 0.0f || _distFront > _warnCm) {
                _state      = AvoidState::FORWARD;
                _stateTimer = 0;
                ESP_LOGI(TAG, "TURN_RIGHT → FORWARD");
            }
            if (_stateTimer > 2000) {
                _robot.stop();
                _state      = AvoidState::SCAN;
                _stateTimer = 0;
            }
            break;
        }

        // ── BACKWARD: retroceder y luego girar ───────────────
        case AvoidState::BACKWARD: {
            _robot.backward(_turnSpeed);
            if (_stateTimer >= 800) {  // 800 ms hacia atrás
                _robot.stop();
                _state      = AvoidState::SCAN;
                _stateTimer = 0;
                ESP_LOGI(TAG, "BACKWARD → SCAN");
            }
            break;
        }
    }
}

// ── getStateName() ───────────────────────────────────────────
const char* ObstacleAvoider::getStateName() const {
    switch (_state) {
        case AvoidState::FORWARD:    return "FORWARD";
        case AvoidState::STOP:       return "STOP";
        case AvoidState::SCAN:       return "SCAN";
        case AvoidState::TURN_LEFT:  return "TURN_LEFT";
        case AvoidState::TURN_RIGHT: return "TURN_RIGHT";
        case AvoidState::BACKWARD:   return "BACKWARD";
        default:                     return "UNKNOWN";
    }
}

// ── _measureAll() (privado) ──────────────────────────────────
void ObstacleAvoider::_measureAll() {
    // Medir con delays entre sensores para evitar interferencia acústica
    if (_sonarLeft)  { _distLeft  = _sonarLeft->getDistanceCm();  vTaskDelay(pdMS_TO_TICKS(10)); }
    _distFront = _sonarFront.getDistanceCm();
    if (_sonarRight) { vTaskDelay(pdMS_TO_TICKS(10)); _distRight = _sonarRight->getDistanceCm(); }
}

// ── _decideTurn() (privado) ──────────────────────────────────
    AvoidState ObstacleAvoider::_decideTurn() const {
    bool leftBlocked  = (_sonarLeft  && _distLeft  > 0.0f && _distLeft  < _sideBlockCm);
    bool rightBlocked = (_sonarRight && _distRight > 0.0f && _distRight < _sideBlockCm);

    // Ambos lados bloqueados → retroceder
    if (leftBlocked && rightBlocked) return AvoidState::BACKWARD;

    // Solo un lado bloqueado → girar al lado libre
    if (leftBlocked)  return AvoidState::TURN_RIGHT;
    if (rightBlocked) return AvoidState::TURN_LEFT;

    // Ninguno bloqueado → elegir el lado con más espacio
    // Si no hay sensores laterales, girar a la derecha por defecto
    float dL = (_sonarLeft  && _distLeft  > 0.0f) ? _distLeft  : 999.0f;
    float dR = (_sonarRight && _distRight > 0.0f) ? _distRight : 999.0f;
    return (dL >= dR) ? AvoidState::TURN_LEFT : AvoidState::TURN_RIGHT;
}
