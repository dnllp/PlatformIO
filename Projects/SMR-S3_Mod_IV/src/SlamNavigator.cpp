/*
 * ============================================================
 * SlamNavigator.cpp — Implementación SLAM Lite
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 */

#include "SlamNavigator.h"
#include "esp_log.h"
#include "Arduino.h"
#include <string.h>

static const char* TAG = "SlamNavigator";

// ── Constructor ──────────────────────────────────────────────
SlamNavigator::SlamNavigator(LidarScanner&  lidar,
                             OccupancyMap&  map,
                             MotorPID&      motorLeft,
                             MotorPID&      motorRight,
                             EncoderReader& encLeft,
                             EncoderReader& encRight)
    : _lidar(lidar), _map(map),
      _motorLeft(motorLeft), _motorRight(motorRight),
      _encLeft(encLeft), _encRight(encRight),
      _pose{0.0f, 0.0f, 0.0f},
      _navState(NavState::MAPPING),
      _ppr(80), _wheelDiamCm(6.5f), _wheelbaseCm(15.0f),
      _metersPerPulse(0.0f),
      _cruiseSpeed(300.0f), _turnSpeed(200.0f),
      _frontClearanceMm(300.0f),
      _lastPulsesLeft(0), _lastPulsesRight(0),
      _stateTimer(0)
{}

// ── init() ───────────────────────────────────────────────────
void SlamNavigator::init(uint16_t pulsesPerRev,
                         float    wheelDiamCm,
                         float    wheelbaseCm) {
    _ppr          = pulsesPerRev;
    _wheelDiamCm  = wheelDiamCm;
    _wheelbaseCm  = wheelbaseCm;

    // Distancia por pulso en cm
    _metersPerPulse = (float)M_PI * _wheelDiamCm / _ppr;

    resetPose();

    ESP_LOGI(TAG, "SLAM init: PPR=%d diam=%.1fcm base=%.1fcm dist/pulso=%.4fcm",
             _ppr, _wheelDiamCm, _wheelbaseCm, _metersPerPulse);
}

// ── resetPose() ──────────────────────────────────────────────
void SlamNavigator::resetPose() {
    _pose = { 0.0f, 0.0f, 0.0f };
    _lastPulsesLeft  = _encLeft.getCount();
    _lastPulsesRight = _encRight.getCount();
    _navState   = NavState::MAPPING;
    _stateTimer = 0;
}

// ── update() ─────────────────────────────────────────────────
void SlamNavigator::update(uint32_t dtMs) {
    _stateTimer += dtMs;

    // 1. Actualizar pose con odometría
    _updateOdometry();

    // 2. Integrar scan LiDAR en el mapa (solo si el LiDAR está listo)
    if (_lidar.isReady()) {
        _updateMap();
    }

    // 3. Tomar decisión de navegación
    _navigate();
}

// ── getNavStateName() ────────────────────────────────────────
const char* SlamNavigator::getNavStateName() const {
    switch (_navState) {
        case NavState::MAPPING:  return "MAPPING";
        case NavState::ROTATING: return "ROTATING";
        case NavState::STOPPED:  return "STOPPED";
        default:                 return "UNKNOWN";
    }
}

// ── _updateOdometry() ────────────────────────────────────────
void SlamNavigator::_updateOdometry() {
    int32_t curL = _encLeft.getCount();
    int32_t curR = _encRight.getCount();

    int32_t dL = curL - _lastPulsesLeft;
    int32_t dR = curR - _lastPulsesRight;

    _lastPulsesLeft  = curL;
    _lastPulsesRight = curR;

    // Distancias recorridas por cada rueda en cm
    float distL = static_cast<float>(dL) * _metersPerPulse;
    float distR = static_cast<float>(dR) * _metersPerPulse;

    // Modelo diferencial:
    // distancia lineal = promedio de ambas ruedas
    // rotación = diferencia / separación entre ruedas
    float distCenter = (distL + distR) / 2.0f;
    float deltaYaw   = (distR - distL) / _wheelbaseCm;  // radianes

    float yawRad = _pose.yaw_deg * (float)M_PI / 180.0f;

    _pose.x_cm    += distCenter * cosf(yawRad + deltaYaw / 2.0f);
    _pose.y_cm    += distCenter * sinf(yawRad + deltaYaw / 2.0f);
    _pose.yaw_deg  = _normalizeAngle(_pose.yaw_deg + deltaYaw * 180.0f / (float)M_PI);
}

// ── _updateMap() ─────────────────────────────────────────────
void SlamNavigator::_updateMap() {
    // Obtener copia del escaneo actual
    ScanPoint scanBuf[LIDAR_BUCKET_COUNT];
    _lidar.getScanCopy(scanBuf);

    // Extraer solo las distancias (mm) para updateFullScan
    float distArr[360] = {};
    for (uint16_t i = 0; i < 360; i++) {
        distArr[i] = scanBuf[i].distance_mm;
    }

    _map.updateFullScan(_pose.x_cm, _pose.y_cm, _pose.yaw_deg, distArr);
}

// ── _navigate() ──────────────────────────────────────────────
void SlamNavigator::_navigate() {
    switch (_navState) {

        case NavState::MAPPING: {
            float frontDist = _frontFreeDistanceMm();

            if (frontDist > 0.0f && frontDist < _frontClearanceMm) {
                // Obstáculo detectado en el mapa → rotar
                _motorLeft.stop();
                _motorRight.stop();
                _navState   = NavState::ROTATING;
                _stateTimer = 0;
                ESP_LOGI(TAG, "MAPPING → ROTATING (frente=%.0f mm)", frontDist);
            } else {
                // Camino libre → avanzar con PID
                _motorLeft.setTargetSpeed(_cruiseSpeed);
                _motorRight.setTargetSpeed(_cruiseSpeed);
            }
            break;
        }

        case NavState::ROTATING: {
            // Girar hacia el mejor ángulo libre
            float bestAngle = _bestTurnAngle();

            if (bestAngle > 0.0f) {
                // Espacio libre a la izquierda
                _motorLeft.setTargetSpeed(-_turnSpeed);
                _motorRight.setTargetSpeed(_turnSpeed);
            } else if (bestAngle < 0.0f) {
                // Espacio libre a la derecha
                _motorLeft.setTargetSpeed(_turnSpeed);
                _motorRight.setTargetSpeed(-_turnSpeed);
            } else {
                // Sin salida encontrada
                _motorLeft.stop();
                _motorRight.stop();
                _navState = NavState::STOPPED;
                ESP_LOGW(TAG, "ROTATING → STOPPED (sin salida)");
                break;
            }

            // Salir de rotación si el frente ya está despejado
            float frontDist = _frontFreeDistanceMm();
            if (frontDist < 0.0f || frontDist > _frontClearanceMm) {
                _navState   = NavState::MAPPING;
                _stateTimer = 0;
                ESP_LOGI(TAG, "ROTATING → MAPPING");
            }

            // Timeout de seguridad: si lleva >3s girando, re-evaluar
            if (_stateTimer > 3000) {
                _navState   = NavState::MAPPING;
                _stateTimer = 0;
            }
            break;
        }

        case NavState::STOPPED:
            // Detenido: esperar intervención externa
            _motorLeft.stop();
            _motorRight.stop();
            break;
    }
}

// ── _frontFreeDistanceMm() ───────────────────────────────────
float SlamNavigator::_frontFreeDistanceMm() const {
    // Sector frontal: ±20° del yaw actual en el LiDAR
    float startAngle = _normalizeAngle(_pose.yaw_deg - 20.0f);
    float endAngle   = _normalizeAngle(_pose.yaw_deg + 20.0f);
    return _lidar.getMinInSector(startAngle, endAngle);
}

// ── _bestTurnAngle() ─────────────────────────────────────────
float SlamNavigator::_bestTurnAngle() const {
    // Comparar espacio libre a la izquierda (yaw+90°) vs derecha (yaw-90°)
    float leftAngleS  = _normalizeAngle(_pose.yaw_deg + 60.0f);
    float leftAngleE  = _normalizeAngle(_pose.yaw_deg + 120.0f);
    float rightAngleS = _normalizeAngle(_pose.yaw_deg - 120.0f);
    float rightAngleE = _normalizeAngle(_pose.yaw_deg - 60.0f);

    float distLeft  = _lidar.getMinInSector(leftAngleS,  leftAngleE);
    float distRight = _lidar.getMinInSector(rightAngleS, rightAngleE);

    bool leftOk  = (distLeft  < 0.0f || distLeft  > _frontClearanceMm);
    bool rightOk = (distRight < 0.0f || distRight > _frontClearanceMm);

    if (!leftOk && !rightOk) return 0.0f;   // Sin salida

    // Retornar >0 para girar izquierda, <0 para girar derecha
    if (leftOk && rightOk) {
        // Ambos libres: elegir el de mayor distancia
        float dL = (distLeft  < 0.0f) ? 9999.0f : distLeft;
        float dR = (distRight < 0.0f) ? 9999.0f : distRight;
        return (dL >= dR) ? 1.0f : -1.0f;
    }
    return leftOk ? 1.0f : -1.0f;
}

// ── _normalizeAngle() ────────────────────────────────────────
float SlamNavigator::_normalizeAngle(float deg) {
    while (deg < 0.0f)    deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}
