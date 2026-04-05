/*
 * ============================================================
 * SlamNavigator.h — Estimación de Pose y Navegación SLAM Lite
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Nivel : 4 — Navegación autónoma (SLAM)
 *
 * Descripción:
 *   Estima la posición y orientación del robot (pose: x, y, yaw)
 *   usando odometría diferencial de los encoders (Módulo 2) e
 *   integra los scans del LiDAR en el OccupancyMap.
 *
 *   No implementa loop-closure completo (requiere procesador
 *   más potente) pero sí dead-reckoning preciso + mapeo reactivo,
 *   lo que es suficiente para navegación en espacios conocidos
 *   y exploración de entornos pequeños con el ESP32-S3.
 *
 *   Estrategia de navegación:
 *   - El robot avanza hacia el frente libre más amplio
 *   - Gira cuando el sector frontal queda bloqueado en el mapa
 *   - Se detiene si no encuentra espacio libre en ninguna dirección
 *
 * Uso básico:
 *   SlamNavigator slam(lidar, map, motorPIDLeft, motorPIDRight,
 *                      encLeft, encRight);
 *   slam.init(PPR, WHEEL_DIAM_CM, WHEELBASE_CM);
 *   // En task Core 1 cada 100 ms:
 *   slam.update(dtMs);
 * ============================================================
 */

#ifndef SLAM_NAVIGATOR_H
#define SLAM_NAVIGATOR_H

#include "LidarScanner.h"
#include "OccupancyMap.h"
#include "MotorPID.h"
#include "EncoderReader.h"
#include <math.h>

// ── Pose del robot ───────────────────────────────────────────
struct Pose {
    float x_cm;     // Posición X en cm (origen = punto de arranque)
    float y_cm;     // Posición Y en cm
    float yaw_deg;  // Orientación en grados [0-359°], 0 = norte
};

// ── Estados de navegación ────────────────────────────────────
enum class NavState : uint8_t {
    MAPPING,       // Avanza y mapea simultáneamente
    ROTATING,      // Gira para encontrar espacio libre
    STOPPED        // Sin salida detectada
};

class SlamNavigator {
public:
    /**
     * @param lidar         LidarScanner inicializado
     * @param map           OccupancyMap inicializado
     * @param motorLeft     MotorPID del motor izquierdo
     * @param motorRight    MotorPID del motor derecho
     * @param encLeft       EncoderReader izquierdo
     * @param encRight      EncoderReader derecho
     */
    SlamNavigator(LidarScanner&  lidar,
                  OccupancyMap&  map,
                  MotorPID&      motorLeft,
                  MotorPID&      motorRight,
                  EncoderReader& encLeft,
                  EncoderReader& encRight);

    /**
     * Configura los parámetros físicos del robot.
     * @param pulsesPerRev   PPR efectivo del encoder (PPR_físico × 4)
     * @param wheelDiamCm    Diámetro de la rueda en cm
     * @param wheelbaseCm    Distancia entre centros de rueda en cm
     */
    void init(uint16_t pulsesPerRev, float wheelDiamCm, float wheelbaseCm);

    /**
     * Ejecuta un ciclo SLAM: odometría → mapeo → decisión de movimiento.
     * Llamar desde un task FreeRTOS en Core 1 cada 100 ms.
     * @param dtMs  Tiempo real desde la última llamada
     */
    void update(uint32_t dtMs);

    /** Pose actual estimada por odometría */
    const Pose& getPose() const { return _pose; }

    /** Estado actual del navegador */
    NavState getNavState() const { return _navState; }
    const char* getNavStateName() const;

    /** Reinicia la pose a (0, 0, 0°) */
    void resetPose();

    /** Velocidad de crucero para navegación [0-8191] */
    void setCruiseSpeed(float pulsesPerSec) { _cruiseSpeed = pulsesPerSec; }

    /** Velocidad de giro para búsqueda [0-8191] */
    void setTurnSpeed(float pulsesPerSec)   { _turnSpeed = pulsesPerSec; }

    /**
     * Distancia frontal mínima antes de girar (en mm).
     * Se consulta del OccupancyMap, no del sensor directamente.
     */
    void setFrontClearanceMm(float mm) { _frontClearanceMm = mm; }

private:
    LidarScanner&  _lidar;
    OccupancyMap&  _map;
    MotorPID&      _motorLeft;
    MotorPID&      _motorRight;
    EncoderReader& _encLeft;
    EncoderReader& _encRight;

    Pose     _pose;
    NavState _navState;

    uint16_t _ppr;
    float    _wheelDiamCm;
    float    _wheelbaseCm;
    float    _metersPerPulse;

    float    _cruiseSpeed;
    float    _turnSpeed;
    float    _frontClearanceMm;

    int32_t  _lastPulsesLeft;
    int32_t  _lastPulsesRight;

    uint32_t _stateTimer;

    // ── Módulos internos ────────────────────────────────────
    // Actualiza la pose con odometría diferencial
    void _updateOdometry();

    // Integra el scan actual en el mapa
    void _updateMap();

    // Evalúa el mapa y decide la acción de movimiento
    void _navigate();

    // Distancia libre hacia adelante en el mapa (mm)
    float _frontFreeDistanceMm() const;

    // Ángulo de giro con más espacio libre (grados relativos)
    float _bestTurnAngle() const;

    static float _normalizeAngle(float deg);
};

#endif // SLAM_NAVIGATOR_H
