/*
 * ============================================================
 * main4.cpp — Integración Completa (Módulos 1+2+3+4)
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Arquitectura FreeRTOS Dual-Core:
 *
 *   CORE 0 (tiempo real):
 *     taskLidar    → Parser YDLidar X3, máxima frecuencia
 *     taskPID      → Lazo PID de velocidad, 50 Hz (20 ms)
 *
 *   CORE 1 (navegación):
 *     taskSLAM     → Odometría + mapeo + decisión, 10 Hz (100 ms)
 *     taskTelemetry→ Telemetría Serial + mapa ASCII, 1 Hz (1000 ms)
 *
 * Ajustar pines GPIO y parámetros físicos antes de compilar.
 * ============================================================
 */

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ── Módulo 1: Motores ────────────────────────────────────────
#include "DiffMotor.h"
#include "RobotBase.h"

// ── Módulo 2: Encoders + PID ─────────────────────────────────
#include "EncoderReader.h"
#include "PIDController.h"
#include "MotorPID.h"

// ── Módulo 3: Ultrasónicos (seguridad adicional) ─────────────
#include "UltrasonicSensor.h"
#include "ObstacleAvoider.h"

// ── Módulo 4: LiDAR + SLAM ───────────────────────────────────
#include "LidarScanner.h"
#include "OccupancyMap.h"
#include "SlamNavigator.h"

// ════════════════════════════════════════════════════════════
//  CONFIGURACIÓN DE HARDWARE — ajustar según su robot
// ════════════════════════════════════════════════════════════

// ── Motores L298N ────────────────────────────────────────────
static constexpr gpio_num_t LEFT_IN1  = GPIO_NUM_5;
static constexpr gpio_num_t LEFT_IN2  = GPIO_NUM_6;
static constexpr gpio_num_t RIGHT_IN1 = GPIO_NUM_7;
static constexpr gpio_num_t RIGHT_IN2 = GPIO_NUM_15;

// ── Encoders de cuadratura ───────────────────────────────────
static constexpr gpio_num_t LEFT_ENC_A  = GPIO_NUM_36;
static constexpr gpio_num_t LEFT_ENC_B  = GPIO_NUM_39;
static constexpr gpio_num_t RIGHT_ENC_A = GPIO_NUM_34;
static constexpr gpio_num_t RIGHT_ENC_B = GPIO_NUM_35;
static constexpr uint16_t   PPR         = 80;      // PPR_físico × 4
static constexpr float      WHEEL_DIAM  = 6.5f;    // cm
static constexpr float      WHEELBASE   = 15.0f;   // cm (distancia entre ruedas)

// ── PID ──────────────────────────────────────────────────────
static constexpr float KP = 3.0f;
static constexpr float KI = 0.8f;
static constexpr float KD = 0.05f;

// ── Sensor ultrasónico frontal (seguridad de emergencia) ─────
static constexpr gpio_num_t TRIG_FRONT = GPIO_NUM_18;
static constexpr gpio_num_t ECHO_FRONT = GPIO_NUM_19;
static constexpr float      EMERGENCY_STOP_CM = 10.0f;  // Parada de emergencia

// ── YDLidar X3 ───────────────────────────────────────────────
static constexpr gpio_num_t LIDAR_RX    = GPIO_NUM_16;  // ESP32 RX ← X3 TX
static constexpr gpio_num_t LIDAR_TX    = GPIO_NUM_17;  // ESP32 TX → X3 RX
static constexpr gpio_num_t LIDAR_MOTOR = GPIO_NUM_4;   // DTR/MOTOCTRL

// ── Velocidades SLAM ─────────────────────────────────────────
static constexpr float SLAM_CRUISE_SPEED = 280.0f;  // pulsos/s
static constexpr float SLAM_TURN_SPEED   = 200.0f;  // pulsos/s
static constexpr float SLAM_FRONT_CLEAR  = 350.0f;  // mm antes de girar

// ── Períodos de tasks ────────────────────────────────────────
static constexpr uint32_t PID_PERIOD_MS  = 20;
static constexpr uint32_t SLAM_PERIOD_MS = 100;

// ════════════════════════════════════════════════════════════
//  INSTANCIAS GLOBALES
// ════════════════════════════════════════════════════════════

// Módulo 1
DiffMotor motorLeft (LEFT_IN1, LEFT_IN2, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
DiffMotor motorRight(RIGHT_IN1, RIGHT_IN2, LEDC_CHANNEL_2, LEDC_CHANNEL_3);
RobotBase robot(motorLeft, motorRight);

// Módulo 2
EncoderReader encLeft (LEFT_ENC_A,  LEFT_ENC_B,  PCNT_UNIT_0);
EncoderReader encRight(RIGHT_ENC_A, RIGHT_ENC_B, PCNT_UNIT_1);
PIDController pidLeft (KP, KI, KD, -8191.0f, 8191.0f);
PIDController pidRight(KP, KI, KD, -8191.0f, 8191.0f);
MotorPID motorPIDLeft (motorLeft,  encLeft,  pidLeft);
MotorPID motorPIDRight(motorRight, encRight, pidRight);

// Módulo 3 (solo frontal como parada de emergencia)
UltrasonicSensor sonarFront(TRIG_FRONT, ECHO_FRONT);

// Módulo 4
LidarScanner  lidar(Serial2, LIDAR_MOTOR);
OccupancyMap  occMap;
SlamNavigator slam(lidar, occMap, motorPIDLeft, motorPIDRight,
                   encLeft, encRight);

// ════════════════════════════════════════════════════════════
//  TASKS FREERTOS
// ════════════════════════════════════════════════════════════

// ── Core 0: Parser LiDAR (máxima prioridad, sin período fijo) ─
void taskLidar(void* pv) {
    while (true) {
        lidar.update();
        // Ceder CPU brevemente para que el task PID pueda ejecutarse
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ── Core 0: Lazo PID de velocidad (50 Hz) ───────────────────
void taskPID(void* pv) {
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
        motorPIDLeft.update(PID_PERIOD_MS);
        motorPIDRight.update(PID_PERIOD_MS);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PID_PERIOD_MS));
    }
}

// ── Core 1: SLAM (odometría + mapa + navegación, 10 Hz) ──────
void taskSLAM(void* pv) {
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
        // Parada de emergencia por ultrasónico (hardware backup)
        float sonarDist = sonarFront.getDistanceCm();
        if (sonarDist > 0.0f && sonarDist < EMERGENCY_STOP_CM) {
            motorPIDLeft.stop();
            motorPIDRight.stop();
            ESP_LOGW("SLAM_TASK", "PARADA EMERGENCIA sonar=%.1f cm", sonarDist);
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SLAM_PERIOD_MS));
            continue;
        }

        slam.update(SLAM_PERIOD_MS);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SLAM_PERIOD_MS));
    }
}

// ── Core 1: Telemetría (1 Hz) ────────────────────────────────
void taskTelemetry(void* pv) {
    while (true) {
        const Pose& p = slam.getPose();
        Serial.printf(
            "[SMR-S3] Pose: x=%.1f cm  y=%.1f cm  yaw=%.1f°  "
            "Estado: %s  Puntos LiDAR: %u\n",
            p.x_cm, p.y_cm, p.yaw_deg,
            slam.getNavStateName(),
            lidar.getValidPointCount()
        );
        Serial.printf(
            "         Vel L: %.0f p/s  Vel R: %.0f p/s  "
            "Sonar: %.1f cm\n",
            motorPIDLeft.getMeasuredSpeed(),
            motorPIDRight.getMeasuredSpeed(),
            sonarFront.getLastDistance()
        );

        // Mapa ASCII reducido cada 5 segundos
        static uint8_t mapTick = 0;
        if (++mapTick >= 5) {
            mapTick = 0;
            Serial.println("── Mapa de ocupación (escala 1:20) ──");
            occMap.printASCII(20);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial.println("[SMR-S3] Módulo 4 — LiDAR + SLAM arrancando...");

    // ── Módulo 1 ─────────────────────────────────────────────
    motorLeft.init();
    motorRight.init();

    // ── Módulo 2 ─────────────────────────────────────────────
    pcnt_isr_service_install(0);  // Una sola vez para ambos encoders
    encLeft.init();
    encRight.init();
    encLeft.reset();
    encRight.reset();

    // ── Módulo 3 (ultrasónico de seguridad) ──────────────────
    sonarFront.init();

    // ── Módulo 4: Mapa ───────────────────────────────────────
    if (!occMap.init()) {
        Serial.println("[ERROR] No se pudo inicializar el mapa. Verificar PSRAM.");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // ── Módulo 4: LiDAR ──────────────────────────────────────
    lidar.init(LIDAR_RX, LIDAR_TX, 115200);
    lidar.startScan();

    // ── SLAM Navigator ───────────────────────────────────────
    slam.init(PPR, WHEEL_DIAM, WHEELBASE);
    slam.setCruiseSpeed(SLAM_CRUISE_SPEED);
    slam.setTurnSpeed(SLAM_TURN_SPEED);
    slam.setFrontClearanceMm(SLAM_FRONT_CLEAR);

    Serial.println("[SMR-S3] Hardware OK. Esperando primer scan completo...");
    // Esperar a que el LiDAR complete 2 revoluciones antes de navegar
    while (!lidar.isReady()) {
        lidar.update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.println("[SMR-S3] LiDAR listo. Lanzando tasks...");

    // ── Tasks FreeRTOS ───────────────────────────────────────
    // Core 0 — tiempo real
    xTaskCreatePinnedToCore(taskLidar, "LiDAR",    4096, nullptr, 6, nullptr, 0);
    xTaskCreatePinnedToCore(taskPID,   "PID",       4096, nullptr, 5, nullptr, 0);
    // Core 1 — navegación
    xTaskCreatePinnedToCore(taskSLAM,  "SLAM",      8192, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(taskTelemetry, "Telem", 4096, nullptr, 1, nullptr, 1);
}

// ── loop() vacío: toda la lógica corre en tasks ──────────────
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
