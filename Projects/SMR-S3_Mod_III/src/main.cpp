/*
 * ============================================================
 * main3.cpp — Programa de Demostración (Nivel 3: Ultrasónicos)
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Arquitectura de tasks FreeRTOS:
 *   Core 0 → taskAvoidance : máquina de evasión a 50 ms
 *   Core 0 → taskPIDControl: lazo PID a 20 ms (del Módulo 2)
 *   Core 1 → taskTelemetry : telemetría Serial cada 300 ms
 *
 * El robot avanza con control PID de velocidad (Módulo 2) y
 * la capa de evasión (Módulo 3) modifica los setpoints de
 * MotorPID cuando detecta obstáculos.
 *
 * Ajustar pines GPIO según el hardware real antes de compilar.
 * ============================================================
 */

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ── Módulo 1 ─────────────────────────────────────────────────
#include "DiffMotor.h"
#include "RobotBase.h"

// ── Módulo 2 ─────────────────────────────────────────────────
#include "EncoderReader.h"
#include "PIDController.h"
#include "MotorPID.h"

// ── Módulo 3 ─────────────────────────────────────────────────
#include "UltrasonicSensor.h"
#include "ObstacleAvoider.h"

// ────────────────────────────────────────────────────────────
// CONFIGURACIÓN DE HARDWARE — ajustar según su robot
// ────────────────────────────────────────────────────────────

// ── Motores L298N ────────────────────────────────────────────
static constexpr gpio_num_t LEFT_IN1  = GPIO_NUM_5;
static constexpr gpio_num_t LEFT_IN2  = GPIO_NUM_6;
static constexpr gpio_num_t RIGHT_IN1 = GPIO_NUM_7;
static constexpr gpio_num_t RIGHT_IN2 = GPIO_NUM_15;

// ── Encoders ─────────────────────────────────────────────────
static constexpr gpio_num_t LEFT_ENC_A  = GPIO_NUM_36;
static constexpr gpio_num_t LEFT_ENC_B  = GPIO_NUM_39;
static constexpr gpio_num_t RIGHT_ENC_A = GPIO_NUM_34;
static constexpr gpio_num_t RIGHT_ENC_B = GPIO_NUM_35;
static constexpr uint16_t   PPR         = 341;
static constexpr float      WHEEL_DIAM  = 6.0f;  // cm

// ── Sensores Ultrasónicos HC-SR04 ────────────────────────────
// Sensor frontal (obligatorio)
static constexpr gpio_num_t TRIG_FRONT = GPIO_NUM_18;
static constexpr gpio_num_t ECHO_FRONT = GPIO_NUM_19;

// Sensor izquierdo (opcional — comentar si no existe)
static constexpr gpio_num_t TRIG_LEFT  = GPIO_NUM_20;
static constexpr gpio_num_t ECHO_LEFT  = GPIO_NUM_21;

// Sensor derecho (opcional — comentar si no existe)
static constexpr gpio_num_t TRIG_RIGHT = GPIO_NUM_9;
static constexpr gpio_num_t ECHO_RIGHT = GPIO_NUM_10;

// ── Umbrales de evasión ──────────────────────────────────────
static constexpr float STOP_CM      = 15.0f;  // Frena si frente < 15 cm
static constexpr float WARN_CM      = 30.0f;  // Reduce vel. si < 30 cm
static constexpr float SIDE_BLOCK   = 20.0f;  // Lateral bloqueado si < 20 cm
static constexpr int   CRUISE_SPEED = 3500;   // Velocidad crucero [0-8191]
static constexpr int   TURN_SPEED   = 2500;   // Velocidad de giro

// ── PID ──────────────────────────────────────────────────────
static constexpr float  KP           = 3.0f;
static constexpr float  KI           = 0.8f;
static constexpr float  KD           = 0.05f;
static constexpr uint32_t PID_MS     = 20;
static constexpr uint32_t AVOID_MS   = 50;

// ────────────────────────────────────────────────────────────
// INSTANCIAS — Módulo 1
// ────────────────────────────────────────────────────────────
DiffMotor motorLeft (LEFT_IN1, LEFT_IN2, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
DiffMotor motorRight(RIGHT_IN1, RIGHT_IN2, LEDC_CHANNEL_2, LEDC_CHANNEL_3);
RobotBase robot(motorLeft, motorRight);

// ────────────────────────────────────────────────────────────
// INSTANCIAS — Módulo 2
// ────────────────────────────────────────────────────────────
EncoderReader encLeft (LEFT_ENC_A,  LEFT_ENC_B,  PCNT_UNIT_0);
EncoderReader encRight(RIGHT_ENC_A, RIGHT_ENC_B, PCNT_UNIT_1);
PIDController pidLeft (KP, KI, KD, -8191.0f, 8191.0f);
PIDController pidRight(KP, KI, KD, -8191.0f, 8191.0f);
MotorPID motorPIDLeft (motorLeft,  encLeft,  pidLeft);
MotorPID motorPIDRight(motorRight, encRight, pidRight);

// ────────────────────────────────────────────────────────────
// INSTANCIAS — Módulo 3
// ────────────────────────────────────────────────────────────
UltrasonicSensor sonarFront(TRIG_FRONT, ECHO_FRONT);
UltrasonicSensor sonarLeft (TRIG_LEFT,  ECHO_LEFT);
UltrasonicSensor sonarRight(TRIG_RIGHT, ECHO_RIGHT);

// Pasar punteros para sensores laterales opcionales
// Si no tiene laterales, reemplazar &sonarLeft/&sonarRight por nullptr
ObstacleAvoider avoider(robot, &sonarLeft, sonarFront, &sonarRight);

// ────────────────────────────────────────────────────────────
// TASK: EVASIÓN DE OBSTÁCULOS (Core 0, 20 Hz)
// ────────────────────────────────────────────────────────────
void taskAvoidance(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
        avoider.update();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(AVOID_MS));
    }
}

// ────────────────────────────────────────────────────────────
// TASK: LAZO PID (Core 0, 50 Hz)
// ────────────────────────────────────────────────────────────
void taskPIDControl(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
        // El PID solo actualiza si el avoider no está manejando el robot
        // directamente. En estado FORWARD, el avoider llama forward()
        // que usa drive() del DiffMotor (sin PID). Para cerrar el lazo
        // en movimiento libre, usar MotorPID aquí es opcional en M3;
        // se integra completamente en el Módulo 4.
        motorPIDLeft.update(PID_MS);
        motorPIDRight.update(PID_MS);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PID_MS));
    }
}

// ────────────────────────────────────────────────────────────
// TASK: TELEMETRÍA SERIAL (Core 1, cada 300 ms)
// ────────────────────────────────────────────────────────────
void taskTelemetry(void* pvParameters) {
    while (true) {
        Serial.printf(
            "[SMR-S3] Estado: %-12s | "
            "Izq: %5.1f cm | Front: %5.1f cm | Der: %5.1f cm | "
            "Vel L: %5.0f p/s | Vel R: %5.0f p/s\n",
            avoider.getStateName(),
            avoider.getDistLeft(),
            avoider.getDistFront(),
            avoider.getDistRight(),
            motorPIDLeft.getMeasuredSpeed(),
            motorPIDRight.getMeasuredSpeed()
        );
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// ────────────────────────────────────────────────────────────
// SETUP
// ────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("[SMR-S3] Módulo 3 — Sensores Ultrasónicos arrancando...");

    // ── Módulo 1: motores ────────────────────────────────────
    motorLeft.init();
    motorRight.init();

    // ── Módulo 2: encoders ───────────────────────────────────
    // Instalar servicio ISR una sola vez antes de ambos encoders
    pcnt_isr_service_install(0);
    encLeft.init();
    encRight.init();
    encLeft.reset();
    encRight.reset();

    // ── Módulo 3: sensores ultrasónicos ──────────────────────
    // Instalar ISR de GPIO una sola vez
    // (gpio_install_isr_service se llama dentro de sonarFront.init())
    sonarFront.init();
    sonarLeft.init();
    sonarRight.init();

    // Configurar umbrales y velocidades del avoider
    avoider.setThresholds(STOP_CM, WARN_CM, SIDE_BLOCK);
    avoider.setCruiseSpeed(CRUISE_SPEED);
    avoider.setTurnSpeed(TURN_SPEED);

    Serial.println("[SMR-S3] Hardware inicializado. Lanzando tasks...");

    // ── Lanzar tasks FreeRTOS ────────────────────────────────
    xTaskCreatePinnedToCore(taskPIDControl, "PID_Ctrl",   4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(taskAvoidance,  "Avoidance",  4096, nullptr, 4, nullptr, 0);
    xTaskCreatePinnedToCore(taskTelemetry,  "Telemetry",  4096, nullptr, 1, nullptr, 1);
}

// ────────────────────────────────────────────────────────────
// LOOP — vacío: toda la lógica corre en tasks FreeRTOS
// ────────────────────────────────────────────────────────────
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
