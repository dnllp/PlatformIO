/*
 * ============================================================
 * main.cpp — Programa de Demostración (Nivel 1: Control PWM)
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Secuencia de prueba:
 *   1. Avanza 2 segundos al 60% de velocidad
 *   2. Gira a la derecha 1 segundo
 *   3. Retrocede 1 segundo
 *   4. Gira a la izquierda 1 segundo
 *   5. Se detiene — ciclo completo
 *
 * Ajusta los pines GPIO según tu hardware antes de compilar.
 * ============================================================
 */

#include "Arduino.h"
#include "RobotBase.h"

// ── Definición de pines (ajustar según PCB) ─────────────────
// Motor Izquierdo — IN1 e IN2 del L298N (canal A)
static constexpr gpio_num_t LEFT_IN1_PIN  = GPIO_NUM_5;
static constexpr gpio_num_t LEFT_IN2_PIN  = GPIO_NUM_6;

// Motor Derecho — IN3 e IN4 del L298N (canal B)
static constexpr gpio_num_t RIGHT_IN1_PIN = GPIO_NUM_7;
static constexpr gpio_num_t RIGHT_IN2_PIN = GPIO_NUM_15;

// Nota: ENA y ENB del L298N deben estar en HIGH (puente o 5V fijo)
//       Se necesitan 4 canales LEDC en total (2 por motor)

// ── Velocidades de prueba ────────────────────────────────────
static constexpr int SPEED_FULL  = 8191;           // 100%
static constexpr int SPEED_MED   = (int)(8191 * 0.60f); // 60%
static constexpr int SPEED_LOW   = (int)(8191 * 0.35f); // 35%

// ── Instancias de motores ────────────────────────────────────
// Cada motor usa 2 canales LEDC: uno para IN1, otro para IN2
DiffMotor motorLeft (LEFT_IN1_PIN,  LEFT_IN2_PIN,  LEDC_CHANNEL_0, LEDC_CHANNEL_1);
DiffMotor motorRight(RIGHT_IN1_PIN, RIGHT_IN2_PIN, LEDC_CHANNEL_2, LEDC_CHANNEL_3);
RobotBase robot(motorLeft, motorRight);

// ── setup() ─────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("[SMR-S3] Iniciando módulo de control de motores...");

    robot.init();

    Serial.println("[SMR-S3] Motores inicializados. Iniciando secuencia de prueba.");
    delay(1000);
}

// ── loop() ──────────────────────────────────────────────────
void loop() {
    // 1. Avanzar
    Serial.printf("[SMR-S3] Avanzando  — speed: %d\n", SPEED_MED);
    robot.forward(SPEED_MED);
    delay(2000);

    // 2. Girar a la derecha
    Serial.printf("[SMR-S3] Girando derecha — speed: %d\n", SPEED_LOW);
    robot.turnRight(SPEED_LOW);
    delay(1000);

    // 3. Retroceder
    Serial.printf("[SMR-S3] Retrocediendo — speed: %d\n", SPEED_MED);
    robot.backward(SPEED_MED);
    delay(1000);

    // 4. Girar a la izquierda
    Serial.printf("[SMR-S3] Girando izquierda — speed: %d\n", SPEED_LOW);
    robot.turnLeft(SPEED_LOW);
    delay(1000);

    // 5. Parar y esperar
    Serial.println("[SMR-S3] Detenido — esperando 2s...");
    robot.stop();
    delay(2000);
}
