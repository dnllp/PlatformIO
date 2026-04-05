/*
 * ============================================================
 * main.cpp — Programa de Demostración (Nivel 2: Encoders + PID)
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Arquitectura dual-core FreeRTOS:
 *   Core 0 → Task PID: lazo de control a 20 ms (50 Hz)
 *   Core 1 → Task Monitor: imprime telemetría cada 500 ms
 *
 * Secuencia de prueba:
 *   1. Avanza 3 s a 400 pulsos/s con PID activo
 *   2. Avanza 3 s a 200 pulsos/s (cambio de setpoint)
 *   3. Retrocede 3 s a -300 pulsos/s
 *   4. Detiene y espera 2 s — repite
 *
 * Ajustar pines GPIO y PPR según el encoder físico instalado.
 * ============================================================
 */

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "DiffMotor.h"
#include "EncoderReader.h"
#include "PIDController.h"
#include "MotorPID.h"

// ────────────────────────────────────────────────────────────
// CONFIGURACIÓN DE HARDWARE — ajustar según su robot
// ────────────────────────────────────────────────────────────

// Motor Izquierdo (IN1, IN2 del L298N canal A)
static constexpr gpio_num_t LEFT_IN1  = GPIO_NUM_5;
static constexpr gpio_num_t LEFT_IN2  = GPIO_NUM_6;

// Motor Derecho (IN3, IN4 del L298N canal B)
static constexpr gpio_num_t RIGHT_IN1 = GPIO_NUM_7;
static constexpr gpio_num_t RIGHT_IN2 = GPIO_NUM_15;

// Encoder Izquierdo
static constexpr gpio_num_t LEFT_ENC_A = GPIO_NUM_36;
static constexpr gpio_num_t LEFT_ENC_B = GPIO_NUM_39;

// Encoder Derecho
static constexpr gpio_num_t RIGHT_ENC_A = GPIO_NUM_34;
static constexpr gpio_num_t RIGHT_ENC_B = GPIO_NUM_35;

// Resolución del encoder: pulsos por revolución (PPR)
// En modo cuadratura x4: PPR_FISICO × 4
// Ejemplo: encoder de 20 ranuras × 4 = 80 PPR efectivos
static constexpr uint16_t PPR          = 341;
static constexpr float    WHEEL_DIAM   = 6.0f;   // cm

// Parámetros PID — sintonizar según la respuesta real del robot
// Punto de partida: Kp alto, Ki bajo, Kd=0 → luego ajustar
static constexpr float KP = 3.0f;
static constexpr float KI = 0.8f;
static constexpr float KD = 0.05f;

// Período del lazo de control (ms)
static constexpr uint32_t PID_PERIOD_MS = 20;

// ────────────────────────────────────────────────────────────
// INSTANCIAS DE OBJETOS
// ────────────────────────────────────────────────────────────

// Motores (Módulo 1, sin cambios)
DiffMotor motorLeft (LEFT_IN1,  LEFT_IN2,  LEDC_CHANNEL_0, LEDC_CHANNEL_1);
DiffMotor motorRight(RIGHT_IN1, RIGHT_IN2, LEDC_CHANNEL_2, LEDC_CHANNEL_3);

// Encoders de cuadratura
EncoderReader encLeft (LEFT_ENC_A,  LEFT_ENC_B,  PCNT_UNIT_0);
EncoderReader encRight(RIGHT_ENC_A, RIGHT_ENC_B, PCNT_UNIT_1);

// Controladores PID (uno por motor, parámetros independientes)
PIDController pidLeft (KP, KI, KD, -8191.0f, 8191.0f);
PIDController pidRight(KP, KI, KD, -8191.0f, 8191.0f);

// Motores con PID integrado
MotorPID motorPIDLeft (motorLeft,  encLeft,  pidLeft);
MotorPID motorPIDRight(motorRight, encRight, pidRight);

// ────────────────────────────────────────────────────────────
// TASK: LAZO DE CONTROL PID (Core 0, 50 Hz)
// ────────────────────────────────────────────────────────────
void taskPIDControl(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {
        // Actualizar ambos lazos cada PID_PERIOD_MS
        motorPIDLeft.update(PID_PERIOD_MS);
        motorPIDRight.update(PID_PERIOD_MS);

        // Esperar exactamente el período (no acumula drift)
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PID_PERIOD_MS));
    }
}

// ────────────────────────────────────────────────────────────
// TASK: TELEMETRÍA SERIAL (Core 1, cada 500 ms)
// ────────────────────────────────────────────────────────────
void taskTelemetry(void* pvParameters) {
    while (true) {
        float speedL = motorPIDLeft.getMeasuredSpeed();
        float speedR = motorPIDRight.getMeasuredSpeed();
        float outL   = motorPIDLeft.getPIDOutput();
        float outR   = motorPIDRight.getPIDOutput();

        float distL  = EncoderReader::pulsesToCm(encLeft.getCount(),  PPR, WHEEL_DIAM);
        float distR  = EncoderReader::pulsesToCm(encRight.getCount(), PPR, WHEEL_DIAM);

        Serial.printf("[SMR-S3] Vel L: %6.1f p/s | Vel R: %6.1f p/s | "
                      "PWM L: %5.0f | PWM R: %5.0f | "
                      "Dist L: %5.1f cm | Dist R: %5.1f cm\n",
                      speedL, speedR, outL, outR, distL, distR);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ────────────────────────────────────────────────────────────
// SETUP
// ────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("[SMR-S3] Módulo 2 — Encoders + PID arrancando...");

    // Inicializar motores (Módulo 1)
    motorLeft.init();
    motorRight.init();

    // Inicializar encoders (PCNT)
    encLeft.init();
    encRight.init();
    encLeft.reset();
    encRight.reset();

    Serial.println("[SMR-S3] Hardware inicializado. Lanzando tasks FreeRTOS...");

    // Task PID → Core 0, prioridad alta
    xTaskCreatePinnedToCore(taskPIDControl, "PID_Control",
                            4096, nullptr, 5, nullptr, 0);

    // Task Telemetría → Core 1, prioridad baja
    xTaskCreatePinnedToCore(taskTelemetry, "Telemetry",
                            4096, nullptr, 1, nullptr, 1);
}

// ────────────────────────────────────────────────────────────
// LOOP — Secuenciador de setpoints (Core 1)
// ────────────────────────────────────────────────────────────
void loop() {
    // 1. Avanzar a 400 pulsos/s
    Serial.println("[SMR-S3] Setpoint: +400 p/s (avance rápido)");
    motorPIDLeft.setTargetSpeed( 400.0f);
    motorPIDRight.setTargetSpeed(400.0f);
    delay(3000);

    // 2. Avanzar a 200 pulsos/s (velocidad reducida)
    Serial.println("[SMR-S3] Setpoint: +200 p/s (avance lento)");
    motorPIDLeft.setTargetSpeed( 200.0f);
    motorPIDRight.setTargetSpeed(200.0f);
    delay(3000);

    // 3. Retroceder a 300 pulsos/s
    Serial.println("[SMR-S3] Setpoint: -300 p/s (retroceso)");
    motorPIDLeft.setTargetSpeed( -300.0f);
    motorPIDRight.setTargetSpeed(-300.0f);
    delay(3000);

    // 4. Parar
    Serial.println("[SMR-S3] Detenido");
    motorPIDLeft.stop();
    motorPIDRight.stop();
    delay(2000);
}
