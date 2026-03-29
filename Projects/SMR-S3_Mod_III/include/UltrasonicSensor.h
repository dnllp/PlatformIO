/*
 * ============================================================
 * UltrasonicSensor.h — Módulo de Sensor Ultrasónico HC-SR04
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Plataforma : ESP32-S3
 * Sensor     : HC-SR04 (compatible con HY-SRF05, JSN-SR04T)
 * Nivel      : 3 — Percepción reactiva del entorno
 *
 * Descripción:
 *   Mide distancias mediante pulso ultrasónico de 40 kHz.
 *   La medición del pulso de eco se realiza con una ISR de GPIO
 *   y el timer de alta resolución esp_timer, evitando el bloqueo
 *   de la CPU con pulseIn() que no es apto para FreeRTOS.
 *
 * Principio de operación:
 *   1. Se envía un pulso TRIG de 10 µs
 *   2. El sensor emite 8 pulsos de 40 kHz
 *   3. El pin ECHO sube al detectar el eco y baja al recibirlo
 *   4. La duración del pulso ECHO en µs → distancia en cm:
 *        distancia_cm = duracion_us / 58.0
 *        (velocidad del sonido ~343 m/s, ida y vuelta)
 *
 * Uso básico:
 *   UltrasonicSensor sonar(GPIO_NUM_18, GPIO_NUM_19);
 *   sonar.init();
 *   float dist = sonar.getDistanceCm();  // distancia en cm
 *   if (dist < 20.0f) { robot.stop(); }
 * ============================================================
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include <stdint.h>

// ── Constantes físicas y límites ─────────────────────────────
static constexpr float    US_SOUND_FACTOR    = 58.0f;  // µs/cm (ida+vuelta)
static constexpr float    US_MAX_DISTANCE_CM = 400.0f; // Rango máximo HC-SR04
static constexpr float    US_MIN_DISTANCE_CM = 2.0f;   // Zona ciega (mín 2 cm)
static constexpr uint32_t US_TIMEOUT_US      = 25000;  // 25 ms → ~400 cm
static constexpr uint32_t US_TRIG_PULSE_US   = 10;     // Pulso TRIG: 10 µs

class UltrasonicSensor {
public:
    /**
     * Constructor
     * @param pinTrig  GPIO conectado al pin TRIG del HC-SR04
     * @param pinEcho  GPIO conectado al pin ECHO del HC-SR04
     *
     * IMPORTANTE: El HC-SR04 opera a 5V. El pin ECHO emite 5V.
     * El ESP32-S3 acepta máximo 3.3V en sus GPIO.
     * Usar un divisor resistivo (1kΩ / 2kΩ) o un módulo
     * conversor de nivel lógico en la línea ECHO.
     */
    UltrasonicSensor(gpio_num_t pinTrig, gpio_num_t pinEcho);

    /**
     * Configura los GPIOs y registra la ISR del pin ECHO.
     * Llamar una vez en setup().
     */
    void init();

    /**
     * Dispara una medición y devuelve la distancia en centímetros.
     * Bloquea brevemente (~duración del eco, máx 25 ms).
     * Para uso no bloqueante, ver triggerAsync() + getLastDistance().
     * @return  Distancia en cm, o -1.0f si hay timeout o fuera de rango.
     */
    float getDistanceCm();

    /**
     * Inicia una medición asíncrona (no bloqueante).
     * El resultado estará disponible en getLastDistance() tras ~25 ms.
     * Usar desde un task FreeRTOS con vTaskDelay entre llamadas.
     */
    void triggerAsync();

    /**
     * Devuelve la última distancia medida (modo asíncrono).
     * Devuelve -1.0f si la medición no ha completado aún.
     */
    float getLastDistance() const { return _lastDistanceCm; }

    /**
     * Indica si hay un obstáculo más cercano que el umbral dado.
     * @param thresholdCm  Distancia mínima en cm
     */
    bool isObstacleDetected(float thresholdCm) const;

private:
    gpio_num_t _pinTrig;
    gpio_num_t _pinEcho;

    volatile int64_t _echoStart;   // Timestamp de flanco de subida (µs)
    volatile int64_t _echoDuration; // Duración del pulso ECHO (µs)
    volatile bool    _measuring;    // Medición en curso

    float _lastDistanceCm;

    // ISR para flancos del pin ECHO
    static void IRAM_ATTR _echoISR(void* arg);

    // Convierte duración µs a distancia cm
    static float _usToCm(int64_t durationUs);
};

#endif // ULTRASONIC_SENSOR_H
