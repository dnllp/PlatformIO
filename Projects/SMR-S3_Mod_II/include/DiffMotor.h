/*
 * ============================================================
 * DiffMotor.h — Módulo de Control de Motores DC
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Plataforma : ESP32-S3
 * Periférico : LEDC (PWM por hardware)
 * Nivel      : 1 — Control Open Loop (sin retroalimentación)
 *
 * Descripción:
 *   Abstrae el control de un motor DC con puente H L298N.
 *   Usa DOS canales LEDC por motor (uno para IN1, otro para IN2).
 *   La dirección se controla aplicando PWM a uno de los pines
 *   y mantiendo el otro en LOW, según la tabla de verdad del L298N:
 *
 *     IN1   | IN2   | Resultado
 *     -------|-------|----------
 *     PWM   | LOW   | Avance
 *     LOW   | PWM   | Retroceso
 *     LOW   | LOW   | Freno (rueda libre)
 *     HIGH  | HIGH  | Freno (cortocircuito — evitar)
 *
 *   El pin ENA del L298N debe estar conectado a 5V (puente físico)
 *   o también puede controlarse con un tercer canal LEDC si se
 *   quiere combinar con lógica de enable externo.
 *
 * Uso básico:
 *   // Motor izq: IN1=GPIO5, IN2=GPIO6, ch0 y ch1
 *   DiffMotor motorIzq(GPIO_NUM_5, GPIO_NUM_6,
 *                      LEDC_CHANNEL_0, LEDC_CHANNEL_1);
 *   motorIzq.init();
 *   motorIzq.drive(4000);   // avanza al ~48%  → IN1=PWM, IN2=LOW
 *   motorIzq.drive(-2000);  // retrocede al ~24% → IN1=LOW, IN2=PWM
 *   motorIzq.stop();        // freno → IN1=LOW, IN2=LOW
 * ============================================================
 */

#ifndef DIFF_MOTOR_H
#define DIFF_MOTOR_H

#include "driver/ledc.h"
#include "driver/gpio.h"

// ── Constantes del periférico LEDC ──────────────────────────
static constexpr ledc_mode_t       MOTOR_SPEED_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_bit_t  MOTOR_RESOLUTION = LEDC_TIMER_13_BIT;
static constexpr uint32_t          MOTOR_FREQ_HZ     = 5000;
static constexpr ledc_timer_t      MOTOR_TIMER       = LEDC_TIMER_0;
static constexpr int               MOTOR_DUTY_MAX     = 8191; // 2^13 - 1

class DiffMotor {
public:
    /**
     * Constructor
     * @param pinIN1   GPIO conectado a IN1 del L298N (avance)
     * @param pinIN2   GPIO conectado a IN2 del L298N (retroceso)
     * @param chIN1    Canal LEDC para IN1 (ej. LEDC_CHANNEL_0)
     * @param chIN2    Canal LEDC para IN2 (ej. LEDC_CHANNEL_1)
     *
     * Nota: ENA/ENB del L298N debe estar en HIGH (puente o 5V fijo).
     *       Se usan 2 canales LEDC por motor → 4 canales en total
     *       para un robot diferencial.
     */
    DiffMotor(gpio_num_t pinIN1, gpio_num_t pinIN2,
              ledc_channel_t chIN1, ledc_channel_t chIN2);

    /**
     * Inicializa ambos canales LEDC y configura los GPIOs.
     * Debe llamarse una vez en setup() antes de usar drive().
     */
    void init();

    /**
     * Controla velocidad y dirección del motor.
     * @param speed  Valor en rango [-8191, 8191]
     *               Positivo → avance    (IN1=PWM, IN2=0)
     *               Negativo → retroceso (IN1=0,   IN2=PWM)
     *               0        → freno     (IN1=0,   IN2=0)
     */
    void drive(int speed);

    /**
     * Frena el motor (IN1=LOW, IN2=LOW).
     */
    void stop();

    /**
     * Devuelve la velocidad actual asignada (sin corrección PID).
     */
    int getSpeed() const { return _currentSpeed; }

private:
    gpio_num_t     _pinIN1;
    gpio_num_t     _pinIN2;
    ledc_channel_t _chIN1;
    ledc_channel_t _chIN2;
    int            _currentSpeed;

    // Configura un canal LEDC en un GPIO
    void _initChannel(gpio_num_t pin, ledc_channel_t ch);

    // Aplica duty a un canal y pone el otro a 0
    void _applyDual(ledc_channel_t chActive, ledc_channel_t chIdle, uint32_t duty);
};

#endif // DIFF_MOTOR_H
