/*
 * ============================================================
 * EncoderReader.h — Módulo de Lectura de Encoders de Cuadratura
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Plataforma : ESP32-S3
 * Periférico : PCNT (Pulse Counter) — decodificación por hardware
 * Nivel      : 2 — Closed Loop (retroalimentación de posición)
 *
 * Descripción:
 *   Lee un encoder de cuadratura (señales A y B) usando el periférico
 *   PCNT del ESP32-S3, que cuenta pulsos sin intervención de la CPU.
 *   Detecta dirección automáticamente por la fase entre A y B.
 *   Acumula un contador de 32 bits con overflow manejado por ISR.
 *
 * Tabla de verdad PCNT (cuadratura):
 *   A sube, B=LOW  → +1 (avance)
 *   A sube, B=HIGH → -1 (retroceso)
 *   B sube, A=LOW  → -1 (retroceso)
 *   B sube, A=HIGH → +1 (avance)
 *
 * Uso básico:
 *   EncoderReader encIzq(GPIO_NUM_36, GPIO_NUM_39, PCNT_UNIT_0);
 *   encIzq.init();
 *   encIzq.reset();
 *   int32_t pulsos = encIzq.getCount();   // contador acumulado
 *   float   vel    = encIzq.getSpeed();   // pulsos/segundo
 * ============================================================
 */

#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include "driver/pcnt.h"
#include "esp_attr.h"
#include <stdint.h>

// ── Límites del contador hardware (16 bits con signo) ────────
// Al llegar a estos valores se dispara la ISR de overflow
static constexpr int16_t PCNT_HIGH_LIMIT =  32000;
static constexpr int16_t PCNT_LOW_LIMIT  = -32000;

class EncoderReader {
public:
    /**
     * Constructor
     * @param pinA    GPIO conectado a la señal A del encoder
     * @param pinB    GPIO conectado a la señal B del encoder
     * @param unit    Unidad PCNT (PCNT_UNIT_0 a PCNT_UNIT_3 disponibles)
     */
    EncoderReader(gpio_num_t pinA, gpio_num_t pinB, pcnt_unit_t unit);

    /**
     * Inicializa el periférico PCNT en modo cuadratura completa (x4).
     * Registra la ISR de overflow para acumulación de 32 bits.
     * Llamar una vez en setup().
     */
    void init();

    /**
     * Reinicia el contador acumulado a cero.
     * Usar al inicio de una maniobra de distancia.
     */
    void reset();

    /**
     * Devuelve el conteo acumulado de pulsos (32 bits con signo).
     * Positivo = avance, Negativo = retroceso.
     */
    int32_t getCount();

    /**
     * Devuelve la velocidad en pulsos por segundo.
     * Debe llamarse periódicamente (ej. cada 20 ms en un task FreeRTOS).
     * @param dtMs  Tiempo transcurrido desde la última llamada (milisegundos)
     */
    float getSpeed(uint32_t dtMs);

    /**
     * Convierte pulsos a centímetros según la resolución del encoder
     * y el diámetro de la rueda configurados.
     * @param pulses         Número de pulsos a convertir
     * @param pulsesPerRev   Pulsos por revolución del encoder (PPR x4 en cuadratura)
     * @param wheelDiamCm    Diámetro de la rueda en centímetros
     */
    static float pulsesToCm(int32_t pulses, uint16_t pulsesPerRev, float wheelDiamCm);

private:
    gpio_num_t  _pinA;
    gpio_num_t  _pinB;
    pcnt_unit_t _unit;

    // Acumulador de 32 bits (el PCNT interno es de 16 bits)
    volatile int32_t _accumulator;

    // Conteo del ciclo anterior para calcular velocidad
    int32_t _lastCount;

    // ISR estática que recibe puntero a la instancia
    static void IRAM_ATTR _overflowISR(void* arg);
};

#endif // ENCODER_READER_H
