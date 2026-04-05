/*
 * ============================================================
 * OccupancyMap.h — Mapa de Ocupación 2D (Grid Map)
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 * Nivel : 4 — Navegación autónoma (SLAM)
 *
 * Descripción:
 *   Implementa un mapa de ocupación binario (occupancy grid)
 *   almacenado en PSRAM del ESP32-S3 (8 MB disponibles).
 *   Cada celda del grid representa un área física de CELL_SIZE_CM².
 *
 *   Valores de celda:
 *     0    = libre (explorado y despejado)
 *     127  = desconocido (no explorado)
 *     255  = ocupado (obstáculo detectado)
 *
 *   El mapa se actualiza con los puntos del LiDAR usando el
 *   algoritmo de Bresenham para marcar el rayo libre hasta
 *   el obstáculo detectado.
 *
 * Coordenadas:
 *   El origen (0,0) está en el centro del mapa.
 *   X positivo = derecha, Y positivo = adelante (convención robótica).
 *   Las celdas se acceden con coordenadas en centímetros.
 * ============================================================
 */

#ifndef OCCUPANCY_MAP_H
#define OCCUPANCY_MAP_H

#include <stdint.h>
#include <math.h>

// ── Parámetros del mapa ──────────────────────────────────────
// Mapa de 200×200 celdas a 5 cm/celda = 10×10 metros
// Memoria: 200×200 = 40 000 bytes ≈ 40 KB (cabe en RAM interna)
static constexpr uint16_t MAP_WIDTH_CELLS  = 200;
static constexpr uint16_t MAP_HEIGHT_CELLS = 200;
static constexpr float    CELL_SIZE_CM     = 5.0f;   // cm por celda

// Valores de celda
static constexpr uint8_t CELL_UNKNOWN  = 127;
static constexpr uint8_t CELL_FREE     = 0;
static constexpr uint8_t CELL_OCCUPIED = 255;

// Incrementos de probabilidad de ocupación (log-odds light)
static constexpr uint8_t CELL_HIT_INC  = 40;  // Aumentar al detectar obstáculo
static constexpr uint8_t CELL_FREE_DEC = 10;  // Disminuir al ver espacio libre

class OccupancyMap {
public:
    OccupancyMap();

    /**
     * Inicializa el mapa marcando todas las celdas como desconocidas.
     * Asigna memoria en PSRAM si está disponible.
     */
    bool init();

    /**
     * Actualiza el mapa con un rayo LiDAR (algoritmo Bresenham).
     * @param robotX_cm   Posición X del robot en cm (origen del rayo)
     * @param robotY_cm   Posición Y del robot en cm
     * @param angleDeg    Ángulo del rayo en el frame global
     * @param distMm      Distancia medida por el LiDAR en mm
     */
    void updateRay(float robotX_cm, float robotY_cm,
                   float angleDeg, float distMm);

    /**
     * Actualiza el mapa con una revolución completa del LiDAR.
     * @param robotX_cm   Posición X del robot
     * @param robotY_cm   Posición Y del robot
     * @param robotYaw    Orientación del robot en grados
     * @param scan        Array de 360 ángulos con sus distancias
     * @param scanDist    Distancias en mm para cada ángulo [0-359]
     */
    void updateFullScan(float robotX_cm, float robotY_cm, float robotYaw,
                        const float scanDist[360]);

    /**
     * Devuelve el valor de ocupación de la celda en (x_cm, y_cm).
     * @return 0=libre, 127=desconocido, 255=ocupado, -1=fuera del mapa
     */
    int16_t getCellAt(float x_cm, float y_cm) const;

    /**
     * true si la celda en (x_cm, y_cm) está marcada como ocupada.
     */
    bool isOccupied(float x_cm, float y_cm) const;

    /**
     * true si la celda en (x_cm, y_cm) es libre (explorada y despejada).
     */
    bool isFree(float x_cm, float y_cm) const;

    /**
     * Borra el mapa completo (todo desconocido).
     */
    void clear();

    /**
     * Imprime una versión reducida del mapa por Serial (para debug).
     * @param scale  Factor de reducción (ej. 10 → 1 carácter por cada 10 celdas)
     */
    void printASCII(uint8_t scale = 10) const;

    // Dimensiones del mapa en cm
    static constexpr float WIDTH_CM  = MAP_WIDTH_CELLS  * CELL_SIZE_CM;
    static constexpr float HEIGHT_CM = MAP_HEIGHT_CELLS * CELL_SIZE_CM;

private:
    uint8_t* _grid;   // Puntero al buffer (PSRAM o heap)
    bool     _ready;

    // Convierte coordenadas cm → índice de celda
    bool _cmToCell(float x_cm, float y_cm,
                   int16_t& cx, int16_t& cy) const;

    // Acceso directo a celda por índice
    uint8_t& _cell(int16_t cx, int16_t cy);
    uint8_t  _cellRead(int16_t cx, int16_t cy) const;

    // Bresenham: marca las celdas libres a lo largo de un rayo
    void _traceFreeRay(int16_t x0, int16_t y0,
                       int16_t x1, int16_t y1);

    static float _normalizeAngle(float deg);
};

#endif // OCCUPANCY_MAP_H
