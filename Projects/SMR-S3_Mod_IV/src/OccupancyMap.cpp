/*
 * ============================================================
 * OccupancyMap.cpp — Implementación del Mapa de Ocupación 2D
 * Framework SMR-S3 | Sistema Modular de Robótica Diferencial
 * ============================================================
 */

#include "OccupancyMap.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "Arduino.h"
#include <string.h>
#include <stdlib.h>

static const char* TAG = "OccupancyMap";

// ── Constructor ──────────────────────────────────────────────
OccupancyMap::OccupancyMap() : _grid(nullptr), _ready(false) {}

// ── init() ───────────────────────────────────────────────────
bool OccupancyMap::init() {
    size_t mapSize = (size_t)MAP_WIDTH_CELLS * MAP_HEIGHT_CELLS;

    // Sin PSRAM: asignar directamente en RAM interna
    // Con MAP_WIDTH=200 x MAP_HEIGHT=200 = 40 KB — cabe con margen
    _grid = static_cast<uint8_t*>(malloc(mapSize));

    if (!_grid) {
        ESP_LOGE(TAG, "Sin memoria para el mapa (%u bytes libres).",
                 (unsigned)ESP.getFreeHeap());
        return false;
    }

    memset(_grid, CELL_UNKNOWN, mapSize);
    _ready = true;

    ESP_LOGI(TAG, "Mapa %.0f x %.0f m inicializado (%u KB en RAM interna).",
             WIDTH_CM / 100.0f, HEIGHT_CM / 100.0f,
             (unsigned)(mapSize / 1024));
    return true;
}

// ── clear() ──────────────────────────────────────────────────
void OccupancyMap::clear() {
    if (_grid) {
        memset(_grid, CELL_UNKNOWN,
               (size_t)MAP_WIDTH_CELLS * MAP_HEIGHT_CELLS);
    }
}

// ── updateRay() ──────────────────────────────────────────────
void OccupancyMap::updateRay(float robotX_cm, float robotY_cm,
                             float angleDeg, float distMm) {
    if (!_ready) return;

    float angleRad = angleDeg * (float)M_PI / 180.0f;
    float distCm   = distMm / 10.0f;

    // Coordenadas del obstáculo en cm (frame global)
    float hitX = robotX_cm + distCm * cosf(angleRad);
    float hitY = robotY_cm + distCm * sinf(angleRad);

    int16_t rx, ry, hx, hy;
    if (!_cmToCell(robotX_cm, robotY_cm, rx, ry)) return;
    if (!_cmToCell(hitX, hitY, hx, hy)) return;

    // Trazar el rayo libre hasta el obstáculo (Bresenham)
    _traceFreeRay(rx, ry, hx, hy);

    // Marcar la celda del obstáculo como ocupada
    uint8_t& c = _cell(hx, hy);
    if (c < 255 - CELL_HIT_INC) c += CELL_HIT_INC;
    else                         c  = CELL_OCCUPIED;
}

// ── updateFullScan() ─────────────────────────────────────────
void OccupancyMap::updateFullScan(float robotX_cm, float robotY_cm,
                                  float robotYaw,
                                  const float scanDist[360]) {
    for (uint16_t i = 0; i < 360; i++) {
        if (scanDist[i] < 1.0f) continue;  // Punto inválido

        // Ángulo global = ángulo del sensor + orientación del robot
        float globalAngle = _normalizeAngle(static_cast<float>(i) + robotYaw);
        updateRay(robotX_cm, robotY_cm, globalAngle, scanDist[i]);
    }
}

// ── getCellAt() ──────────────────────────────────────────────
int16_t OccupancyMap::getCellAt(float x_cm, float y_cm) const {
    int16_t cx, cy;
    if (!_cmToCell(x_cm, y_cm, cx, cy)) return -1;
    return static_cast<int16_t>(_cellRead(cx, cy));
}

// ── isOccupied() / isFree() ──────────────────────────────────
bool OccupancyMap::isOccupied(float x_cm, float y_cm) const {
    int16_t v = getCellAt(x_cm, y_cm);
    return (v > CELL_UNKNOWN);
}

bool OccupancyMap::isFree(float x_cm, float y_cm) const {
    int16_t v = getCellAt(x_cm, y_cm);
    return (v >= 0 && v <= CELL_FREE + CELL_HIT_INC);
}

// ── printASCII() ─────────────────────────────────────────────
void OccupancyMap::printASCII(uint8_t scale) const {
    if (!_ready) return;
    uint16_t cols = MAP_WIDTH_CELLS  / scale;
    uint16_t rows = MAP_HEIGHT_CELLS / scale;

    for (uint16_t r = 0; r < rows; r++) {
        for (uint16_t c = 0; c < cols; c++) {
            uint8_t v = _cellRead(c * scale, r * scale);
            if (v == CELL_UNKNOWN)       Serial.print('?');
            else if (v >= CELL_OCCUPIED - CELL_HIT_INC) Serial.print('#');
            else                         Serial.print('.');
        }
        Serial.println();
    }
}

// ── _cmToCell() ──────────────────────────────────────────────
bool OccupancyMap::_cmToCell(float x_cm, float y_cm,
                              int16_t& cx, int16_t& cy) const {
    // El origen está en el centro del mapa
    cx = static_cast<int16_t>((x_cm / CELL_SIZE_CM) + MAP_WIDTH_CELLS  / 2);
    cy = static_cast<int16_t>((y_cm / CELL_SIZE_CM) + MAP_HEIGHT_CELLS / 2);

    return (cx >= 0 && cx < MAP_WIDTH_CELLS &&
            cy >= 0 && cy < MAP_HEIGHT_CELLS);
}

// ── _cell() / _cellRead() ────────────────────────────────────
uint8_t& OccupancyMap::_cell(int16_t cx, int16_t cy) {
    return _grid[(size_t)cy * MAP_WIDTH_CELLS + cx];
}

uint8_t OccupancyMap::_cellRead(int16_t cx, int16_t cy) const {
    return _grid[(size_t)cy * MAP_WIDTH_CELLS + cx];
}

// ── _traceFreeRay() — Bresenham ───────────────────────────────
void OccupancyMap::_traceFreeRay(int16_t x0, int16_t y0,
                                  int16_t x1, int16_t y1) {
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;

    while (!(x0 == x1 && y0 == y1)) {
        // Marcar celda actual como libre (decrementar)
        if (x0 >= 0 && x0 < MAP_WIDTH_CELLS &&
            y0 >= 0 && y0 < MAP_HEIGHT_CELLS) {
            uint8_t& c = _cell(x0, y0);
            if (c > CELL_FREE + CELL_FREE_DEC) c -= CELL_FREE_DEC;
            else                               c  = CELL_FREE;
        }
        int16_t e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

// ── _normalizeAngle() ────────────────────────────────────────
float OccupancyMap::_normalizeAngle(float deg) {
    while (deg < 0.0f)    deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}
