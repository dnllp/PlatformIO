/*
 * ============================================================
 * diagnostico.cpp — Verificación de PSRAM y memoria
 * Framework SMR-S3 | Diagnóstico previo al Módulo 4
 * ============================================================
 * Cargar este sketch ANTES del main4.cpp para confirmar
 * que el ESP32-S3 tiene PSRAM disponible y suficiente memoria
 * para el OccupancyMap (360 KB).
 * ============================================================
 */

#include "Arduino.h"
#include "esp_heap_caps.h"
#include "esp_system.h"

void setup() {
    Serial.begin(115200);
    delay(2000);  // Dar tiempo al monitor serial a conectarse

    Serial.println("============================================");
    Serial.println("  SMR-S3 — Diagnóstico de Memoria y PSRAM");
    Serial.println("============================================");

    // ── Información general ──────────────────────────────────
    Serial.printf("\nChip: %s  Rev: %d  Cores: %d  Freq: %d MHz\n",
        ESP.getChipModel(),
        ESP.getChipRevision(),
        ESP.getChipCores(),
        ESP.getCpuFreqMHz());

    Serial.printf("Flash: %u MB\n\n", ESP.getFlashChipSize() / (1024 * 1024));

    // ── RAM interna ──────────────────────────────────────────
    Serial.println("── RAM Interna ──────────────────────────");
    Serial.printf("  Total heap    : %6u bytes  (%u KB)\n",
        ESP.getHeapSize(), ESP.getHeapSize() / 1024);
    Serial.printf("  Heap libre    : %6u bytes  (%u KB)\n",
        ESP.getFreeHeap(), ESP.getFreeHeap() / 1024);
    Serial.printf("  Heap mín libre: %6u bytes  (%u KB)\n",
        ESP.getMinFreeHeap(), ESP.getMinFreeHeap() / 1024);

    // ── PSRAM ────────────────────────────────────────────────
    Serial.println("\n── PSRAM ────────────────────────────────");
    size_t psramTotal = ESP.getPsramSize();
    size_t psramFree  = ESP.getFreePsram();

    if (psramTotal == 0) {
        Serial.println("  ❌ PSRAM NO DETECTADA");
        Serial.println("     → Su módulo ESP32-S3 no tiene PSRAM soldada,");
        Serial.println("       o falta board_build.arduino.memory_type = qio_opi");
        Serial.println("       en el platformio.ini.");
    } else {
        Serial.printf("  ✅ PSRAM detectada\n");
        Serial.printf("  Total PSRAM   : %6u bytes  (%u KB / %.1f MB)\n",
            psramTotal, psramTotal / 1024, psramTotal / 1048576.0f);
        Serial.printf("  PSRAM libre   : %6u bytes  (%u KB)\n",
            psramFree, psramFree / 1024);
    }

    // ── Prueba de asignación del OccupancyMap ─────────────────
    Serial.println("\n── Prueba de asignación OccupancyMap (360 KB) ──");
    size_t mapSize = 600UL * 600UL;  // 360 000 bytes

    // Intentar en PSRAM
    void* ptrPSRAM = heap_caps_malloc(mapSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ptrPSRAM) {
        Serial.println("  ✅ Asignación en PSRAM: OK");
        free(ptrPSRAM);
    } else {
        Serial.println("  ❌ Asignación en PSRAM: FALLÓ");

        // Intentar en RAM interna como fallback
        void* ptrRAM = malloc(mapSize);
        if (ptrRAM) {
            Serial.println("  ⚠  Asignación en RAM interna: OK (pero ajustada)");
            Serial.println("     → El mapa funcionará pero consumirá toda la RAM libre.");
            Serial.println("       Reducir MAP_WIDTH_CELLS y MAP_HEIGHT_CELLS en OccupancyMap.h");
            free(ptrRAM);
        } else {
            Serial.println("  ❌ Asignación en RAM interna: FALLÓ también");
            Serial.println("     → No hay memoria suficiente para el OccupancyMap.");
            Serial.println("       Soluciones: habilitar PSRAM o reducir el mapa.");
        }
    }

    // ── Heap por capacidades ──────────────────────────────────
    Serial.println("\n── Memoria disponible por tipo ──────────");
    Serial.printf("  MALLOC_CAP_8BIT   : %7u bytes libres\n",
        heap_caps_get_free_size(MALLOC_CAP_8BIT));
    Serial.printf("  MALLOC_CAP_32BIT  : %7u bytes libres\n",
        heap_caps_get_free_size(MALLOC_CAP_32BIT));
    Serial.printf("  MALLOC_CAP_DMA    : %7u bytes libres\n",
        heap_caps_get_free_size(MALLOC_CAP_DMA));
    Serial.printf("  MALLOC_CAP_SPIRAM : %7u bytes libres\n",
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    Serial.printf("  MALLOC_CAP_INTERNAL:%7u bytes libres\n",
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    Serial.println("\n============================================");
    Serial.println("  Diagnóstico completado.");
    Serial.println("============================================\n");
}

void loop() {
    // Repetir reporte cada 5 segundos para facilitar la lectura
    delay(5000);
    Serial.printf("[live] Heap libre: %u KB  |  PSRAM libre: %u KB\n",
        ESP.getFreeHeap() / 1024,
        ESP.getFreePsram() / 1024);
}