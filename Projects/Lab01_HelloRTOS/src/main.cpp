#include <Arduino.h>
 
// ── Handles de tareas ──────────────────────────────────
TaskHandle_t hTaskSensor = NULL;
TaskHandle_t hTaskDisplay = NULL;
TaskHandle_t hTaskHeartbeat = NULL;
 
// ── Tarea 1: lectura de sensor (core 1, prioridad 2) ───
void taskSensor(void* param) {
    Serial.printf("[Sensor]    Core %d  Prio %d\n", xPortGetCoreID(), uxTaskPriorityGet(NULL));
    uint32_t count = 0;
    for (;;) {
        // Simular lectura de sensor (200 ms)
        Serial.printf("[Sensor]    Lectura #%u\n", ++count);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ── Tarea 2: actualizar display (core 1, prioridad 1) ──
void taskDisplay(void* param) {
    Serial.printf("[Display]   Core %d  Prio %d\n", xPortGetCoreID(), uxTaskPriorityGet(NULL));
    for (;;) {
        Serial.println("[Display]   Actualizando pantalla...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
 
// ── Tarea 3: LED heartbeat (core 0, prioridad 1) ───────
void taskHeartbeat(void* param) {
    Serial.printf("[Heartbeat] Core %d  Prio %d\n", xPortGetCoreID(), uxTaskPriorityGet(NULL));
    const int LED = 2;
    pinMode(LED, OUTPUT);
    for (;;) {
        Serial.println("[Heartbeat] LED toggled");
        digitalWrite(LED, !digitalRead(LED));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
 
void setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.println("=== FreeRTOS ESP32 ===");
 
    //                      funcion        nombre      stack param  prio  handle     core
    xTaskCreatePinnedToCore(taskSensor,    "Sensor",    4096, NULL, 2, &hTaskSensor,    1);
    xTaskCreatePinnedToCore(taskDisplay,   "Display",   4096, NULL, 1, &hTaskDisplay,   1);
    xTaskCreatePinnedToCore(taskHeartbeat, "Heartbeat", 2048, NULL, 1, &hTaskHeartbeat, 0);
 
    Serial.println("Tareas creadas. loop() corre en core 1 prio 1.");
}
 
// loop() en ESP32-Arduino ES una tarea FreeRTOS en core 1
void loop() {
    // Imprimir estado de tareas cada 3 segundos
    static uint32_t last = 0;
    if (millis() - last > 3000) {
        last = millis();
        Serial.printf("[loop]      Stack libre Sensor: %u  Display: %u\n",
                      uxTaskGetStackHighWaterMark(hTaskSensor),
                      uxTaskGetStackHighWaterMark(hTaskDisplay));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}
