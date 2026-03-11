#include <Arduino.h>
 
// Tarea de alta prioridad — potencialmente bloquea a las demas
void taskAlta(void* p) {
    uint64_t count = 0;
    for (;;) {
        count++;
        // SIN vTaskDelay: monopoliza el CPU continuamente
        // Con esta linea activa hay starvation de tareas de menor prio:
        // (descomenta para ver el efecto)
// if (count % 1000000 == 0) vTaskDelay(1);  // cede CPU brevemente
        vTaskDelay(pdMS_TO_TICKS(1));   // Con esto NO hay starvation
    }
}
 
// Tarea de baja prioridad — victima del starvation
void taskBaja(void* p) {
    uint32_t runs = 0;
    for (;;) {
        runs++;
        Serial.printf("[Baja] Ejecucion #%u\n", runs);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
 
// Tarea watchdog — detecta si taskBaja no reporta
void taskWatchdog(void* p) {
    extern TaskHandle_t hBaja;
    for (;;) {
        uint32_t notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
        if (notif == 0) {
            Serial.println("[WATCHDOG] ALERTA: taskBaja no respondio en 2s — STARVATION?");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
 
TaskHandle_t hBaja = NULL;
TaskHandle_t hAlta = NULL;
 
// Tarea de diagnostico — imprime stats de todas las tareas
void taskStats(void* p) {
    static char buf[512];
    for (;;) {
        vTaskList(buf);        // Requiere configUSE_TRACE_FACILITY=1
        Serial.println("\n--- ESTADO DE TAREAS ---");
        Serial.println("Nombre          Estado  Prio  Stack  Num");
        Serial.println(buf);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
 
void setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreatePinnedToCore(taskAlta,     "Alta",   4096, NULL, 5, &hAlta, 1);
    xTaskCreatePinnedToCore(taskBaja,     "Baja",   4096, NULL, 1, &hBaja, 1);
    xTaskCreatePinnedToCore(taskWatchdog, "WDog",   4096, NULL, 3, NULL,   0);
    xTaskCreatePinnedToCore(taskStats,    "Stats",  8192, NULL, 1, NULL,   0);
}
 
void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
