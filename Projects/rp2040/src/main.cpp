#include <Arduino.h>
#include <FreeRTOS.h> // El framework lo usa internamente para los mutex
#include <semphr.h>

// Variable compartida
volatile int contador_global = 0;
SemaphoreHandle_t xMutex;

// --- CONFIGURACIÓN PARA CORE 0 ---
void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    xMutex = xSemaphoreCreateMutex();
}

void loop() {
    // Tarea del Core 0: Parpadear LED e imprimir valor
    digitalWrite(LED_BUILTIN, HIGH);
    
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        Serial.printf("[Core 0] Leyendo contador: %d\n", contador_global);
        xSemaphoreGive(xMutex);
    }
    
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}

// --- CONFIGURACIÓN PARA CORE 1 (Ejecución paralela) ---
void setup1() {
    // No requiere configuración especial, el Core 0 ya inició el hardware
}

void loop1() {
    // Tarea del Core 1: Incremento constante con protección de Mutex
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        contador_global++;
        xSemaphoreGive(xMutex);
    }
    
    // Simulamos una carga de trabajo pesada
    delay(10); 
}