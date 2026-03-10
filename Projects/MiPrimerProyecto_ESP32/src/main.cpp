#include <Arduino.h>

// Declaramos los "manejadores" (handles) de nuestras tareas
TaskHandle_t TareaIMU;
TaskHandle_t TareaLoRa;
int led = 2;
// --- 1. Definimos el código de la Tarea del IMU ---
void codigoTareaIMU(void * parameter) {
  for(;;) { // Bucle infinito de la tarea
    // Aquí iría el código de lectura I2C del MPU6050
    Serial.print("Leyendo IMU a alta velocidad en Core: ");
    Serial.println(xPortGetCoreID());
    
    // En FreeRTOS NUNCA usamos delay(). Usamos vTaskDelay()
    // Esto cede el control al procesador en lugar de congelarlo.
    vTaskDelay(10 / portTICK_PERIOD_MS); // Espera 10ms
  }
}

// --- 2. Definimos el código de la Tarea de Transmisión ---
void codigoTareaLoRa(void * parameter) {
  bool f = true;
  for(;;) {
    // Aquí iría la rutina pesada de enviar paquetes LoRa o WiFi
    Serial.print("Transmitiendo datos pesados en Core: ");
    Serial.println(xPortGetCoreID());
    digitalWrite(led,f);
    f=!f;
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Transmite cada 2 segundos
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(led,OUTPUT);

  // --- 3. Creamos las tareas y las asignamos a los núcleos ---
  
  xTaskCreatePinnedToCore(
    codigoTareaIMU,   // Función que contiene el código de la tarea
    "Tarea_IMU",      // Nombre para depuración
    10000,            // Tamaño de la pila (Stack size en bytes)
    NULL,             // Parámetros de entrada
    2,                // Prioridad (Mayor número = mayor prioridad)
    &TareaIMU,        // Manejador de la tarea
    1);               // Asignada al Core 1

  xTaskCreatePinnedToCore(
    codigoTareaLoRa,  
    "Tarea_LoRa",    
    10000,            
    NULL,             
    1,                // Prioridad menor (la transmisión puede esperar un poco)
    &TareaLoRa,       
    0);               // Asignada al Core 0 (Para no estorbar al IMU)
}

void loop() {
  // En una arquitectura RTOS profesional, el loop principal suele quedar vacío
  // o se usa para tareas de muy baja prioridad (ej. watchdog).
  vTaskDelay(1000 / portTICK_PERIOD_MS); 
}