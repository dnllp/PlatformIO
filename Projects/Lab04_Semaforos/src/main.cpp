// SIN proteccion — las dos tareas pueden interrumpirse mutuamente:
//
// Core 1: Tarea OLED inicia transaccion SPI [CS_OLED=LOW] ...enviando bytes...
//           INTERRUMPIDA por scheduler
// Core 1: Tarea ETH inicia transaccion SPI  [CS_ETH=LOW]  ...enviando bytes...
//           ETH termina, OLED retoma...  bytes mezclados = pantalla corrupta
 
// CON Mutex:
// Tarea OLED: xSemaphoreTake(xSpiMutex, portMAX_DELAY)
//   → Tarea ETH llama xSemaphoreTake → BLOQUEADA hasta que OLED libere
//   Tarea OLED termina transaccion completa sin interrupcion
//   xSemaphoreGive(xSpiMutex)
//   → Tarea ETH se desbloquea y obtiene el bus
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Ethernet.h>
#include <Adafruit_BMP280.h>
 
#define OLED_MOSI 23
#define OLED_CLK  18
#define OLED_DC    2
#define OLED_CS    5
#define OLED_RST   4
#define ETH_CS    15
 
Adafruit_SSD1306 display(128,64,OLED_MOSI,OLED_CLK,OLED_DC,OLED_RST,OLED_CS);
Adafruit_BMP280  bmp;
 
// El Mutex protege el bus SPI compartido
SemaphoreHandle_t xSpiMutex = NULL;
 
// Semaforo binario para senalizar dato nuevo disponible
SemaphoreHandle_t xDataReady = NULL;
 
volatile float gTemp = 0, gPres = 0;
 
// ── Tarea: actualiza OLED (necesita SPI) ────────────────
void taskOLED(void* p) {
    uint32_t updates = 0;
    for (;;) {
        // Esperar senal de dato nuevo (semaforo binario)
        xSemaphoreTake(xDataReady, portMAX_DELAY);
 
        // Tomar Mutex antes de usar SPI
        if (xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            display.clearDisplay();
            display.setTextColor(WHITE);
            display.setTextSize(1);
            display.setCursor(0,0);
            display.printf("RTOS Mutex  #%u", ++updates);
            display.setTextSize(2);
            display.setCursor(0,18);
            display.printf("%.1f C", gTemp);
            display.setCursor(0,42);
            display.printf("%.0fhPa", gPres);
            display.display();
            xSemaphoreGive(xSpiMutex);   // Liberar SIEMPRE
        } else {
            Serial.println("[OLED] Timeout esperando Mutex SPI");
        }
    }
}
 
// ── Tarea: envio Ethernet periodico (necesita SPI) ──────
void taskEthernet(void* p) {
    byte mac[] = {0xDE,0xAD,0xBE,0xEF,0xFE,0x01};
    IPAddress ip(192,168,1,100);
    Ethernet.init(ETH_CS);
    Ethernet.begin(mac, ip);
    vTaskDelay(pdMS_TO_TICKS(2000));
    uint32_t sends = 0;
    for (;;) {
        if (xSemaphoreTake(xSpiMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            Serial.printf("[ETH] Enviando paquete #%u T=%.1f\n", ++sends, gTemp);
            // Aqui iria el envio real via Ethernet.client...
            vTaskDelay(pdMS_TO_TICKS(50));   // Simular TX
            xSemaphoreGive(xSpiMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
 
// ── Tarea: leer sensor y senalizar ──────────────────────
void taskSensor(void* p) {
    for (;;) {
        gTemp = bmp.readTemperature();
        gPres = bmp.readPressure() / 100.0f;
        // Senalizar que hay dato nuevo (semaforo binario)
        xSemaphoreGive(xDataReady);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
 
void setup() {
    Serial.begin(115200);
    Wire.begin(21,22);
    bmp.begin(0x76);
    display.begin(SSD1306_SWITCHCAPVCC);
 
    xSpiMutex = xSemaphoreCreateMutex();
    xDataReady = xSemaphoreCreateBinary();
 
    xTaskCreatePinnedToCore(taskSensor,   "Sensor",  4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskOLED,     "OLED",    4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskEthernet, "ETH",     4096, NULL, 1, NULL, 0);
}
 
void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
