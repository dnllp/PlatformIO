#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
 
// ── Estructura de datos compartida ──────────────────────
struct SensorReading {
    float    temperature;
    float    pressure;
    float    altitude;
    uint32_t timestamp_ms;
    uint8_t  seq;
};
 
// Queue con capacidad para 5 lecturas
QueueHandle_t xSensorQueue;
 
Adafruit_BMP280 bmp;
 
#define OLED_MOSI 23
#define OLED_CLK  18
#define OLED_DC    2
#define OLED_CS    5
#define OLED_RST   4
Adafruit_SSD1306 display(128,64,OLED_MOSI,OLED_CLK,OLED_DC,OLED_RST,OLED_CS);
 
// ── Productor: lee BMP280 cada 500 ms ───────────────────
void taskProducer(void* p) {
    uint8_t seq = 0;
    for (;;) {
        SensorReading reading;
        reading.temperature  = bmp.readTemperature();
        reading.pressure     = bmp.readPressure() / 100.0f;
        reading.altitude     = bmp.readAltitude(1013.25f);
        reading.timestamp_ms = millis();
        reading.seq          = seq++;
 
        // Envia a la cola — espera max 10 ms si esta llena
        if (xQueueSend(xSensorQueue, &reading, pdMS_TO_TICKS(10)) != pdTRUE) {
            Serial.println("[Producer] AVISO: cola llena, lectura descartada");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
 
// ── Consumidor: actualiza OLED cuando hay datos ─────────
void taskConsumer(void* p) {
    SensorReading r;
    uint32_t received = 0;
    for (;;) {
        // Bloquea hasta recibir — 0 CPU mientras espera
        if (xQueueReceive(xSensorQueue, &r, portMAX_DELAY) == pdTRUE) {
            received++;
            display.clearDisplay();
            display.setTextColor(WHITE);
            display.setTextSize(1);
 
            display.setCursor(0, 0);
            display.printf("FreeRTOS Queue #%u", r.seq);
            display.drawFastHLine(0, 10, 128, WHITE);
 
            display.setTextSize(2);
            display.setCursor(0, 14);
            display.printf("%.1fC", r.temperature);
            display.setCursor(64, 14);
            display.printf("%.0f", r.pressure);
 
            display.setTextSize(1);
            display.setCursor(64, 30);
            display.print("hPa");
            display.setCursor(0, 38);
            display.printf("Alt: %.1f m", r.altitude);
            display.setCursor(0, 50);
            display.printf("t=%lums  rcv=%u", r.timestamp_ms, received);
 
            display.display();
        }
    }
}
 
// ── Monitor de la cola ───────────────────────────────────
void taskMonitor(void* p) {
    for (;;) {
        UBaseType_t waiting   = uxQueueMessagesWaiting(xSensorQueue);
        UBaseType_t available = uxQueueSpacesAvailable(xSensorQueue);
        Serial.printf("[Queue] Esperando: %u  Espacio: %u\n", waiting, available);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
 
void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    bmp.begin(0x76);
    display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay(); display.display();
 
    // Crear cola de 5 elementos tipo SensorReading
    xSensorQueue = xQueueCreate(5, sizeof(SensorReading));
 
    xTaskCreatePinnedToCore(taskProducer, "Producer", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskConsumer, "Consumer", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(taskMonitor,  "Monitor",  2048, NULL, 1, NULL, 0);
}
 
void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
