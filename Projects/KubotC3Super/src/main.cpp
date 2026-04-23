#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <FS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_TCS34725.h>

// --- CONFIGURACIÓN DE HARDWARE ---
const int M1A = 0; const int M1B = 1; // Motor Izquierdo
const int M2A = 2; const int M2B = 3; // Motor Derecho
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// Instancias
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Variables Globales (Compartidas entre Tareas)
float currentYaw = 0;
uint16_t r, g, b, c;
int currentPWM = 600;
String detectedColor = "Ninguno";
bool sensoresListos = false; // Variable global
// --- PROTOTIPOS ---
void TaskControl(void *pvParameters);
void TaskTelemetry(void *pvParameters);
void mover(int speedL, int speedR);

void setup() {
    Serial.begin(115200);

    unsigned long start = millis();
    while (!Serial && (millis() - start < 5000));

    Wire.begin(SDA_PIN, SCL_PIN);

    // 1. Inicializar Sistema de Archivos
    if(!LittleFS.begin(true)){ // El 'true' formatea si el sistema está dañado
        Serial.println("Error al montar LittleFS");
        return;
    }

    // 2. Inicializar Sensores
    if(bno.begin() && tcs.begin()) {
        sensoresListos = true;
        Serial.println("Sensores detectados!");
    } else {
        Serial.println("⚠️ Trabajando en modo SIN SENSORES (Simulación)");
    }

    // 3. Configurar Motores (PWM Nativo ESP32)
    analogWriteResolution(8); // 0-255

    // 4. Configurar Punto de Acceso UAT
    WiFi.softAP("Kubot_UAT_Mante", "mante2026");
    Serial.println(WiFi.softAPIP());

    // 5. Configurar Servidor Web y WebSockets
    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
        if(type == WS_EVT_DATA) {
            if(strncmp((char*)data, "calibrar", len) == 0) {
                Serial.println("Calibrando Blanco...");
                // Aquí podrías guardar factores de escala en LittleFS
            }
        }
    });
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.serveStatic("/", LittleFS, "/");
    server.begin();

    // 6. CREAR TAREAS FREERTOS
    xTaskCreate(TaskControl, "Control", 4096, NULL, 2, NULL);
    xTaskCreate(TaskTelemetry, "Telemetry", 4096, NULL, 1, NULL);
}

void loop() {
    // El loop se mantiene vacío en RTOS para evitar interferencias
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// --- TAREA 1: LÓGICA DE NAVEGACIÓN (Prioridad Alta) ---
void TaskControl(void *pvParameters) {
    for(;;) {
        // Leer IMU
        if (sensoresListos) {
            sensors_event_t event;
            bno.getEvent(&event);
            currentYaw = event.orientation.x;

            // Leer Color
            tcs.getRawData(&r, &g, &b, &c);

            // Lógica de Tarjetas (Ejemplo simplificado)
            if (r > 200 && g < 100) {
                detectedColor = "ROJO - STOP";
                mover(0, 0);
            } else if (g > 150 && r < 100) {
                detectedColor = "VERDE - GIRO";
                // Lógica de giro con BNO055 aquí
            } else {
                detectedColor = "Buscando...";
                mover(currentPWM, currentPWM);
            }
        }else {
            // MODO SEGURO: Si no hay sensores, no pedimos datos I2C
            currentYaw = 0; 
            detectedColor = "Hardware Desconectado";
            }
        vTaskDelay(pdMS_TO_TICKS(20)); // Ciclo de 50Hz
    }
}

// --- TAREA 2: TELEMETRÍA (Prioridad Baja) ---
void TaskTelemetry(void *pvParameters) {
    for(;;) {
        StaticJsonDocument<200> doc;
        doc["yaw"] = currentYaw;
        doc["r"] = r >> 8; // Convertir a 8 bits para el CSS
        doc["g"] = g >> 8;
        doc["b"] = b >> 8;
        doc["colorName"] = detectedColor;
        doc["pwm"] = currentPWM;

        String jsonString;
        serializeJson(doc, jsonString);
        ws.textAll(jsonString);

        vTaskDelay(pdMS_TO_TICKS(200)); // Refresco de 5Hz
    }
}

// --- FUNCION DE MOVIMIENTO ---
void mover(int speedL, int speedR) {
    // Motor Izquierdo
    if(speedL >= 0) { analogWrite(M1A, speedL); analogWrite(M1B, 0); }
    else { analogWrite(M1A, 0); analogWrite(M1B, abs(speedL)); }
    
    // Motor Derecho
    if(speedR >= 0) { analogWrite(M2A, speedR); analogWrite(M2B, 0); }
    else { analogWrite(M2A, 0); analogWrite(M2B, abs(speedR)); }
}