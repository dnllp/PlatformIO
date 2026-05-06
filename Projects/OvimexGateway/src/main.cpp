/**
 * ============================================================
 *  GEOFENCE GATEWAY — LilyGO TTGO LoRa32 V2.1
 *
 *  Hardware integrado:
 *    - LoRa : SX1276 (RF96) 915 MHz
 *              CS=GPIO18, RST=GPIO23, DIO0=GPIO26
 *              SCK=GPIO5,  MOSI=GPIO27, MISO=GPIO19
 *    - OLED : SSD1306 128x64
 *              SDA=GPIO21, SCL=GPIO22
 *    - LED  : GPIO25 (activo HIGH)
 *    - Buzzer: GPIO 13
 *
 *  Pantalla OLED muestra:
 *    - Logo / nombre OVIMEX
 *    - Estado Wi-Fi y WebSocket
 *    - Lista de nodos activos con estado dentro/fuera
 *    - Contador de paquetes recibidos
 * ============================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BUZZER_PIN   13

// ── Wi-Fi ──────────────────────────────────────────────────
const char* WIFI_SSID = "TU_SSID";
const char* WIFI_PASS = "TU_PASSWORD";

// ── Servidor Python ────────────────────────────────────────
const char* WS_HOST = "192.168.1.100";   // IP de tu PC
const uint16_t WS_PORT = 8765;
const char* WS_PATH = "/gateway";

// ── LoRa SX1276 (RF96) — pines TTGO LoRa32 V2.1 ──────────
#define LORA_SCK     5
#define LORA_MISO    19
#define LORA_MOSI    27
#define LORA_CS      18
#define LORA_RST     23
#define LORA_DIO0    26
#define LORA_FREQ    915E6

// ── OLED SSD1306 128x64 ────────────────────────────────────
#define OLED_SDA     21
#define OLED_SCL     22
#define OLED_ADDR    0x3C
#define SCREEN_W     128
#define SCREEN_H     64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// ── LED indicador ──────────────────────────────────────────
#define LED_PIN      25

// ── Estado global ──────────────────────────────────────────
WebSocketsClient wsClient;
bool wsConnected  = false;
bool wifiOk       = false;
uint32_t pktCount = 0;

// ── Nodos activos (máx 5) ──────────────────────────────────
#define MAX_NODES 5
struct NodeInfo {
    char     id[16];
    float    lat, lon;
    int      rssi;
    bool     inside;        // true=dentro, false=fuera
    bool     hasGeofence;   // el servidor ya respondió al menos 1 vez
    uint32_t lastSeen;      // millis()
    bool     active;
};
NodeInfo nodes[MAX_NODES];

void updateBuzzer() {
    // Revisar si algún nodo activo está fuera
    bool anyOutside = false;
    for (int i = 0; i < MAX_NODES; i++) {
        if (!nodes[i].active) continue;
        if (!nodes[i].hasGeofence) continue;
        bool stale = (millis() - nodes[i].lastSeen) > 15000;
        if (!stale && !nodes[i].inside) { anyOutside = true; break; }
    }

    if (!anyOutside) {
        digitalWrite(BUZZER_PIN, LOW);
        return;
    }

    // Patrón 200ms ON / 200ms OFF
    static unsigned long lastToggle = 0;
    static bool state = false;
    if (millis() - lastToggle >= 200) {
        state = !state;
        digitalWrite(BUZZER_PIN, state ? HIGH : LOW);
        lastToggle = millis();
    }
}

int findOrCreateNode(const char* id) {
    for (int i = 0; i < MAX_NODES; i++)
        if (nodes[i].active && strcmp(nodes[i].id, id) == 0) return i;
    for (int i = 0; i < MAX_NODES; i++)
        if (!nodes[i].active) {
            strncpy(nodes[i].id, id, 15);
            nodes[i].active      = true;
            nodes[i].hasGeofence = false;
            nodes[i].inside      = true;
            return i;
        }
    return 0; // Si está lleno sobreescribe el primero
}

// ── Cola TX ────────────────────────────────────────────────
struct TxItem { char data[128]; };
TxItem txQueue[4];
int txHead = 0, txTail = 0;

void enqueueTx(const char* data) {
    int next = (txTail + 1) % 4;
    if (next == txHead) return;
    strncpy(txQueue[txTail].data, data, 127);
    txTail = next;
}

// ═══════════════════════════════════════════════════════════
//  OLED — Pantallas
// ═══════════════════════════════════════════════════════════

// Pantalla de splash al arrancar
void oledSplash() {
    display.clearDisplay();

    // Borde exterior
    display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
    display.drawRect(2, 2, 124, 60, SSD1306_WHITE);

    // Logo / nombre empresa
    display.setTextSize(3);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 8);
    display.print("OVIMEX");

    // Línea separadora
    display.drawLine(4, 38, 124, 38, SSD1306_WHITE);

    // Subtítulo
    display.setTextSize(1);
    display.setCursor(18, 43);
    display.print("GEOFENCE MONITOR");
    display.setCursor(30, 54);
    display.print("v1.0  915 MHz");

    display.display();
    delay(3000);
}

// Pantalla de conexión Wi-Fi
void oledConnecting(const char* ssid) {
    display.clearDisplay();
    display.drawRect(0, 0, 128, 64, SSD1306_WHITE);

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(4, 4);
    display.print("OVIMEX GATEWAY");
    display.drawLine(0, 14, 128, 14, SSD1306_WHITE);

    display.setCursor(4, 18);
    display.print("Conectando WiFi...");
    display.setCursor(4, 30);
    display.print(ssid);

    // Animación puntos
    static int dots = 0;
    display.setCursor(4, 46);
    for (int i = 0; i < (dots % 4); i++) display.print(".");
    dots++;

    display.display();
}

// Pantalla principal — estado + nodos
void oledMain() {
    display.clearDisplay();

    // ── Cabecera ──────────────────────────────────────────
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("OVIMEX");

    // Estado WS
    display.setCursor(72, 0);
    if (wsConnected) {
        display.print("[WS OK]");
    } else if (wifiOk) {
        display.print("[NO WS]");
    } else {
        display.print("[NO WIFI]");
    }

    // Contador paquetes
    display.setCursor(0, 9);
    display.print("PKT:");
    display.print(pktCount);

    // IP local
    if (wifiOk) {
        display.setCursor(50, 9);
        display.print(WiFi.localIP().toString().c_str());
    }

    // Separador
    display.drawLine(0, 18, 128, 18, SSD1306_WHITE);

    // ── Lista de nodos ────────────────────────────────────
    int y = 21;
    int activeCount = 0;

    for (int i = 0; i < MAX_NODES; i++) {
        if (!nodes[i].active) continue;

        // Marcar inactivo si no se recibió nada en 15 s
        bool stale = (millis() - nodes[i].lastSeen) > 15000;

        display.setCursor(0, y);

        // Icono de estado
        if (stale) {
            display.print("? ");
        } else if (!nodes[i].hasGeofence) {
            display.print("~ ");
        } else if (nodes[i].inside) {
            display.print("* ");   // dentro
        } else {
            // Fuera: parpadeo invertido
            bool blink = (millis() / 400) % 2;
            if (blink) {
                display.fillRect(0, y - 1, 10, 9, SSD1306_WHITE);
                display.setTextColor(SSD1306_BLACK);
                display.setCursor(0, y);
                display.print("!");
                display.setTextColor(SSD1306_WHITE);
                display.setCursor(10, y);
            } else {
                display.print("! ");
            }
        }

        // ID del nodo (máx 8 chars)
        char shortId[9];
        strncpy(shortId, nodes[i].id, 8);
        shortId[8] = '\0';
        display.print(shortId);

        // RSSI
        display.setCursor(76, y);
        display.print(nodes[i].rssi);
        display.print("dB");

        // Estado texto
        display.setCursor(104, y);
        if (stale)                  display.print("OFF");
        else if (!nodes[i].hasGeofence) display.print("---");
        else if (nodes[i].inside)   display.print("IN ");
        else                        display.print("OUT");

        y += 10;
        activeCount++;
        if (y > 58) break;
    }

    if (activeCount == 0) {
        display.setCursor(20, 32);
        display.print("Sin nodos activos");
    }

    // Separador inferior + total
    display.drawLine(0, 56, 128, 56, SSD1306_WHITE);
    display.setCursor(0, 58);
    display.print("Nodos: ");
    display.print(activeCount);
    display.setCursor(70, 58);
    display.print("915MHz RF96");

    display.display();
}

// ═══════════════════════════════════════════════════════════
//  WebSocket
// ═══════════════════════════════════════════════════════════
void onWsEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            wsConnected = true;
            digitalWrite(LED_PIN, HIGH);
            Serial.println("[WS] Conectado al servidor");
            wsClient.sendTXT("{\"type\":\"gateway\"}");
            break;

        case WStype_DISCONNECTED:
            wsConnected = false;
            digitalWrite(LED_PIN, LOW);
            Serial.println("[WS] Desconectado");
            break;

        case WStype_TEXT: {
            // Respuesta del servidor → retransmitir por LoRa al nodo
            Serial.printf("[WS→LoRa] %s\n", payload);
            enqueueTx((const char*)payload);

            // Actualizar estado del nodo en pantalla
            JsonDocument doc;
            if (deserializeJson(doc, payload) == DeserializationError::Ok) {
                if (doc.containsKey("id") && doc.containsKey("inside")) {
                    int idx = findOrCreateNode(doc["id"]);
                    nodes[idx].inside      = (doc["inside"].as<int>() == 1);
                    nodes[idx].hasGeofence = true;
                    nodes[idx].lastSeen    = millis();
                }
            }
            break;
        }
        default: break;
    }
}

// ═══════════════════════════════════════════════════════════
//  LoRa
// ═══════════════════════════════════════════════════════════
void initLoRa() {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    while (!LoRa.begin(LORA_FREQ)) {
        Serial.println("[LoRa] Error, reintentando...");
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(4, 28);
        display.print("ERROR: LoRa RF96");
        display.display();
        delay(1000);
    }

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.receive();
    Serial.println("[LoRa] RF96 OK @ 915 MHz");
}

// ── TX Queue procesado ─────────────────────────────────────
unsigned long lastTxAttempt = 0;

void processTxQueue() {
    if (txHead == txTail) return;
    if (millis() - lastTxAttempt < 150) return;
    lastTxAttempt = millis();
    LoRa.beginPacket();
    LoRa.print(txQueue[txHead].data);
    LoRa.endPacket(true);
    Serial.printf("[LoRa TX] %s\n", txQueue[txHead].data);
    txHead = (txHead + 1) % 4;
}

// ═══════════════════════════════════════════════════════════
//  Wi-Fi
// ═══════════════════════════════════════════════════════════
void connectWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] Conectando a %s", WIFI_SSID);

    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 40) {
        delay(500);
        Serial.print(".");
        oledConnecting(WIFI_SSID);
        tries++;
    }

    wifiOk = (WiFi.status() == WL_CONNECTED);
    if (wifiOk) {
        Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WiFi] Timeout — continuando sin red");
    }
}

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(300);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Inicializar OLED
    Wire.begin(OLED_SDA, OLED_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("[OLED] ERROR al inicializar");
    }
    display.setTextWrap(false);
    oledSplash();

    // Inicializar nodos
    memset(nodes, 0, sizeof(nodes));

    connectWiFi();
    initLoRa();

    if (wifiOk) {
        wsClient.begin(WS_HOST, WS_PORT, WS_PATH);
        wsClient.onEvent(onWsEvent);
        wsClient.setReconnectInterval(3000);
    }

    Serial.println("[GATEWAY] Listo");
}

// ═══════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════
unsigned long lastDisplayUpdate = 0;

void loop() {
    wsClient.loop();

    // ── Leer LoRa ─────────────────────────────────────────
    int pktSize = LoRa.parsePacket();
    if (pktSize > 0) {
        String raw = "";
        while (LoRa.available()) raw += (char)LoRa.read();
        int rssi = LoRa.packetRssi();

        Serial.printf("[LoRa RX] %s | RSSI=%d\n", raw.c_str(), rssi);
        pktCount++;

        // Parpadeo LED al recibir
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));

        // Actualizar nodo en pantalla
        JsonDocument doc;
        if (deserializeJson(doc, raw) == DeserializationError::Ok) {
            if (doc.containsKey("id")) {
                int idx = findOrCreateNode(doc["id"]);
                nodes[idx].lat      = doc["lat"] | 0.0f;
                nodes[idx].lon      = doc["lon"] | 0.0f;
                nodes[idx].rssi     = rssi;
                nodes[idx].lastSeen = millis();

                // Agregar RSSI y reenviar al servidor
                doc["rssi"] = rssi;
                char out[192];
                serializeJson(doc, out, sizeof(out));

                if (wsConnected) wsClient.sendTXT(out);
            }
        }
    }

    processTxQueue();
    updateBuzzer();

    // ── Actualizar OLED cada 500 ms ───────────────────────
    if (millis() - lastDisplayUpdate >= 500) {
        lastDisplayUpdate = millis();
        oledMain();
    }

    // ── Reconectar WiFi si cae ────────────────────────────
    if (WiFi.status() != WL_CONNECTED) {
        wifiOk = false;
        WiFi.reconnect();
        delay(1000);
    } else {
        wifiOk = true;
    }
}