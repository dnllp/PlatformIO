/**
 * ============================================================
 *  GEOFENCE GATEWAY — LILYGO LoRa32 V1.3
 *
 *  Pines internos de la placa:
 *
 *  LoRa SX1276 (RF96):
 *    CS   = GPIO18
 *    RST  = GPIO14   ← diferente al V2.1 (que usa GPIO23)
 *    DIO0 = GPIO26
 *    SCK  = GPIO5
 *    MOSI = GPIO27
 *    MISO = GPIO19
 *
 *  OLED SSD1306 128x64:
 *    SDA  = GPIO4    ← diferente al V2.1 (que usa GPIO21)
 *    SCL  = GPIO15   ← diferente al V2.1 (que usa GPIO22)
 *    RST  = GPIO16   ← pin de reset por software de la OLED
 *
 *  LED  = GPIO2  (activo HIGH, LED azul integrado)
 *
 *  Buzzer externo:
 *    +    = GPIO13
 *    -    = GND
 *    (usa transistor NPN si el buzzer consume >40mA)
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

// ── Wi-Fi ──────────────────────────────────────────────────
const char* WIFI_SSID = "UAM-ROBOTICA"; 
const char* WIFI_PASS = "m4nt32024uat";

// ── Servidor Python ────────────────────────────────────────
const char* WS_HOST  = "192.168.200.106";  // IP de tu PC
const uint16_t WS_PORT = 8765;
const char* WS_PATH  = "/gateway";

// ── LoRa SX1276 — pines LILYGO LoRa32 V1.3 ────────────────
#define LORA_SCK     5
#define LORA_MISO    19
#define LORA_MOSI    27
#define LORA_CS      18
#define LORA_RST     14    // ← V1.3 usa GPIO14 (V2.1 usa GPIO23)
#define LORA_DIO0    26
#define LORA_FREQ    915E6

// ── OLED SSD1306 — pines LILYGO LoRa32 V1.3 ───────────────
#define OLED_SDA     4     // ← V1.3 usa GPIO4  (V2.1 usa GPIO21)
#define OLED_SCL     15    // ← V1.3 usa GPIO15 (V2.1 usa GPIO22)
#define OLED_RST     16    // ← V1.3 tiene reset por software
#define OLED_ADDR    0x3C
#define SCREEN_W     128
#define SCREEN_H     64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RST);

// ── LED y Buzzer ───────────────────────────────────────────
#define LED_PIN      2     // LED azul integrado
#define BUZZER_PIN   13    // Buzzer externo con NPN

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
    bool     inside;
    bool     hasGeofence;
    uint32_t lastSeen;
    bool     active;
};
NodeInfo nodes[MAX_NODES];

int findOrCreateNode(const char* id) {
    for (int i = 0; i < MAX_NODES; i++)
        if (nodes[i].active && strcmp(nodes[i].id, id) == 0) return i;
    for (int i = 0; i < MAX_NODES; i++)
        if (!nodes[i].active) {
            strncpy(nodes[i].id, id, 15);
            nodes[i].id[15]      = '\0';
            nodes[i].active      = true;
            nodes[i].hasGeofence = false;
            nodes[i].inside      = true;
            return i;
        }
    return 0;
}

// ── Cola TX ────────────────────────────────────────────────
struct TxItem { char data[128]; };
TxItem txQueue[4];
int txHead = 0, txTail = 0;

void enqueueTx(const char* data) {
    int next = (txTail + 1) % 4;
    if (next == txHead) return;
    strncpy(txQueue[txTail].data, data, 127);
    txQueue[txTail].data[127] = '\0';
    txTail = next;
}

// ═══════════════════════════════════════════════════════════
//  BUZZER
// ═══════════════════════════════════════════════════════════
void updateBuzzer() {
    bool anyOutside = false;
    for (int i = 0; i < MAX_NODES; i++) {
        if (!nodes[i].active)      continue;
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
    static bool buzzerState = false;
    if (millis() - lastToggle >= 200) {
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
        lastToggle = millis();
    }
}

// ═══════════════════════════════════════════════════════════
//  OLED — Pantallas
// ═══════════════════════════════════════════════════════════
void oledSplash() {
    display.clearDisplay();
    display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
    display.drawRect(2, 2, 124, 60, SSD1306_WHITE);

    display.setTextSize(3);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 8);
    display.print("OVIMEX");

    display.drawLine(4, 38, 124, 38, SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(18, 43);
    display.print("GEOFENCE MONITOR");
    display.setCursor(30, 54);
    display.print("v1.0  915 MHz");

    display.display();
    delay(3000);
}

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

    static int dots = 0;
    display.setCursor(4, 46);
    for (int i = 0; i < (dots % 4); i++) display.print(".");
    dots++;

    display.display();
}

void oledMain() {
    display.clearDisplay();

    // ── Cabecera ──────────────────────────────────────────
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("OVIMEX");

    display.setCursor(72, 0);
    if (wsConnected)       display.print("[WS OK]");
    else if (wifiOk)       display.print("[NO WS]");
    else                   display.print("[NO WIFI]");

    display.setCursor(0, 9);
    display.print("PKT:");
    display.print(pktCount);

    if (wifiOk) {
        display.setCursor(50, 9);
        String ip = WiFi.localIP().toString();
        display.print(ip.substring(ip.lastIndexOf('.') + 1)); // solo último octeto
        display.print(" ...");
    }

    display.drawLine(0, 18, 128, 18, SSD1306_WHITE);

    // ── Lista de nodos ────────────────────────────────────
    int y = 21;
    int activeCount = 0;

    for (int i = 0; i < MAX_NODES; i++) {
        if (!nodes[i].active) continue;
        bool stale = (millis() - nodes[i].lastSeen) > 15000;

        display.setCursor(0, y);

        if (stale) {
            display.print("? ");
        } else if (!nodes[i].hasGeofence) {
            display.print("~ ");
        } else if (nodes[i].inside) {
            display.print("* ");
        } else {
            // Parpadeo invertido para alerta
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

        char shortId[9];
        strncpy(shortId, nodes[i].id, 8);
        shortId[8] = '\0';
        display.print(shortId);

        display.setCursor(76, y);
        display.print(nodes[i].rssi);
        display.print("dB");

        display.setCursor(104, y);
        if (stale)                       display.print("OFF");
        else if (!nodes[i].hasGeofence)  display.print("---");
        else if (nodes[i].inside)        display.print("IN ");
        else                             display.print("OUT");

        y += 10;
        activeCount++;
        if (y > 56) break;
    }

    if (activeCount == 0) {
        display.setCursor(20, 32);
        display.print("Sin nodos activos");
    }

    display.drawLine(0, 56, 128, 56, SSD1306_WHITE);
    display.setCursor(0, 58);
    display.print("Nodos:");
    display.print(activeCount);
    display.setCursor(60, 58);
    display.print("RF96 915MHz");

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
            Serial.println("[WS] Conectado");
            wsClient.sendTXT("{\"type\":\"gateway\"}");
            break;

        case WStype_DISCONNECTED:
            wsConnected = false;
            digitalWrite(LED_PIN, LOW);
            Serial.println("[WS] Desconectado");
            break;

        case WStype_TEXT: {
            Serial.printf("[WS→LoRa] %s\n", payload);
            enqueueTx((const char*)payload);

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
        Serial.println("[LoRa] Error RF96, reintentando...");
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
    if (wifiOk)
        Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    else
        Serial.println("\n[WiFi] Timeout — continuando sin red");
}

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(300);

    pinMode(LED_PIN,    OUTPUT); digitalWrite(LED_PIN, LOW);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

    // Reset OLED por software (necesario en V1.3)
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);
    delay(20);

    // Inicializar OLED con pines correctos del V1.3
    Wire.begin(OLED_SDA, OLED_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false)) {
        Serial.println("[OLED] ERROR al inicializar");
    }
    display.setTextWrap(false);
    oledSplash();

    memset(nodes, 0, sizeof(nodes));

    connectWiFi();
    initLoRa();

    if (wifiOk) {
        wsClient.begin(WS_HOST, WS_PORT, WS_PATH);
        wsClient.onEvent(onWsEvent);
        wsClient.setReconnectInterval(3000);
    }

    Serial.println("[GATEWAY V1.3] Listo");
}

// ═══════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════
unsigned long lastDisplayUpdate = 0;

void loop() {
    wsClient.loop();

    // Leer LoRa
    int pktSize = LoRa.parsePacket();
    if (pktSize > 0) {
        String raw = "";
        while (LoRa.available()) raw += (char)LoRa.read();
        int rssi = LoRa.packetRssi();

        Serial.printf("[LoRa RX] %s | RSSI=%d\n", raw.c_str(), rssi);
        pktCount++;

        // Parpadeo LED
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));

        JsonDocument doc;
        if (deserializeJson(doc, raw) == DeserializationError::Ok) {
            if (doc.containsKey("id")) {
                int idx = findOrCreateNode(doc["id"]);
                nodes[idx].lat      = doc["lat"] | 0.0f;
                nodes[idx].lon      = doc["lon"] | 0.0f;
                nodes[idx].rssi     = rssi;
                nodes[idx].lastSeen = millis();

                doc["rssi"] = rssi;
                char out[192];
                serializeJson(doc, out, sizeof(out));
                if (wsConnected) wsClient.sendTXT(out);
            }
        }
    }

    processTxQueue();
    updateBuzzer();

    // Actualizar OLED cada 500 ms
    if (millis() - lastDisplayUpdate >= 500) {
        lastDisplayUpdate = millis();
        oledMain();
    }

    // Reconectar WiFi
    if (WiFi.status() != WL_CONNECTED) {
        wifiOk = false;
        WiFi.reconnect();
        delay(1000);
    } else {
        wifiOk = true;
    }
}