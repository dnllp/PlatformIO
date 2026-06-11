/**
 * ============================================================
 *  OVIMEX GEOFENCE — LILYGO LoRa32 V1.3  STANDALONE
 *
 *  Todo en el ESP32:
 *    - Access Point Wi-Fi  → SSID: OVIMEX-GEOFENCE  IP: 192.168.4.1
 *    - Servidor Web        → http://192.168.4.1      (sirve index.html desde LittleFS)
 *    - Servidor WebSocket  → ws://192.168.4.1:81
 *    - Lógica de geofence  → círculo y polígono
 *    - Pantalla OLED       → estado de nodos en tiempo real
 *    - LoRa RF96           → recibe posición de nodos, responde dentro/fuera
 *    - Buzzer              → alerta cuando nodo sale del geofence
 *
 *  Pines LILYGO LoRa32 V1.3:
 *    LoRa : CS=18  RST=14  DIO0=26  SCK=5  MOSI=27  MISO=19
 *    OLED : SDA=4  SCL=15  RST=16
 *    LED  : GPIO2
 *    Buzzer externo: GPIO13
 * ============================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "soc/rtc_cntl_reg.h"

// ── Access Point ───────────────────────────────────────────
#define AP_SSID     "OVIMEX-GEOFENCE"
#define AP_PASS     "ovimex2026"       // mín 8 caracteres, o "" para abierto
#define AP_IP       IPAddress(192, 168, 4, 1)
#define AP_GATEWAY  IPAddress(192, 168, 4, 1)
#define AP_SUBNET   IPAddress(255, 255, 255, 0)

// ── LoRa SX1276 RF96 ───────────────────────────────────────
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_CS     18
#define LORA_RST    14
#define LORA_DIO0   26
#define LORA_FREQ   915E6

// ── OLED SSD1306 ───────────────────────────────────────────
#define OLED_SDA    4
#define OLED_SCL    15
#define OLED_RST    16
#define OLED_ADDR   0x3C
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);

// ── Periféricos ────────────────────────────────────────────
#define LED_PIN     2
#define BUZZER_PIN  13

// ── Servidores ─────────────────────────────────────────────
AsyncWebServer  httpServer(80);
AsyncWebSocket  ws("/ws");

// ── Nodos ──────────────────────────────────────────────────
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

// ── Geofences (máx 6: global + 5 nodos) ───────────────────
#define MAX_GEO 6
struct GeoPoint { float lat, lon; };

struct Geofence {
    char     nodeId[16];
    uint8_t  type;        // 0=ninguna 1=círculo 2=polígono
    float    cLat, cLon;  // centro círculo
    float    radius;
    GeoPoint poly[20];
    uint8_t  polyCount;
    bool     active;
};
Geofence geofences[MAX_GEO];

uint32_t pktCount = 0;

// ══════════════════════════════════════════════════════════
//  GEOFENCE — Geometría
// ══════════════════════════════════════════════════════════

float haversine(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371000.0f;
    float p1 = radians(lat1), p2 = radians(lat2);
    float dp = radians(lat2 - lat1);
    float dl = radians(lon2 - lon1);
    float a  = sin(dp/2)*sin(dp/2) +
               cos(p1)*cos(p2)*sin(dl/2)*sin(dl/2);
    return 2.0f * R * atan2(sqrt(a), sqrt(1.0f - a));
}

bool pointInPolygon(float lat, float lon, GeoPoint* poly, uint8_t n) {
    bool inside = false;
    int j = n - 1;
    for (int i = 0; i < n; i++) {
        float xi = poly[i].lon, yi = poly[i].lat;
        float xj = poly[j].lon, yj = poly[j].lat;
        if (((yi > lat) != (yj > lat)) &&
            (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi))
            inside = !inside;
        j = i;
    }
    return inside;
}

// Busca geofence para un nodo (primero específica, luego global)
Geofence* findGeofence(const char* nodeId) {
    Geofence* global = nullptr;
    for (int i = 0; i < MAX_GEO; i++) {
        if (!geofences[i].active) continue;
        if (strcmp(geofences[i].nodeId, "global") == 0) global = &geofences[i];
        if (strcmp(geofences[i].nodeId, nodeId)   == 0) return &geofences[i];
    }
    return global;
}

// Retorna: 1=dentro, 0=fuera, -1=sin geofence
int checkGeofence(const char* nodeId, float lat, float lon) {
    Geofence* gf = findGeofence(nodeId);
    if (!gf) return -1;
    if (gf->type == 1) {
        return haversine(lat, lon, gf->cLat, gf->cLon) <= gf->radius ? 1 : 0;
    } else if (gf->type == 2) {
        return pointInPolygon(lat, lon, gf->poly, gf->polyCount) ? 1 : 0;
    }
    return -1;
}

// ══════════════════════════════════════════════════════════
//  NODOS
// ══════════════════════════════════════════════════════════

int findOrCreateNode(const char* id) {
    for (int i = 0; i < MAX_NODES; i++)
        if (nodes[i].active && strcmp(nodes[i].id, id) == 0) return i;
    for (int i = 0; i < MAX_NODES; i++) {
        if (!nodes[i].active) {
            memset(&nodes[i], 0, sizeof(NodeInfo));
            strncpy(nodes[i].id, id, 15);
            nodes[i].active      = true;
            nodes[i].hasGeofence = false;
            nodes[i].inside      = true;
            return i;
        }
    }
    return 0;
}

// ══════════════════════════════════════════════════════════
//  BUZZER
// ══════════════════════════════════════════════════════════

void updateBuzzer() {
    bool anyOutside = false;
    for (int i = 0; i < MAX_NODES; i++) {
        if (!nodes[i].active)      continue;
        if (!nodes[i].hasGeofence) continue;
        if ((millis() - nodes[i].lastSeen) > 15000) continue;
        if (!nodes[i].inside) { anyOutside = true; break; }
    }
    if (!anyOutside) { digitalWrite(BUZZER_PIN, LOW); return; }
    static uint32_t lastToggle = 0;
    static bool buzState = false;
    if (millis() - lastToggle >= 200) {
        buzState = !buzState;
        digitalWrite(BUZZER_PIN, buzState);
        lastToggle = millis();
    }
}

// ══════════════════════════════════════════════════════════
//  WEBSOCKET — broadcast
// ══════════════════════════════════════════════════════════

void wsBroadcast(const String& json) {
    ws.textAll(json);
}

// Construir JSON de estado completo (para init al conectar)
String buildInitJson() {
    JsonDocument doc;
    doc["type"] = "init";

    JsonObject nodesObj = doc["nodes"].to<JsonObject>();
    for (int i = 0; i < MAX_NODES; i++) {
        if (!nodes[i].active) continue;
        JsonObject n = nodesObj[nodes[i].id].to<JsonObject>();
        n["lat"]  = nodes[i].lat;
        n["lon"]  = nodes[i].lon;
        n["rssi"] = nodes[i].rssi;
        n["fix"]  = 1;
        n["spd"]  = 0;
        n["inside"] = nodes[i].hasGeofence ? (nodes[i].inside ? 1 : 0) : (int)-1;
    }

    JsonObject geoObj = doc["geofences"].to<JsonObject>();
    for (int i = 0; i < MAX_GEO; i++) {
        if (!geofences[i].active) continue;
        JsonObject g = geoObj[geofences[i].nodeId].to<JsonObject>();
        if (geofences[i].type == 1) {
            g["type"]   = "circle";
            JsonObject c = g["center"].to<JsonObject>();
            c["lat"] = geofences[i].cLat;
            c["lon"] = geofences[i].cLon;
            g["radius"] = geofences[i].radius;
        } else if (geofences[i].type == 2) {
            g["type"] = "polygon";
            JsonArray pts = g["points"].to<JsonArray>();
            for (int j = 0; j < geofences[i].polyCount; j++) {
                JsonObject p = pts.add<JsonObject>();
                p["lat"] = geofences[i].poly[j].lat;
                p["lon"] = geofences[i].poly[j].lon;
            }
        }
    }

    String out;
    serializeJson(doc, out);
    return out;
}

// ══════════════════════════════════════════════════════════
//  WEBSOCKET — handler de mensajes del browser
// ══════════════════════════════════════════════════════════

void handleWsMessage(AsyncWebSocketClient* client, const String& raw) {
    JsonDocument doc;
    if (deserializeJson(doc, raw) != DeserializationError::Ok) return;

    const char* type = doc["type"];
    if (!type) return;

    // ── set_geofence ──────────────────────────────────────
    if (strcmp(type, "set_geofence") == 0) {
        const char* nodeId = doc["node_id"] | "global";
        JsonObject  gf     = doc["geofence"];
        const char* gfType = gf["type"];

        // Buscar slot libre o existente
        int slot = -1;
        for (int i = 0; i < MAX_GEO; i++) {
            if (!geofences[i].active) { if (slot < 0) slot = i; continue; }
            if (strcmp(geofences[i].nodeId, nodeId) == 0) { slot = i; break; }
        }
        if (slot < 0) slot = 0;

        memset(&geofences[slot], 0, sizeof(Geofence));
        strncpy(geofences[slot].nodeId, nodeId, 15);
        geofences[slot].active = true;

        if (strcmp(gfType, "circle") == 0) {
            geofences[slot].type   = 1;
            geofences[slot].cLat   = gf["center"]["lat"];
            geofences[slot].cLon   = gf["center"]["lon"];
            geofences[slot].radius = gf["radius"];
        } else if (strcmp(gfType, "polygon") == 0) {
            geofences[slot].type = 2;
            JsonArray pts = gf["points"];
            int cnt = 0;
            for (JsonObject p : pts) {
                if (cnt >= 20) break;
                geofences[slot].poly[cnt].lat = p["lat"];
                geofences[slot].poly[cnt].lon = p["lon"];
                cnt++;
            }
            geofences[slot].polyCount = cnt;
        }

        Serial.printf("[GEO] Guardada para %s tipo=%s\n", nodeId, gfType);

        // Confirmar al browser
        JsonDocument resp;
        resp["type"]     = "geofence_updated";
        resp["node_id"]  = nodeId;
        resp["geofence"] = gf;
        String out;
        serializeJson(resp, out);
        wsBroadcast(out);
    }

    // ── clear_geofence ────────────────────────────────────
    else if (strcmp(type, "clear_geofence") == 0) {
        const char* nodeId = doc["node_id"] | "global";
        for (int i = 0; i < MAX_GEO; i++) {
            if (geofences[i].active && strcmp(geofences[i].nodeId, nodeId) == 0) {
                geofences[i].active = false;
            }
        }
        JsonDocument resp;
        resp["type"]    = "geofence_cleared";
        resp["node_id"] = nodeId;
        String out;
        serializeJson(resp, out);
        wsBroadcast(out);
    }
}

// ══════════════════════════════════════════════════════════
//  LORA — procesar paquete recibido
// ══════════════════════════════════════════════════════════

void processLoRaPacket(const String& raw, int rssi) {
    JsonDocument doc;
    if (deserializeJson(doc, raw) != DeserializationError::Ok) return;
    if (!doc.containsKey("id")) return;

    const char* nodeId = doc["id"];
    float lat  = doc["lat"] | 0.0f;
    float lon  = doc["lon"] | 0.0f;
    int   fix  = doc["fix"] | 0;
    float spd  = doc["spd"] | 0.0f;

    int idx = findOrCreateNode(nodeId);
    nodes[idx].lat      = lat;
    nodes[idx].lon      = lon;
    nodes[idx].rssi     = rssi;
    nodes[idx].lastSeen = millis();
    pktCount++;

    // Calcular geofence
    int result = checkGeofence(nodeId, lat, lon);
    bool hasGeo = (result >= 0);
    bool inside = (result == 1);

    nodes[idx].hasGeofence = hasGeo;
    nodes[idx].inside      = inside;

    // Responder al nodo por LoRa
    if (hasGeo) {
        JsonDocument reply;
        reply["id"]     = nodeId;
        reply["inside"] = inside ? 1 : 0;
        char buf[64];
        serializeJson(reply, buf, sizeof(buf));
        LoRa.beginPacket();
        LoRa.print(buf);
        LoRa.endPacket(true);
    }

    // Broadcast posición al browser
    JsonDocument pos;
    pos["type"]    = "position";
    pos["node_id"] = nodeId;
    pos["lat"]     = lat;
    pos["lon"]     = lon;
    pos["fix"]     = fix;
    pos["spd"]     = spd;
    pos["rssi"]    = rssi;
    pos["inside"]  = hasGeo ? (inside ? 1 : 0) : (int)-1;

    String posStr;
    serializeJson(pos, posStr);
    wsBroadcast(posStr);

    // Alerta si está fuera
    if (hasGeo && !inside) {
        JsonDocument alert;
        alert["type"]    = "alert";
        alert["node_id"] = nodeId;
        alert["message"] = String("⚠ ") + nodeId + " salió del geofence";
        String alertStr;
        serializeJson(alert, alertStr);
        wsBroadcast(alertStr);
    }

    Serial.printf("[LoRa] %s lat=%.5f lon=%.5f fix=%d rssi=%d inside=%d\n",
                  nodeId, lat, lon, fix, rssi, inside);
}

// ══════════════════════════════════════════════════════════
//  OLED
// ══════════════════════════════════════════════════════════

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
    display.setCursor(22, 54);
    display.print("AP: " AP_SSID);
    display.display();
    delay(3000);
}

void oledMain() {
    display.clearDisplay();

    // Cabecera
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("OVIMEX");
    display.setCursor(72, 0);
    display.print("192.168.4.1");
    display.setCursor(0, 9);
    display.print("PKT:");
    display.print(pktCount);
    display.setCursor(55, 9);
    display.print("AP:");
    display.print(WiFi.softAPgetStationNum());
    display.print(" client");

    display.drawLine(0, 18, 128, 18, SSD1306_WHITE);

    // Lista de nodos
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
            bool blink = (millis() / 400) % 2;
            if (blink) {
                display.fillRect(0, y-1, 10, 9, SSD1306_WHITE);
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
        if (stale)                      display.print("OFF");
        else if (!nodes[i].hasGeofence) display.print("---");
        else if (nodes[i].inside)       display.print("IN ");
        else                            display.print("OUT");

        y += 10;
        activeCount++;
        if (y > 56) break;
    }

    if (activeCount == 0) {
        display.setCursor(16, 35);
        display.print("Sin nodos activos");
    }

    display.drawLine(0, 56, 128, 56, SSD1306_WHITE);
    display.setCursor(0, 58);
    display.print("Nodos:");
    display.print(activeCount);
    display.setCursor(55, 58);
    display.print("RF96 915MHz");

    display.display();
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(300);
    Serial.println("\n=== OVIMEX GEOFENCE STANDALONE ===");

    pinMode(LED_PIN,    OUTPUT); digitalWrite(LED_PIN, LOW);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

    // ── OLED ─────────────────────────────────────────────
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);  delay(20);
    digitalWrite(OLED_RST, HIGH); delay(20);
    Wire.begin(OLED_SDA, OLED_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false))
        Serial.println("[OLED] Error");
    display.setTextWrap(false);
    oledSplash();

    // ── LittleFS ──────────────────────────────────────────
    if (!LittleFS.begin(true)) {
        Serial.println("[FS] Error LittleFS");
    } else {
        Serial.println("[FS] LittleFS OK");
    }

    // ── Access Point ──────────────────────────────────────
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[AP] SSID: %s  IP: %s\n", AP_SSID,
                  WiFi.softAPIP().toString().c_str());

    // ── WebSocket ─────────────────────────────────────────
    ws.onEvent([](AsyncWebSocket* server, AsyncWebSocketClient* client,
                  AwsEventType type, void* arg, uint8_t* data, size_t len) {
        if (type == WS_EVT_CONNECT) {
            Serial.printf("[WS] Cliente conectado id=%u\n", client->id());
            client->text(buildInitJson());
        } else if (type == WS_EVT_DISCONNECT) {
            Serial.printf("[WS] Cliente desconectado id=%u\n", client->id());
        } else if (type == WS_EVT_DATA) {
            AwsFrameInfo* info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len) {
                String msg = "";
                for (size_t i = 0; i < len; i++) msg += (char)data[i];
                handleWsMessage(client, msg);
            }
        }
    });
    httpServer.addHandler(&ws);

    // ── HTTP — servir index.html desde LittleFS ───────────
    httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(LittleFS, "/index.html", "text/html");
    });
    httpServer.serveStatic("/", LittleFS, "/");
    httpServer.onNotFound([](AsyncWebServerRequest* req) {
        req->send(404, "text/plain", "No encontrado");
    });
    httpServer.begin();
    Serial.println("[HTTP] Servidor en http://192.168.4.1");

    // ── LoRa ─────────────────────────────────────────────
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
    while (!LoRa.begin(LORA_FREQ)) {
        Serial.println("[LoRa] Error RF96...");
        delay(1000);
    }
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.receive();
    Serial.println("[LoRa] RF96 OK @ 915 MHz");

    memset(nodes,     0, sizeof(nodes));
    memset(geofences, 0, sizeof(geofences));

    Serial.println("[READY] Conéctate a: " AP_SSID);
    Serial.println("[READY] Abre: http://192.168.4.1");
}

// ══════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════

uint32_t lastOled   = 0;
uint32_t lastWsClean = 0;

void loop() {
    // Leer LoRa
    int pktSize = LoRa.parsePacket();
    if (pktSize > 0) {
        String raw = "";
        while (LoRa.available()) raw += (char)LoRa.read();
        int rssi = LoRa.packetRssi();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        processLoRaPacket(raw, rssi);
    }

    updateBuzzer();

    // OLED cada 500 ms
    if (millis() - lastOled >= 500) {
        lastOled = millis();
        oledMain();
    }

    // Limpiar clientes WS muertos cada 5 s
    if (millis() - lastWsClean >= 5000) {
        lastWsClean = millis();
        ws.cleanupClients();
    }
}
