#include <Arduino.h>
// LAB 9: Servidor de Comandos Serial
// Protocolo: CMD:parametro\n → respuesta JSON
#include <DHT.h>
#define DHT_PIN 2
DHT dht(DHT_PIN, DHT11);
const int LED = 13, BUZZ = 8, LDR = A0;
char buffer[64];
int bufIdx = 0;
void responderJSON(String cmd, String data) {
    Serial.print('{');
    Serial.print('"cmd":"'); Serial.print(cmd);
    Serial.print('","data":'); Serial.print(data);
    Serial.println('}');
}
void procesarComando(String cmd) {
    cmd.trim();
    if (cmd == "TEMP") {
        float t = dht.readTemperature();
        responderJSON("TEMP", isnan(t) ? "null" : String(t,1));
    } else if (cmd == "HUM") {
        float h = dht.readHumidity();
        responderJSON("HUM", isnan(h) ? "null" : String(h,1));
    } else if (cmd == "LUZ") {
        responderJSON("LUZ", String(analogRead(LDR)));
    } else if (cmd.startsWith("LED:")) {
        String val = cmd.substring(4);
        digitalWrite(LED, val == "ON" ? HIGH : LOW);
        responderJSON("LED", '"' + val + '"');
    } else if (cmd == "ALL") {
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        int l = analogRead(LDR);
        String d = '{"temp":' + String(t,1)
        + ',"hum":' + String(h,1)
        + ',"luz":' + String(l) + '}';
        responderJSON("ALL", d);
    } else {
        responderJSON("ERR", '"Comando desconocido: ' + cmd + '"');
    }
}
void setup() {
    Serial.begin(115200);
    dht.begin();
    pinMode(LED, OUTPUT);
    Serial.println('{"status":"ready","version":"1.0"}');
}
void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            buffer[bufIdx] = '\0';
            procesarComando(String(buffer));
            bufIdx = 0;
        } else if (bufIdx < 63) {
            buffer[bufIdx++] = c;
        }
    }
}