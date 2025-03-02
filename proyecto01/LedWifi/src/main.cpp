#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "TuSSID"; // Replace con tu SSID de Wi-Fi
const char* password = "pass"; // replace con tu password de Wi-Fi

WebServer server(80);
const int ledPin = 2;

void handleRoot() {
  String html = "<html><body><h1>Control de LED</h1>";
  html += "<a href='/led/on'><button>Turn On LED</button></a><br>";
  html += "<a href='/led/off'><button>Turn Off LED</button></a>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleLedOn() {
  digitalWrite(ledPin, HIGH);
  server.send(200, "text/plain", "LED On");
}

void handleLedOff() {
  digitalWrite(ledPin, LOW);
  server.send(200, "text/plain", "LED Off");
}

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting  to Wi-Fi...");
  }

  Serial.println("Connected a Wi-Fi");
  Serial.print("Addres IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/led/on", handleLedOn);
  server.on("/led/off", handleLedOff);
  server.begin();
}

void loop() {
  server.handleClient();
}
