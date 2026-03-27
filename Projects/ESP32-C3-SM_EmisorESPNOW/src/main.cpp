#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Reemplaza con la MAC de tu receptor
uint8_t broadcastAddress[] = {0xef, 0xef, 0xef, 0xef, 0xef, 0xef}; 

#define PIR_PIN 2
bool lastState = false;

typedef struct struct_message {
  bool alert;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) return;

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  bool currentState = digitalRead(PIR_PIN);
  if (currentState != lastState) {
    myData.alert = currentState;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    lastState = currentState;
    delay(200); // Debounce simple
  }
}
