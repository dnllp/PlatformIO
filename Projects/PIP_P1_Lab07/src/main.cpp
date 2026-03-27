// LAB 7: DHT11 - Monitor Climático con Alarmas
// Librería necesaria: DHT sensor library by Adafruit
#include <Arduino.h>
#include <DHT.h>
#define DHT_PIN 2
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
const float TEMP_ON = 30.0, TEMP_OFF = 28.0;
const float HUM_ON = 80.0, HUM_OFF = 75.0;
const int LED_TEMP = 9, LED_HUM = 10;
const int BUZZER = 8;
bool alertaTemp = false, alertaHum = false;
void setup() {
dht.begin();
pinMode(LED_TEMP, OUTPUT);
pinMode(LED_HUM, OUTPUT);
pinMode(BUZZER, OUTPUT);
Serial.begin(9600);
Serial.println("Tiempo(s),Temp(C),Hum(%),A_Temp,A_Hum");
}
void loop() {
float t = dht.readTemperature();
float h = dht.readHumidity();
if (isnan(t) || isnan(h)) {
Serial.println("Error: sensor no conectado");
delay(2000); return;
}
// Lógica con histéresis
if (!alertaTemp && t >= TEMP_ON) alertaTemp = true;
if ( alertaTemp && t <= TEMP_OFF) alertaTemp = false;
if (!alertaHum && h >= HUM_ON) alertaHum = true;
if ( alertaHum && h <= HUM_OFF) alertaHum = false;
digitalWrite(LED_TEMP, alertaTemp);
digitalWrite(LED_HUM, alertaHum);
digitalWrite(BUZZER, alertaTemp || alertaHum);
Serial.print(millis()/1000); Serial.print(",");
Serial.print(t,1); Serial.print(",");
Serial.print(h,1); Serial.print(",");
Serial.print(alertaTemp); Serial.print(",");
Serial.println(alertaHum);
delay(2000); // DHT11 necesita >= 1s entre lecturas
}