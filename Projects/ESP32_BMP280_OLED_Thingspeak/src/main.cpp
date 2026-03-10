#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include "ThingSpeak.h"

/*ADVERTENCIA:
  Utiliza una fuente adicional de 3.3v para alimentar los sensores, al utilizar WiFi la ESP32 tiene picos de 
  voltaje que pueden causar el reinicio de la ESP32. 
  Solo asegurate de conectar el GND de la ESP32 al GND de la fuente de 3.3v*/

// --- CONFIGURACIÓN ---
const char* SSID = "ROBOTICA-UAM";
const char* PASS = "m4nt32024uat";
unsigned long CHANNEL_ID = 1234567;
const char* WRITE_API_KEY = "TU_API_KEY";

#define DHTPIN 18       // Pin de datos del DHT22
#define DHTTYPE DHT22  

// --- INSTANCIAS ---
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_BMP280 bmp;
DHT dht(DHTPIN, DHTTYPE);
WiFiClient client;

void setup() {
    Serial.begin(115200);
    dht.begin();
    
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for(;;);
    }
    display.clearDisplay();
    display.setTextColor(WHITE);

    if (!bmp.begin(0x76)) {
        Serial.println("Error BMP280");
        while (1);
    }

    WiFi.begin(SSID, PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    ThingSpeak.begin(client);
}

void loop() {
    // Lecturas de sensores
    float temp_bmp = bmp.readTemperature();
    float pres = bmp.readPressure() / 100.0;
    float alt  = bmp.readAltitude(1013.25);
    
    float hum = dht.readHumidity();
    float temp_dht = dht.readTemperature();

    // Validar lectura DHT
    if (isnan(hum) || isnan(temp_dht)) {
        Serial.println("Error leyendo DHT22");
        return;
    }

    // Usamos la temperatura del BMP280 por ser más precisa, pero la humedad del DHT
    // 1. Sensación Térmica (Heat Index oficial de la NOAA)
    float feelsLike = dht.computeHeatIndex(temp_bmp, hum, false);

    // 2. Punto de Rocío (Fórmula de Magnus)
    float a = 17.27; float b = 237.7;
    float gamma = ((a * temp_bmp) / (b + temp_bmp)) + log(hum/100.0);
    float dewPoint = (b * gamma) / (a - gamma);

    // --- ENVIAR A THINGSPEAK (8 CAMPOS) ---
    ThingSpeak.setField(1, temp_bmp);   // Temp BMP280
    ThingSpeak.setField(2, hum);        // Humedad Real DHT22
    ThingSpeak.setField(3, pres);       // Presión
    ThingSpeak.setField(4, feelsLike);  // Sensación Térmica
    ThingSpeak.setField(5, dewPoint);   // Punto de Rocío
    ThingSpeak.setField(6, alt);        // Altitud
    ThingSpeak.setField(7, WiFi.RSSI());// Señal WiFi
    ThingSpeak.setField(8, temp_dht);   // Temp DHT22 (para comparar)

    int status = ThingSpeak.writeFields(CHANNEL_ID, WRITE_API_KEY);

    // --- MOSTRAR EN OLED ---
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.printf("T:%.1fC | H:%.1f%%\n", temp_bmp, hum);
    display.printf("P:%.1f hPa\n", pres);
    display.printf("S.Term: %.1f C\n", feelsLike);
    display.printf("Rocio:  %.1f C\n", dewPoint);
    display.printf("Alt:    %.0fm\n", alt);
    
    display.setCursor(0, 55);
    if(status == 200) display.print("Enviado a TS OK");
    else display.print("Error TS: " + String(status));
    
    display.display();

    delay(20000); // Espera de 20s para ThingSpeak
}
