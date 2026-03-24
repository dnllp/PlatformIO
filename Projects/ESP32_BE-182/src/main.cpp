#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// Pines UART2 del ESP32
#define GPS_RX_PIN 16   // GPIO16 <- TX del BE-182
#define GPS_TX_PIN 17   // GPIO17 -> RX del BE-182
#define GPS_BAUD   9600

TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART2

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("BE-182 GPS iniciado...");
}

void loop() {
  // Leer datos del GPS y alimentar al parser
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Mostrar datos cada segundo
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    if (gps.location.isValid()) {
      Serial.print("Lat: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print("  Lon: ");
      Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("Buscando señal GPS...");
    }

    if (gps.altitude.isValid()) {
      Serial.print("Altitud: ");
      Serial.print(gps.altitude.meters());
      Serial.println(" m");
    }

    if (gps.speed.isValid()) {
      Serial.print("Velocidad: ");
      Serial.print(gps.speed.kmph());
      Serial.println(" km/h");
    }

    if (gps.date.isValid() && gps.time.isValid()) {
      Serial.printf("Fecha/Hora UTC: %02d/%02d/%04d %02d:%02d:%02d\n",
        gps.date.day(), gps.date.month(), gps.date.year(),
        gps.time.hour(), gps.time.minute(), gps.time.second());
    }

    Serial.print("Satélites: ");
    Serial.println(gps.satellites.isValid() ? gps.satellites.value() : 0);
    Serial.println("---");
  }
}