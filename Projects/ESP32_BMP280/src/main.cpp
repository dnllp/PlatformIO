#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

/***********CONEXION*********/
/*
Sensor    ESP32   	Descripción
VCC       3.3V      VCC Alimentación
GND       GND       GND Tierra
SCL       GPIO 22   SCL Reloj I2C
SDA       GPIO 21   SDA Datos I2C
*/

/***********PLATFORMIO.INI*********/
/*
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    adafruit/Adafruit BMP280 Library @ ^2.6.8
    adafruit/Adafruit Unified Sensor @ ^1.1.14
    
*/

Adafruit_BMP280 bmp; // Objeto para el sensor

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando BMP280...");

  // Dirección I2C común: 0x76 o 0x77
  if (!bmp.begin(0x76)) { 
    Serial.println("No se encontró un sensor BMP280 válido. ¡Revisa el cableado!");
    while (1);
  }

  /* Configuración opcional para mayor precisión */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 
}

void loop() {
    Serial.print(F("Temperatura: "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Presión: "));
    Serial.print(bmp.readPressure() / 100); // Convertir a hPa
    Serial.println(" hPa");

    Serial.print(F("Altitud aprox: "));
    Serial.print(bmp.readAltitude(1013.25)); // Ajustar según presión local al nivel del mar
    Serial.println(" m");

    Serial.println("-----------------------");
    delay(2000);
}