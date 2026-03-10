#include <Arduino.h>
#include <Wire.h>

void setup() {
  Wire.begin(21, 22); // SDA en GPIO 21, SCL en GPIO 22
  Serial.begin(115200);
  while (!Serial); // Esperar a que el monitor serie abra
  Serial.println("\n--- Escaneando bus I2C ---");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Escaneando...");

  for (address = 1; address < 127; address++) {
    // El i2c_scanner usa el valor de retorno de Write.endTransmission 
    // para ver si un dispositivo respondió a la dirección.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo encontrado en direccion 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identificadores comunes para tu proyecto:
      if (address == 0x28 || address == 0x29) Serial.println(" (BNO055)");
      else if (address == 0x3C || address == 0x3D) Serial.println(" (OLED SSD1306)");
      else if (address == 0x76 || address == 0x77) Serial.println(" (BMP280)");
      else Serial.println(" (Desconocido)");

      nDevices++;
    } 
    else if (error == 4) {
      Serial.print("Error desconocido en direccion 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No se encontraron dispositivos I2C\n");
  else
    Serial.println("Escaneo finalizado\n");

  delay(5000); // Esperar 5 segundos para el siguiente escaneo
}