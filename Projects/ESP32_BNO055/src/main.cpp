#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// El sensor usa la dirección 0x28 por defecto (o 0x29 si el pin ADR está en alto)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

void setup() {
  Serial.begin(115200);
  Serial.println("Prueba del Sensor de Orientación BNO055");

  /* Inicializar el sensor */
  if(!bno.begin()) {
    Serial.print("¡No se detectó el BNO055! Revisa las conexiones o la dirección I2C.");
    while(1);
  }

  delay(1000);
  bno.setExtCrystalUse(true); // Usar cristal externo para mayor precisión
}

void loop() {
  /* Obtener evento de orientación */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Mostrar los ángulos de Euler (grados) */
  Serial.print("X (Yaw): ");
  Serial.print(event.orientation.x);
  Serial.print("\tY (Pitch): ");
  Serial.print(event.orientation.y);
  Serial.print("\tZ (Roll): ");
  Serial.print(event.orientation.z);

  /* Mostrar estado de calibración (0=No calitrado, 3=Totalmente calibrado) */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  Serial.print("\t CAL: S:");
  Serial.print(system);
  Serial.print(" G:");
  Serial.print(gyro);
  Serial.print(" A:");
  Serial.print(accel);
  Serial.print(" M:");
  Serial.println(mag);

  delay(100); // Frecuencia de actualización
}
