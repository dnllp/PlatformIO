#include<Arduino.h>
#include <SoftwareSerial.h>

// Definimos los pines: RX del Arduino (conectar al TX del HC-06) 
// y TX del Arduino (conectar al RX del HC-06 mediante divisor de tensión)
SoftwareSerial BTSerial(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);
  Serial.println("Listo. Escribe tus comandos AT en el monitor serial:");
  
  // La velocidad por defecto del HC-06 suele ser 9600
  BTSerial.begin(9600);
}

void loop() {
  // Leer del HC-06 y mostrar en el Monitor Serial
  if (BTSerial.available()) {
    Serial.write(BTSerial.read());
  }
  
  // Leer del Monitor Serial y enviar al HC-06
  if (Serial.available()) {
    BTSerial.write(Serial.read());
  }
}
