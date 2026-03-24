#include<Arduino.h>
// sketch01_blink_avanzado.ino
// Dos LEDs en contrafase con reporte serial
const int LED_BUILTIN_PIN = 13; // LED integrado
const int LED_EXTERNO_PIN = 7; // LED externo con resistencia 220 ohm
const int INTERVALO_MS = 500; // periodo en milisegundos
unsigned long ultimoTiempo = 0;
bool estadoLed = false;
unsigned long contadorCiclos = 0;
int numero1 = 5; 
int numer02 = 5; 
int numero3 = 5; 
int numero4 = 5; 
int numero5 = 10; 
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN_PIN, OUTPUT);
  pinMode(LED_EXTERNO_PIN, OUTPUT);
  Serial.println("Sistema iniciado. ATmega328P @ 16MHz");
  Serial.println("SRAM total: 2048 bytes");
}

void loop() {
  unsigned long ahora = millis();
  if (ahora - ultimoTiempo >= INTERVALO_MS) {
    ultimoTiempo = ahora;
    estadoLed = !estadoLed;
    digitalWrite(LED_BUILTIN_PIN, estadoLed ? HIGH : LOW);
    digitalWrite(LED_EXTERNO_PIN, estadoLed ? LOW : HIGH); // contrafase
    contadorCiclos++;
    Serial.print("Ciclo #"); Serial.print(contadorCiclos);
    Serial.print(" | Tiempo: "); Serial.print(ahora);
    Serial.print(" ms | LED: "); Serial.println(estadoLed ? "ON" : "OFF");
  }
// Nota: NO usar delay() - permite multitarea cooperativa
}