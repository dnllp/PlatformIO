#include <Arduino.h>
// LAB 8: Display 7 Segmentos - Contador con Multiplexación
// Pines de segmentos a, b, c, d, e, f, g
const int SEG[] = {2, 3, 4, 5, 6, 7, 8};
// Pines de selección de dígito (cátodo común → transistor NPN)
const int DIG[] = {9, 10};
// Tabla de segmentos para dígitos 0-9 (cátodo común)
const byte TABLA[] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111 // 9
};
int contador = 0;
unsigned long ultimoTick = 0;
void mostrarDigito(int digito, int valor) {
    // Apagar todos los dígitos
    for(int i = 0; i < 2; i++) digitalWrite(DIG[i], HIGH);
      byte seg = TABLA[valor];
    for(int s = 0; s < 7; s++)
      digitalWrite(SEG[s], (seg >> s) & 1);
    // Activar el dígito seleccionado
    digitalWrite(DIG[digito], LOW);
    delayMicroseconds(2000); // 2ms por dígito = 50Hz total
}
void setup() {
    for(int i = 0; i < 7; i++) pinMode(SEG[i], OUTPUT);
    for(int i = 0; i < 2; i++) { pinMode(DIG[i], OUTPUT); digitalWrite(DIG[i],
    HIGH); }
    Serial.begin(9600);
}
void loop() {
    // Mostrar decenas y unidades
    mostrarDigito(0, contador / 10);
    mostrarDigito(1, contador % 10);
    // Incrementar cada segundo sin usar delay()
    if (millis() - ultimoTick >= 1000) {
        ultimoTick = millis();
        contador = (contador + 1) % 100;
        Serial.println(contador);
    }
}