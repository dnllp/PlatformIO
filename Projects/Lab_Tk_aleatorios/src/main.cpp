#include <Arduino.h>

// Variables globales
int n = 0; // Contador consecutivo

void setup() {
    // Iniciamos la comunicación serial a 115200 baudios
    Serial.begin(115200);
    delay(1000); // Pequeña pausa para estabilizar la conexión

    // Opcional: Imprimir el encabezado CSV
    Serial.println("N,temp,pres,hum,alt");
}

void loop() {
    // Generación de valores aleatorios (flotantes)
    // random(min, max) devuelve un entero, lo dividimos para obtener decimales
    float temp = random(1500, 3500) / 100.0;    // Rango: 15.00 - 35.00
    float pres = random(95000, 105000) / 100.0; // Rango: 950.00 - 1050.00
    float hum  = random(3000, 9000) / 100.0;    // Rango: 30.00 - 90.00
    float alt  = random(0, 500000) / 100.0;     // Rango: 0.00 - 5000.00

    // Impresión en formato CSV
    Serial.print(n);
    Serial.print(",");
    Serial.print(temp, 2); // 2 decimales
    Serial.print(",");
    Serial.print(pres, 2);
    Serial.print(",");
    Serial.print(hum, 2);
    Serial.print(",");
    Serial.println(alt, 2); // println al final para el salto de línea

    // Incrementar contador
    n++;

    // Esperar 1 segundo entre lecturas
    delay(1000);
}