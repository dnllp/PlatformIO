#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// El LED RGB integrado en la mayoría de las placas ESP32-S3 está en el GPIO 48
#define PIN_RGB        48 
#define NUM_PIXELS     1  // Solo hay 1 LED integrado

// Configuración del objeto NeoPixel
Adafruit_NeoPixel led_interno(NUM_PIXELS, PIN_RGB, NEO_GRB + NEO_KHZ800);

void setup() {
  led_interno.begin();           // Inicializa el LED
  led_interno.setBrightness(50); // Brillo de 0 a 255 (no lo pongas al máximo, brilla mucho)
}

void loop() {
  // Rojo
  led_interno.setPixelColor(0, led_interno.Color(255, 0, 0));
  led_interno.show();
  delay(1000);

  // Verde
  led_interno.setPixelColor(0, led_interno.Color(0, 255, 0));
  led_interno.show();
  delay(1000);

  // Azul
  led_interno.setPixelColor(0, led_interno.Color(0, 0, 255));
  led_interno.show();
  delay(1000);
}
