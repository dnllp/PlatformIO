#include<Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32 // Ajustado a tu pantalla
#define OLED_RESET    -1 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  // Inicializa I2C en pines 8 (SDA) y 9 (SCL)
  Wire.begin(8, 9); 

  // Dirección I2C típica: 0x3C
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    for(;;); // Bloquea si no encuentra la pantalla
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("ESP32-C3 Mini");
  display.println("OLED 128x32 OK!");
  display.display();
}

void loop() {
  // Tu código aquí
}