#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 

// Declaración de objetos
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(115200);

  // Inicializar Pantalla OLED (Dirección 0x3C común)
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 no encontrado"));
    for(;;);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println("Iniciando...");
  display.display();

  // Inicializar Sensor BMP280 (Dirección 0x76 o 0x77)
  if (!bmp.begin(0x76)) {
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("Error BMP280!");
    display.display();
    while (1);
  }
  delay(1000);
}

void loop() {
  float temp = bmp.readTemperature();
  float pres = bmp.readPressure() / 100.0;
  float alt = bmp.readAltitude(1013.25);

  // Mostrar en la pantalla OLED
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("--- DATOS BMP280 ---");

  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Temp: ");
  display.print(temp);
  display.println(" C");

  display.setCursor(0, 35);
  display.print("Pres: ");
  display.print(pres);
  display.println(" hPa");

  display.setCursor(0, 50);
  display.print("Alt:  ");
  display.print(alt);
  display.println(" m");

  display.display(); // Actualizar pantalla

  delay(2000);
}