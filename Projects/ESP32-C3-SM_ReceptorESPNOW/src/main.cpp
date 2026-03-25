#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Configuración de la pantalla OLED 128x32
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1 
#define SCREEN_ADDRESS 0x3C // Dirección I2C común

// Pines I2C específicos para ESP32-C3 Super Mini
#define I2C_SDA 8
#define I2C_SCL 9

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Estructura de datos (debe ser idéntica a la del emisor)
typedef struct struct_message {
    bool alerta;
} struct_message;

struct_message datosRecibidos;

// Función que se ejecuta al recibir datos
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&datosRecibidos, incomingData, sizeof(datosRecibidos));
    
    display.clearDisplay();
    display.setCursor(0, 10);
    
    if (datosRecibidos.alerta) {
        display.setTextSize(2);
        display.println("¡ALERTA!"); // Se muestra cuando el PIR detecta movimiento
    } else {
        display.setTextSize(1);
        display.println("Sistema OK");
        display.println("Esperando...");
    }
    display.display();
}

void setup() {
    Serial.begin(115200);

    // Inicializar I2C con pines específicos
    Wire.begin(I2C_SDA, I2C_SCL);

    // Inicializar Pantalla OLED
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("Error al iniciar SSD1306"));
        for(;;);
    }
    
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Receptor ESP-NOW");
    display.display();

    // Configurar WiFi en modo Estación (necesario para ESP-NOW)
    WiFi.mode(WIFI_STA);

    // Inicializar ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error inicializando ESP-NOW");
        return;
    }

    // Registrar la función de recepción
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
    // El procesamiento ocurre en OnDataRecv
}