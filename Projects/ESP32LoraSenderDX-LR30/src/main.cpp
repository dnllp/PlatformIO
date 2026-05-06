#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// Definición de pines para ESP32 Estándar
// NSS: 5, DIO1: 2, RST: 14, BUSY: 32
SX1262 radio = new Module(5, 2, 14, 32);

void setup() {
  Serial.begin(115200);
  while (!Serial); 

  Serial.println(F("[DX-LR30] Iniciando bus SPI..."));
  // Iniciamos SPI con pines estándar del ESP32
  SPI.begin(18, 19, 23, 5); 

  Serial.print(F("[DX-LR30] Inicializando módulo a 915MHz... "));
  // Inicialización básica
  int state = radio.begin(915.0);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("¡Éxito!"));

    // --- CONFIGURACIÓN CRÍTICA DE DIO2 ---
    // Esta línea le dice al SX1262 que use su pin DIO2 para 
    // alimentar el switch de antena automáticamente.
    state = radio.setDio2AsRfSwitch();

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("[DIO2] Control de antena automático activado."));
    } else {
      Serial.print(F("[DIO2] Error al configurar, código: "));
      Serial.println(state);
    }
    // -------------------------------------

  } else {
    Serial.print(F("Fallo al iniciar hardware, código: "));
    Serial.println(state);
    while (true); 
  }
}

void loop() {
  Serial.print(F("Enviando mensaje... "));
  
  // Transmitimos un paquete
  int state = radio.transmit("Prueba LoRa con DIO2");

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("¡Enviado!"));
  } else {
    Serial.print(F("Error de transmisión: "));
    Serial.println(state);
  }

  delay(5000); // Esperar 5 segundos
}
