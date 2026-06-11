/**
 * ============================================================
 * TCS34725 + LED RGB + Botón — Calibración y Detección
 * Plataforma : PlatformIO + Arduino (ESP32)
 * Biblioteca : Adafruit TCS34725
 * ============================================================
 *
 * Conexión I2C (ESP32 30 pines):
 *   TCS34725  -->  ESP32
 *   VIN       -->  3.3V
 *   GND       -->  GND
 *   SDA       -->  GPIO 21
 *   SCL       -->  GPIO 22
 *
 * LED RGB (cátodo común):
 *   R         -->  GPIO 25  (LEDC canal 0)
 *   G         -->  GPIO 26  (LEDC canal 1)
 *   B         -->  GPIO 27  (LEDC canal 2)
 *   COM       -->  GND
 *
 * Botón:
 *   Un extremo -->  GPIO 4  (INPUT_PULLUP interno)
 *   Otro       -->  GND
 *
 * ── Modos de operación ──────────────────────────────────────
 *  ESPERA      → LED apagado, aguarda acción del botón
 *  CALIBRACIÓN → Mantener botón 2 s para entrar
 *                LED ROJO  : listo para leer color N
 *                (poner tarjeta, 5 s / 20 lecturas promediadas)
 *                LED AZUL  : color guardado, preparar siguiente
 *                LED VERDE : 8 colores guardados, fin calibración
 *  DETECCIÓN   → Pulsar botón una vez para entrar
 *                LED parpadea con el color más cercano calibrado
 *                LED apagado si no hay coincidencia
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// ── Pines ─────────────────────────────────────────────────────
#define PIN_LED_R    25
#define PIN_LED_G    26
#define PIN_LED_B    27
#define PIN_BOTON     4

// ── LEDC (PWM ESP32) ──────────────────────────────────────────
#define LEDC_FREQ    5000
#define LEDC_RES        8   // 8 bits → 0–255
#define LEDC_CH_R       0
#define LEDC_CH_G       1
#define LEDC_CH_B       2

// ── LED ánodo/cátodo común ────────────────────────────────────
#define LED_ANODO_COMUN  0   // Cambiar a 1 si es ánodo común

// ── Calibración ───────────────────────────────────────────────
#define NUM_COLORES       8
#define LECTURAS_CAL     20       // muestras por color
#define INTERVALO_CAL   250       // ms entre lecturas
#define MS_HOLD_CAL    2000       // ms para entrar en calibración
#define MS_PAUSA_CAL   1500       // ms de pausa entre colores (LED azul)

// ── Detección ─────────────────────────────────────────────────
#define UMBRAL_DISTANCIA  60.0f   // distancia euclidiana máxima
#define PARPADEO_MS       150     // ms por semiciclo de parpadeo

// ── Sensor ───────────────────────────────────────────────────
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_GAIN_4X
);

// ── Estado de la máquina ──────────────────────────────────────
enum Estado { ESPERA, CALIBRACION, DETECCION };
Estado estadoActual = ESPERA;

// ── Almacenamiento de colores calibrados (RAM) ────────────────
struct ColorRGB { uint8_t r, g, b; };
ColorRGB coloresCal[NUM_COLORES];
uint8_t  totalCalibrados = 0;

// ── Variables de parpadeo (detección) ────────────────────────
uint8_t  ledR_det = 0, ledG_det = 0, ledB_det = 0;
bool     ledEncendido   = false;
uint32_t tUltimoParpadeo = 0;

// ── Prototipos ────────────────────────────────────────────────
void  iniciarLEDC();
void  setLed(uint8_t r, uint8_t g, uint8_t b);
void  apagarLed();
void  secuenciaBienvenida();
void  manejarBoton();
void  modoCalibracion();
void  modoDeteccion();
ColorRGB leerColorPromedio();
int8_t   buscarColorMasCercano(ColorRGB muestra);
float    distanciaRGB(ColorRGB a, ColorRGB b);
void     imprimirColor(const char* etiqueta, ColorRGB c);
void     escanearI2C();

// ── Botón ─────────────────────────────────────────────────────
uint32_t tBotonPresionado = 0;
bool     botonAnterior    = HIGH;

// =============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("\n==========================================="));
    Serial.println(F("  TCS34725 | Calibracion + Deteccion"));
    Serial.println(F("===========================================\n"));

    iniciarLEDC();
    secuenciaBienvenida();

    pinMode(PIN_BOTON, INPUT_PULLUP);

    escanearI2C();

    if (!tcs.begin()) {
        Serial.println(F("[ERROR] TCS34725 no encontrado."));
        while (true) {
            setLed(255, 0, 0); delay(300);
            apagarLed();       delay(300);
        }
    }
    Serial.println(F("[OK] TCS34725 listo. Direccion I2C: 0x29"));
    Serial.println(F("\n── Instrucciones ──────────────────────────"));
    Serial.println(F("  Mantener boton 2 s  → modo CALIBRACION"));
    Serial.println(F("  Pulsar boton 1 vez  → modo DETECCION"));
    Serial.println(F("───────────────────────────────────────────\n"));
}

// =============================================================
void loop() {
    manejarBoton();

    if (estadoActual == DETECCION) {
        // Parpadeo no bloqueante del color detectado
        uint32_t ahora = millis();
        if (ahora - tUltimoParpadeo >= PARPADEO_MS) {
            tUltimoParpadeo = ahora;
            if (ledEncendido) {
                apagarLed();
            } else {
                setLed(ledR_det, ledG_det, ledB_det);
            }
            ledEncendido = !ledEncendido;
        }

        // Leer sensor continuamente y buscar coincidencia
        static uint32_t tUltimaLectura = 0;
        if (ahora - tUltimaLectura >= 300) {
            tUltimaLectura = ahora;

            uint16_t r, g, b, c;
            tcs.getRawData(&r, &g, &b, &c);
            if (c == 0) return;

            ColorRGB muestra = {
                (uint8_t)constrain((float)r / c * 255, 0, 255),
                (uint8_t)constrain((float)g / c * 255, 0, 255),
                (uint8_t)constrain((float)b / c * 255, 0, 255)
            };

            int8_t idx = buscarColorMasCercano(muestra);

            if (idx >= 0) {
                ledR_det = coloresCal[idx].r;
                ledG_det = coloresCal[idx].g;
                ledB_det = coloresCal[idx].b;
                Serial.print(F("Detectado: Color #"));
                Serial.print(idx + 1);
                Serial.print(F("  -->  "));
                imprimirColor("", coloresCal[idx]);
            } else {
                ledR_det = 0; ledG_det = 0; ledB_det = 0;
                Serial.println(F("Detectado: sin coincidencia"));
            }
        }
    }
}

// =============================================================
// Lee el estado del botón y determina si fue pulsación corta
// (→ DETECCIÓN) o larga ≥2 s (→ CALIBRACIÓN)
void manejarBoton() {
    bool botonActual = digitalRead(PIN_BOTON);

    // Flanco de bajada: botón presionado
    if (botonAnterior == HIGH && botonActual == LOW) {
        tBotonPresionado = millis();
    }

    // Flanco de subida: botón soltado
    if (botonAnterior == LOW && botonActual == HIGH) {
        uint32_t duracion = millis() - tBotonPresionado;

        if (duracion >= MS_HOLD_CAL) {
            // Pulsación larga → calibración
            Serial.println(F("\n[BOTON] Pulsacion larga → CALIBRACION"));
            estadoActual = CALIBRACION;
            modoCalibracion();
        } else if (duracion >= 50) {
            // Pulsación corta → detección (solo si hay colores calibrados)
            if (totalCalibrados == 0) {
                Serial.println(F("[BOTON] Sin colores calibrados. Calibra primero."));
                // Avisar con 3 destellos amarillos
                for (uint8_t i = 0; i < 3; i++) {
                    setLed(255, 180, 0); delay(150);
                    apagarLed();         delay(150);
                }
            } else {
                Serial.println(F("\n[BOTON] Pulsacion corta → DETECCION"));
                estadoActual    = DETECCION;
                ledR_det = 0; ledG_det = 0; ledB_det = 0;
                ledEncendido    = false;
                tUltimoParpadeo = millis();
                Serial.print(F("Detectando entre "));
                Serial.print(totalCalibrados);
                Serial.println(F(" colores calibrados..."));
            }
        }
    }

    botonAnterior = botonActual;
}

// =============================================================
// Flujo completo de calibración (bloqueante durante la sesión)
void modoCalibracion() {
    totalCalibrados = 0;
    Serial.println(F("\n════ MODO CALIBRACION ════"));
    Serial.print(F("Se capturaran "));
    Serial.print(NUM_COLORES);
    Serial.println(F(" colores. Mantener cada tarjeta bajo el sensor."));

    for (uint8_t i = 0; i < NUM_COLORES; i++) {
        // ── Señal: listo para leer (LED rojo) ────────────────
        Serial.print(F("\nColor #"));
        Serial.print(i + 1);
        Serial.println(F(" — Coloca la tarjeta y espera 5 segundos..."));
        setLed(255, 0, 0);
        delay(1000);   // Pausa de 1 s para que el usuario posicione la tarjeta

        // ── Tomar 20 lecturas cada 250 ms ────────────────────
        uint32_t sumR = 0, sumG = 0, sumB = 0;
        for (uint8_t j = 0; j < LECTURAS_CAL; j++) {
            uint16_t r, g, b, c;
            tcs.getRawData(&r, &g, &b, &c);
            if (c > 0) {
                sumR += (uint32_t)constrain((float)r / c * 255, 0, 255);
                sumG += (uint32_t)constrain((float)g / c * 255, 0, 255);
                sumB += (uint32_t)constrain((float)b / c * 255, 0, 255);
            }
            Serial.print(F("."));
            delay(INTERVALO_CAL);
        }
        Serial.println();

        coloresCal[i] = {
            (uint8_t)(sumR / LECTURAS_CAL),
            (uint8_t)(sumG / LECTURAS_CAL),
            (uint8_t)(sumB / LECTURAS_CAL)
        };
        totalCalibrados++;

        Serial.print(F("Guardado #"));
        Serial.print(i + 1);
        Serial.print(F(": "));
        imprimirColor("", coloresCal[i]);

        // ── Señal: guardado (LED azul) ────────────────────────
        setLed(0, 0, 255);
        delay(MS_PAUSA_CAL);
    }

    // ── Señal: calibración completa (LED verde) ───────────────
    setLed(0, 255, 0);
    Serial.println(F("\n[OK] Calibracion completa. 8 colores guardados."));
    Serial.println(F("     Pulsa el boton para iniciar deteccion.\n"));
    delay(2000);
    apagarLed();
    estadoActual = ESPERA;
}

// =============================================================
// Devuelve el índice del color calibrado más cercano,
// o -1 si ninguno supera el umbral de distancia
int8_t buscarColorMasCercano(ColorRGB muestra) {
    float   menorDist = UMBRAL_DISTANCIA;
    int8_t  idx       = -1;

    for (uint8_t i = 0; i < totalCalibrados; i++) {
        float d = distanciaRGB(muestra, coloresCal[i]);
        if (d < menorDist) {
            menorDist = d;
            idx       = i;
        }
    }
    return idx;
}

// =============================================================
// Distancia euclidiana en espacio RGB
float distanciaRGB(ColorRGB a, ColorRGB b) {
    float dr = (float)a.r - b.r;
    float dg = (float)a.g - b.g;
    float db = (float)a.b - b.b;
    return sqrt(dr*dr + dg*dg + db*db);
}

// =============================================================
void imprimirColor(const char* etiqueta, ColorRGB c) {
    if (etiqueta && etiqueta[0] != '\0') {
        Serial.print(etiqueta);
        Serial.print(F(": "));
    }
    Serial.print(F("R:"));  Serial.print(c.r);
    Serial.print(F(" G:")); Serial.print(c.g);
    Serial.print(F(" B:")); Serial.println(c.b);
}

// =============================================================
void iniciarLEDC() {
    ledcSetup(LEDC_CH_R, LEDC_FREQ, LEDC_RES);
    ledcSetup(LEDC_CH_G, LEDC_FREQ, LEDC_RES);
    ledcSetup(LEDC_CH_B, LEDC_FREQ, LEDC_RES);
    ledcAttachPin(PIN_LED_R, LEDC_CH_R);
    ledcAttachPin(PIN_LED_G, LEDC_CH_G);
    ledcAttachPin(PIN_LED_B, LEDC_CH_B);
    apagarLed();
    Serial.println(F("[OK] LED RGB LEDC listo (pines 25/26/27)."));
}

void setLed(uint8_t r, uint8_t g, uint8_t b) {
#if LED_ANODO_COMUN
    ledcWrite(LEDC_CH_R, 255 - r);
    ledcWrite(LEDC_CH_G, 255 - g);
    ledcWrite(LEDC_CH_B, 255 - b);
#else
    ledcWrite(LEDC_CH_R, r);
    ledcWrite(LEDC_CH_G, g);
    ledcWrite(LEDC_CH_B, b);
#endif
}

void apagarLed() { setLed(0, 0, 0); }

void secuenciaBienvenida() {
    setLed(255, 0, 0); delay(250);
    setLed(0, 255, 0); delay(250);
    setLed(0, 0, 255); delay(250);
    apagarLed();
}

// =============================================================
void escanearI2C() {
    Serial.println(F("[I2C] Escaneando bus..."));
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("  0x"));
            if (addr < 16) Serial.print(F("0"));
            Serial.print(addr, HEX);
            if (addr == 0x29) Serial.print(F("  <-- TCS34725"));
            Serial.println();
            found++;
        }
    }
    if (!found) Serial.println(F("  [!] Ningun dispositivo encontrado."));
    Serial.println();
}