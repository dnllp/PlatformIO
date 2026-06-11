/**
 * ============================================================
 * Robot Diferencial — TCS34725 + BNO055 + L298N + LED RGB
 * Plataforma : PlatformIO + Arduino (ESP32 30 pines)
 * ============================================================
 *
 * ── Bus I2C (GPIO 21=SDA, GPIO 22=SCL) ──────────────────────
 *   TCS34725  0x29  →  sensor de color
 *   BNO055    0x28  →  IMU (yaw)
 *
 * ── LED RGB (cátodo común) ───────────────────────────────────
 *   R → GPIO 25  (LEDC ch 0)
 *   G → GPIO 26  (LEDC ch 1)
 *   B → GPIO 27  (LEDC ch 2)
 *
 * ── Botón ────────────────────────────────────────────────────
 *   GPIO 4  (INPUT_PULLUP, activo en LOW)
 *
 * ── L298N — Puente H ─────────────────────────────────────────
 *   IN1 → GPIO 16  (LEDC ch 3)  Motor A adelante
 *   IN2 → GPIO 17  (LEDC ch 4)  Motor A atrás
 *   IN3 → GPIO 18  (LEDC ch 5)  Motor B adelante  (invertido HW)
 *   IN4 → GPIO 19  (LEDC ch 6)  Motor B atrás     (invertido HW)
 *   ENA/ENB → puentes cerrados (5 V permanente en el L298N)
 *
 * ── Lógica de colores calibrados ─────────────────────────────
 *   Color 1 → STOP  (alto total)
 *   Color 2 → AVANZAR al 70 % con corrección IMU
 *   Color 3 → Girar izquierda  90° y avanzar
 *   Color 4 → Girar derecha    90° y avanzar
 *   Color 5 → Girar izquierda  45° y avanzar
 *   Color 6 → Girar derecha    45° y avanzar
 *   Color 7 → Girar            180° y avanzar
 *   Color 8 → AVANZAR al 100 % con corrección IMU
 *
 * ── Notas ────────────────────────────────────────────────────
 *   • Motor B está invertido en hardware; se compensa en sw.
 *   • Velocidad global: 70 % por defecto, 100 % tras Color 8.
 *   • Giros: frenado suave al entrar en zona ±10° del objetivo.
 *   • Corrección lateral avance: controlador proporcional sobre yaw.
 *   • Tolerancia de giro: ±5°.
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ════════════════════════════════════════════════════════════
//  PINES
// ════════════════════════════════════════════════════════════
#define PIN_LED_R   25
#define PIN_LED_G   26
#define PIN_LED_B   27

#define PIN_BOTON    4

#define PIN_IN1     16   // Motor A adelante
#define PIN_IN2     17   // Motor A atrás
#define PIN_IN3     18   // Motor B adelante (HW invertido)
#define PIN_IN4     19   // Motor B atrás    (HW invertido)

// ════════════════════════════════════════════════════════════
//  LEDC
// ════════════════════════════════════════════════════════════
#define LEDC_FREQ   5000
#define LEDC_RES       8   // 0–255
#define CH_LED_R       0
#define CH_LED_G       1
#define CH_LED_B       2
#define CH_IN1         3
#define CH_IN2         4
#define CH_IN3         5
#define CH_IN4         6

// ════════════════════════════════════════════════════════════
//  CONFIGURACIÓN GENERAL
// ════════════════════════════════════════════════════════════
#define LED_ANODO_COMUN   0     // 1 si es ánodo común

#define NUM_COLORES       8
#define LECTURAS_CAL     20
#define INTERVALO_CAL   250
#define MS_HOLD_CAL    2000
#define MS_PAUSA_CAL   1500

#define UMBRAL_DISTANCIA  60.0f

#define PARPADEO_MS      150

// ── Velocidades ───────────────────────────────────────────────
#define VEL_DEFECTO      178    // ~70 % de 255
#define VEL_MAXIMA       255    // 100 %
#define VEL_GIRO_LENTO    76    // ~30 % para frenado suave

// ── Corrección de avance (proporcional sobre yaw) ─────────────
#define KP_AVANCE       1.8f   // ganancia proporcional
#define MAX_CORR         40    // límite de corrección PWM

// ── Giro ──────────────────────────────────────────────────────
#define TOL_GIRO         5.0f  // ±5° tolerancia de llegada
#define ZONA_FRENO      10.0f  // ±10° inicio de frenado suave

// ════════════════════════════════════════════════════════════
//  OBJETOS DE SENSOR
// ════════════════════════════════════════════════════════════
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
    TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ════════════════════════════════════════════════════════════
//  ESTADO GLOBAL
// ════════════════════════════════════════════════════════════
enum Estado { ESPERA, CALIBRACION, DETECCION };
Estado estadoActual = ESPERA;

struct ColorRGB { uint8_t r, g, b; };
ColorRGB coloresCal[NUM_COLORES];
uint8_t  totalCalibrados = 0;

uint8_t  velActual  = VEL_DEFECTO;   // velocidad global actual
float    yawObjetivo = 0.0f;         // ángulo de referencia de avance

// ── Parpadeo no bloqueante ────────────────────────────────────
uint8_t  ledR_det = 0, ledG_det = 0, ledB_det = 0;
bool     ledEncendido    = false;
uint32_t tUltimoParpadeo = 0;

// ── Botón ─────────────────────────────────────────────────────
uint32_t tBotonPresionado = 0;
bool     botonAnterior    = HIGH;

// ════════════════════════════════════════════════════════════
//  PROTOTIPOS
// ════════════════════════════════════════════════════════════
// LED
void iniciarLEDC();
void setLed(uint8_t r, uint8_t g, uint8_t b);
void apagarLed();
void secuenciaBienvenida();

// Motores
void iniciarMotores();
void motorRaw(int pwmA, int pwmB);   // + adelante, - atrás
void avanzar(uint8_t vel);
void frenar();
void girarIzquierda(uint8_t vel);
void girarDerecha(uint8_t vel);

// IMU
float   leerYaw();
float   diffAngular(float objetivo, float actual);
void    girarGrados(float grados);   // + izq, - der
void    avanzarConIMU();

// Sensor color
ColorRGB leerColorPromedio();
int8_t   buscarColorMasCercano(ColorRGB muestra);
float    distanciaRGB(ColorRGB a, ColorRGB b);

// Lógica principal
void manejarBoton();
void modoCalibracion();
void ejecutarComportamiento(uint8_t idxColor);

// Utilidades
void imprimirColor(const char* etiq, ColorRGB c);
void escanearI2C();

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("\n==========================================="));
    Serial.println(F("  Robot Diferencial | TCS34725 + BNO055"));
    Serial.println(F("===========================================\n"));

    iniciarLEDC();
    iniciarMotores();
    secuenciaBienvenida();

    pinMode(PIN_BOTON, INPUT_PULLUP);

    escanearI2C();

    // ── TCS34725 ──────────────────────────────────────────────
    if (!tcs.begin()) {
        Serial.println(F("[ERROR] TCS34725 no encontrado."));
        while (true) { setLed(255,0,0); delay(300); apagarLed(); delay(300); }
    }
    Serial.println(F("[OK] TCS34725 listo (0x29)"));

    // ── BNO055 ────────────────────────────────────────────────
    if (!bno.begin()) {
        Serial.println(F("[ERROR] BNO055 no encontrado."));
        while (true) { setLed(255,80,0); delay(300); apagarLed(); delay(300); }
    }
    bno.setExtCrystalUse(true);
    Serial.println(F("[OK] BNO055 listo (0x28)"));

    Serial.println(F("\n── Instrucciones ──────────────────────────"));
    Serial.println(F("  Mantener boton 2 s  → CALIBRACION"));
    Serial.println(F("  Pulsar boton 1 vez  → DETECCION"));
    Serial.println(F("───────────────────────────────────────────\n"));
}

// ════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════
void loop() {
    manejarBoton();

    if (estadoActual == DETECCION) {

        // ── Parpadeo no bloqueante ─────────────────────────────
        uint32_t ahora = millis();
        if (ahora - tUltimoParpadeo >= PARPADEO_MS) {
            tUltimoParpadeo = ahora;
            ledEncendido ? apagarLed() : setLed(ledR_det, ledG_det, ledB_det);
            ledEncendido = !ledEncendido;
        }

        // ── Lectura de color cada 300 ms ───────────────────────
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
                Serial.print(F("Color detectado: #"));
                Serial.println(idx + 1);
                ejecutarComportamiento(idx);
            } else {
                ledR_det = 0; ledG_det = 0; ledB_det = 0;
                Serial.println(F("Sin coincidencia"));
                frenar();
            }
        }
    }
}

// ════════════════════════════════════════════════════════════
//  BOTÓN
// ════════════════════════════════════════════════════════════
void manejarBoton() {
    bool actual = digitalRead(PIN_BOTON);

    if (botonAnterior == HIGH && actual == LOW)
        tBotonPresionado = millis();

    if (botonAnterior == LOW && actual == HIGH) {
        uint32_t dur = millis() - tBotonPresionado;

        if (dur >= MS_HOLD_CAL) {
            frenar();
            Serial.println(F("\n[BOTON] Pulsacion larga → CALIBRACION"));
            estadoActual = CALIBRACION;
            modoCalibracion();

        } else if (dur >= 50) {
            if (totalCalibrados == 0) {
                Serial.println(F("[BOTON] Sin colores calibrados."));
                for (uint8_t i = 0; i < 3; i++) {
                    setLed(255,180,0); delay(150); apagarLed(); delay(150);
                }
            } else {
                frenar();
                Serial.println(F("\n[BOTON] Pulsacion corta → DETECCION"));
                estadoActual    = DETECCION;
                ledR_det = ledG_det = ledB_det = 0;
                ledEncendido    = false;
                tUltimoParpadeo = millis();
            }
        }
    }
    botonAnterior = actual;
}

// ════════════════════════════════════════════════════════════
//  CALIBRACIÓN
// ════════════════════════════════════════════════════════════
void modoCalibracion() {
    totalCalibrados = 0;
    velActual = VEL_DEFECTO;   // resetear velocidad al calibrar
    Serial.println(F("\n════ CALIBRACION ════"));

    for (uint8_t i = 0; i < NUM_COLORES; i++) {
        Serial.print(F("\nColor #")); Serial.print(i + 1);
        Serial.println(F(" — Coloca la tarjeta (5 s)..."));
        setLed(255, 0, 0);
        delay(1000);

        uint32_t sumR = 0, sumG = 0, sumB = 0;
        for (uint8_t j = 0; j < LECTURAS_CAL; j++) {
            uint16_t r, g, b, c;
            tcs.getRawData(&r, &g, &b, &c);
            if (c > 0) {
                sumR += (uint32_t)constrain((float)r / c * 255, 0, 255);
                sumG += (uint32_t)constrain((float)g / c * 255, 0, 255);
                sumB += (uint32_t)constrain((float)b / c * 255, 0, 255);
            }
            Serial.print('.');
            delay(INTERVALO_CAL);
        }
        Serial.println();

        coloresCal[i] = {
            (uint8_t)(sumR / LECTURAS_CAL),
            (uint8_t)(sumG / LECTURAS_CAL),
            (uint8_t)(sumB / LECTURAS_CAL)
        };
        totalCalibrados++;

        Serial.print(F("Guardado #")); Serial.print(i + 1); Serial.print(F(": "));
        imprimirColor("", coloresCal[i]);

        setLed(0, 0, 255);
        delay(MS_PAUSA_CAL);
    }

    setLed(0, 255, 0);
    Serial.println(F("\n[OK] 8 colores calibrados. Pulsa el boton para detectar.\n"));
    delay(2000);
    apagarLed();
    estadoActual = ESPERA;
}

// ════════════════════════════════════════════════════════════
//  COMPORTAMIENTO POR COLOR
// ════════════════════════════════════════════════════════════
void ejecutarComportamiento(uint8_t idx) {
    switch (idx) {

        case 0:  // ── Color 1: STOP ────────────────────────────
            frenar();
            Serial.println(F("→ STOP"));
            break;

        case 1:  // ── Color 2: Avanzar 70 % con corrección IMU ─
            velActual = VEL_DEFECTO;
            Serial.println(F("→ AVANZAR 70 % + correccion IMU"));
            yawObjetivo = leerYaw();
            avanzarConIMU();
            break;

        case 2:  // ── Color 3: Girar izq 90° y avanzar ─────────
            velActual = VEL_DEFECTO;
            Serial.println(F("→ GIRAR IZQ 90° + AVANZAR"));
            frenar(); delay(100);
            girarGrados(90.0f);
            yawObjetivo = leerYaw();
            avanzarConIMU();
            break;

        case 3:  // ── Color 4: Girar der 90° y avanzar ─────────
            velActual = VEL_DEFECTO;
            Serial.println(F("→ GIRAR DER 90° + AVANZAR"));
            frenar(); delay(100);
            girarGrados(-90.0f);
            yawObjetivo = leerYaw();
            avanzarConIMU();
            break;

        case 4:  // ── Color 5: Girar izq 45° y avanzar ─────────
            velActual = VEL_DEFECTO;
            Serial.println(F("→ GIRAR IZQ 45° + AVANZAR"));
            frenar(); delay(100);
            girarGrados(45.0f);
            yawObjetivo = leerYaw();
            avanzarConIMU();
            break;

        case 5:  // ── Color 6: Girar der 45° y avanzar ─────────
            velActual = VEL_DEFECTO;
            Serial.println(F("→ GIRAR DER 45° + AVANZAR"));
            frenar(); delay(100);
            girarGrados(-45.0f);
            yawObjetivo = leerYaw();
            avanzarConIMU();
            break;

        case 6:  // ── Color 7: Girar 180° y avanzar ───────────
            velActual = VEL_DEFECTO;
            Serial.println(F("→ GIRAR 180° + AVANZAR"));
            frenar(); delay(100);
            girarGrados(180.0f);
            yawObjetivo = leerYaw();
            avanzarConIMU();
            break;

        case 7:  // ── Color 8: Avanzar velocidad máxima ────────
            velActual = VEL_MAXIMA;
            Serial.println(F("→ AVANZAR VELOCIDAD MAXIMA (100 %)"));
            yawObjetivo = leerYaw();
            avanzarConIMU();
            break;

        default:
            frenar();
            break;
    }
}

// ════════════════════════════════════════════════════════════
//  IMU — YAW
// ════════════════════════════════════════════════════════════

// Devuelve yaw en grados [0, 360)
float leerYaw() {
    sensors_event_t ev;
    bno.getEvent(&ev);
    return ev.orientation.x;   // BNO055: eje X = yaw (heading)
}

// Diferencia angular mínima en [-180, +180]
// positivo = hay que girar a la izquierda para alcanzar objetivo
float diffAngular(float objetivo, float actual) {
    float d = objetivo - actual;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// Gira 'grados' usando BNO055.
// grados > 0 → izquierda, grados < 0 → derecha
void girarGrados(float grados) {
    float inicio  = leerYaw();
    float objetivo = inicio + grados;
    // Normalizar objetivo a [0, 360)
    while (objetivo <   0.0f) objetivo += 360.0f;
    while (objetivo >= 360.0f) objetivo -= 360.0f;

    Serial.print(F("  Girando ")); Serial.print(grados);
    Serial.print(F("° | inicio: ")); Serial.print(inicio);
    Serial.print(F("° | objetivo: ")); Serial.print(objetivo); Serial.println(F("°"));

    uint32_t tMax = millis() + 5000;   // tiempo máximo de seguridad

    while (millis() < tMax) {
        float actual = leerYaw();
        float error  = diffAngular(objetivo, actual);

        if (fabs(error) <= TOL_GIRO) {
            Serial.println(F("  Giro completado."));
            break;
        }

        // Velocidad reducida cerca del objetivo
        uint8_t velGiro = (fabs(error) <= ZONA_FRENO) ? VEL_GIRO_LENTO : velActual;

        if (error > 0) girarIzquierda(velGiro);
        else           girarDerecha(velGiro);

        delay(10);
    }
    frenar();
    delay(150);
}

// Avanza con corrección proporcional de yaw.
// Bloqueante hasta que el modo cambie (nuevo color detectado).
// El loop() llama a ejecutarComportamiento() que llama esta función;
// la detección de nuevo color interrumpirá el ciclo porque
// el sensor se lee en el loop y sobreescribe el comportamiento.
// Aquí se hace una sola iteración de control para no bloquear loop().
void avanzarConIMU() {
    float actual = leerYaw();
    float error  = diffAngular(yawObjetivo, actual);
    float corr   = constrain(KP_AVANCE * error, -MAX_CORR, MAX_CORR);

    int pwmA = (int)velActual + (int)corr;
    int pwmB = (int)velActual - (int)corr;

    pwmA = constrain(pwmA, 0, 255);
    pwmB = constrain(pwmB, 0, 255);

    motorRaw(pwmA, pwmB);
}

// ════════════════════════════════════════════════════════════
//  MOTORES
// ════════════════════════════════════════════════════════════
void iniciarMotores() {
    ledcSetup(CH_IN1, LEDC_FREQ, LEDC_RES); ledcAttachPin(PIN_IN1, CH_IN1);
    ledcSetup(CH_IN2, LEDC_FREQ, LEDC_RES); ledcAttachPin(PIN_IN2, CH_IN2);
    ledcSetup(CH_IN3, LEDC_FREQ, LEDC_RES); ledcAttachPin(PIN_IN3, CH_IN3);
    ledcSetup(CH_IN4, LEDC_FREQ, LEDC_RES); ledcAttachPin(PIN_IN4, CH_IN4);
    frenar();
    Serial.println(F("[OK] Motores listos (GPIO 16/17/18/19)."));
}

// pwmA, pwmB: [-255, 255]  positivo=adelante, negativo=atrás
// Motor B invertido en hardware → sus IN se invierten en SW
void motorRaw(int pwmA, int pwmB) {
    // Motor A
    if (pwmA >= 0) {
        ledcWrite(CH_IN1, pwmA); ledcWrite(CH_IN2, 0);
    } else {
        ledcWrite(CH_IN1, 0);    ledcWrite(CH_IN2, -pwmA);
    }
    // Motor B (invertido HW: intercambiar IN3/IN4)
    if (pwmB >= 0) {
        ledcWrite(CH_IN3, 0);    ledcWrite(CH_IN4, pwmB);
    } else {
        ledcWrite(CH_IN3, -pwmB); ledcWrite(CH_IN4, 0);
    }
}

void avanzar(uint8_t vel)       { motorRaw( vel,  vel); }
void frenar()                   { motorRaw(0, 0); }
void girarIzquierda(uint8_t vel){ motorRaw(-vel,  vel); }
void girarDerecha(uint8_t vel)  { motorRaw( vel, -vel); }

// ════════════════════════════════════════════════════════════
//  SENSOR DE COLOR — UTILIDADES
// ════════════════════════════════════════════════════════════
int8_t buscarColorMasCercano(ColorRGB muestra) {
    float  menorDist = UMBRAL_DISTANCIA;
    int8_t idx = -1;
    for (uint8_t i = 0; i < totalCalibrados; i++) {
        float d = distanciaRGB(muestra, coloresCal[i]);
        if (d < menorDist) { menorDist = d; idx = i; }
    }
    return idx;
}

float distanciaRGB(ColorRGB a, ColorRGB b) {
    float dr = (float)a.r - b.r;
    float dg = (float)a.g - b.g;
    float db = (float)a.b - b.b;
    return sqrtf(dr*dr + dg*dg + db*db);
}

void imprimirColor(const char* etiq, ColorRGB c) {
    if (etiq && etiq[0]) { Serial.print(etiq); Serial.print(F(": ")); }
    Serial.print(F("R:")); Serial.print(c.r);
    Serial.print(F(" G:")); Serial.print(c.g);
    Serial.print(F(" B:")); Serial.println(c.b);
}

// ════════════════════════════════════════════════════════════
//  LED RGB
// ════════════════════════════════════════════════════════════
void iniciarLEDC() {
    ledcSetup(CH_LED_R, LEDC_FREQ, LEDC_RES); ledcAttachPin(PIN_LED_R, CH_LED_R);
    ledcSetup(CH_LED_G, LEDC_FREQ, LEDC_RES); ledcAttachPin(PIN_LED_G, CH_LED_G);
    ledcSetup(CH_LED_B, LEDC_FREQ, LEDC_RES); ledcAttachPin(PIN_LED_B, CH_LED_B);
    apagarLed();
    Serial.println(F("[OK] LED RGB listo (GPIO 25/26/27)."));
}

void setLed(uint8_t r, uint8_t g, uint8_t b) {
#if LED_ANODO_COMUN
    ledcWrite(CH_LED_R, 255-r);
    ledcWrite(CH_LED_G, 255-g);
    ledcWrite(CH_LED_B, 255-b);
#else
    ledcWrite(CH_LED_R, r);
    ledcWrite(CH_LED_G, g);
    ledcWrite(CH_LED_B, b);
#endif
}

void apagarLed() { setLed(0,0,0); }

void secuenciaBienvenida() {
    setLed(255,0,0); delay(250);
    setLed(0,255,0); delay(250);
    setLed(0,0,255); delay(250);
    apagarLed();
}

// ════════════════════════════════════════════════════════════
//  I2C SCAN
// ════════════════════════════════════════════════════════════
void escanearI2C() {
    Serial.println(F("[I2C] Escaneando bus..."));
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("  0x"));
            if (addr < 16) Serial.print('0');
            Serial.print(addr, HEX);
            if (addr == 0x29) Serial.print(F("  <-- TCS34725"));
            if (addr == 0x28) Serial.print(F("  <-- BNO055"));
            Serial.println();
            found++;
        }
    }
    if (!found) Serial.println(F("  [!] Ningun dispositivo encontrado."));
    Serial.println();
}