/**
 * =============================================================================
 * FIRMWARE KUBOT - UAT MANTE
 * =============================================================================
 * Plataforma : ESP32 (con FreeRTOS integrado)
 * Sensores   : BNO055 (IMU/orientación) + TCS34725 (color RGB)
 * Actuadores : 2 motores DC con control PWM nativo
 * Conectividad: WiFi Access Point + Servidor Web + WebSockets
 * Almacenamiento: LittleFS (sirve la interfaz web desde flash)
 *
 * ARQUITECTURA DE TAREAS:
 *   ┌─ TaskControl   (50 Hz, prioridad 2) → Lógica de navegación
 *   └─ TaskTelemetry  (5 Hz, prioridad 1) → Envío de datos al cliente web
 *
 * CORRECCIONES RESPECTO A LA VERSIÓN ANTERIOR:
 *   1. Mutex para proteger variables compartidas entre tareas (race conditions)
 *   2. Normalización del color con canal claro 'c' para robustez ambiental
 *   3. currentPWM corregido a rango 8-bit (0–255)
 *   4. ws.cleanupClients() para evitar fuga de memoria en WebSockets
 *   5. Función de giro PID básica implementada
 *   6. Rampa de aceleración para proteger los motores
 * =============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <FS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_TCS34725.h>

// =============================================================================
// CONFIGURACIÓN DE PINES DE HARDWARE
// =============================================================================

// Pines de control de motores (PWM nativo del ESP32)
// Cada motor necesita 2 pines para controlar dirección y velocidad:
//   speedX > 0 → avanza   (MxA=speed, MxB=0)
//   speedX < 0 → retrocede (MxA=0,    MxB=|speed|)
const int M1A = 0;  // Motor Izquierdo - señal A
const int M1B = 1;  // Motor Izquierdo - señal B
const int M2A = 2;  // Motor Derecho   - señal A
const int M2B = 3;  // Motor Derecho   - señal B

// Pines I²C personalizados (el ESP32 permite reasignarlos)
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// =============================================================================
// PARÁMETROS DE CONFIGURACIÓN
// =============================================================================

// PWM: resolución 8 bits → rango válido 0–255
// CORREGIDO: el valor anterior (600) desbordaba el rango de 8 bits
const int PWM_CRUCERO   = 180;  // Velocidad normal de avance
const int PWM_GIRO      = 140;  // Velocidad durante un giro
const int PWM_MAX       = 255;  // Límite absoluto
const int RAMPA_PASO    = 10;   // Incremento por ciclo para suavizar arranque

// Tolerancia del PID de heading (en grados)
// Si el error de orientación es menor a esto, no corregimos
const float TOLERANCIA_HEADING = 2.0f;

// Ganancia proporcional del control de heading (ajustar en campo)
const float KP_HEADING = 1.5f;

// Umbrales de color NORMALIZADOS (proporción sobre canal claro)
// Esto hace la detección independiente de la iluminación ambiental
const float UMBRAL_ROJO_R  = 0.45f;  // R/C > este valor → componente rojo dominante
const float UMBRAL_ROJO_G  = 0.25f;  // G/C < este valor → verde bajo (confirma rojo)
const float UMBRAL_VERDE_G = 0.40f;  // G/C > este valor → componente verde dominante
const float UMBRAL_VERDE_R = 0.25f;  // R/C < este valor → rojo bajo (confirma verde)

// =============================================================================
// INSTANCIAS DE PERIFÉRICOS
// =============================================================================

// BNO055: IMU de 9 ejes. Entrega orientación absoluta (yaw/pitch/roll) en grados.
// Dirección I²C: 0x28 (pin ADR a GND) o 0x29 (pin ADR a 3.3V)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// TCS34725: Sensor de color RGB con canal claro.
// INTEGRATIONTIME_50MS: tiempo de integración del fotodiodo (más largo = más preciso)
// GAIN_4X: amplificación de la señal (ajustar según la distancia al objeto)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Servidor HTTP asíncrono en el puerto 80
AsyncWebServer server(80);

// WebSocket en la ruta /ws para telemetría bidireccional en tiempo real
AsyncWebSocket ws("/ws");

// =============================================================================
// VARIABLES GLOBALES COMPARTIDAS ENTRE TAREAS
// =============================================================================
// IMPORTANTE: Las variables compartidas entre tareas de FreeRTOS deben
// protegerse con un mutex para evitar condiciones de carrera (race conditions).
// Sin mutex, una tarea puede leer un valor a medio actualizar por la otra.

SemaphoreHandle_t dataMutex;  // Mutex que protege las variables de abajo

// Datos del IMU (escritos por TaskControl, leídos por TaskTelemetry)
float currentYaw = 0.0f;

// Datos del sensor de color (escritos por TaskControl, leídos por TaskTelemetry)
uint16_t r = 0, g = 0, b = 0, c = 0;  // Valores RAW de 16 bits del TCS34725
String detectedColor = "Iniciando...";

// Velocidad actual del robot (puede ajustarse desde la interfaz web en el futuro)
int currentPWM = PWM_CRUCERO;

// Flag que indica si los sensores I²C están disponibles
// Si es false, el robot entra en modo seguro (sin movimiento autónomo)
bool sensoresListos = false;

// Heading objetivo para el control de orientación (en grados, 0–360)
float targetYaw = 0.0f;

// =============================================================================
// PROTOTIPOS DE FUNCIONES
// =============================================================================
void TaskControl(void *pvParameters);
void TaskTelemetry(void *pvParameters);
void mover(int speedL, int speedR);
void moverConRampa(int speedL, int speedR);
void girarAHeading(float heading);
float calcularErrorHeading(float actual, float objetivo);

// =============================================================================
// SETUP - Ejecutado una vez al arrancar
// =============================================================================
void setup() {
    Serial.begin(115200);

    // Esperar al monitor serial (máx. 5 segundos para no bloquear el arranque)
    unsigned long start = millis();
    while (!Serial && (millis() - start < 5000));
    Serial.println("\n=== KUBOT INICIANDO ===");

    // -------------------------------------------------------------------------
    // 1. Inicializar el bus I²C con los pines personalizados
    // -------------------------------------------------------------------------
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("[I2C] Bus inicializado en SDA=" + String(SDA_PIN) + " SCL=" + String(SCL_PIN));

    // -------------------------------------------------------------------------
    // 2. Crear el mutex ANTES de crear las tareas que lo usan
    //    Si falla la creación del mutex, no hay memoria dinámica disponible.
    // -------------------------------------------------------------------------
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("[ERROR] No se pudo crear el mutex. Sistema detenido.");
        while(1); // Halt
    }
    Serial.println("[RTOS] Mutex creado correctamente.");

    // -------------------------------------------------------------------------
    // 3. Inicializar Sistema de Archivos LittleFS
    //    LittleFS almacena los archivos de la interfaz web (HTML, CSS, JS).
    //    El parámetro 'true' formatea la flash si el sistema está dañado.
    // -------------------------------------------------------------------------
    if (!LittleFS.begin(true)) {
        Serial.println("[ERROR] Fallo al montar LittleFS. Continuando sin interfaz web.");
    } else {
        Serial.println("[FS] LittleFS montado correctamente.");
    }

    // -------------------------------------------------------------------------
    // 4. Inicializar Sensores I²C
    //    Si alguno falla, el robot opera en modo seguro (sin sensores).
    // -------------------------------------------------------------------------
    bool bnoOk = bno.begin();
    bool tcsOk = tcs.begin();

    if (bnoOk && tcsOk) {
        sensoresListos = true;
        // Habilitar LED del TCS34725 para iluminación controlada del objeto
        tcs.setInterrupt(false); // LED encendido durante la medición
        Serial.println("[SENSORES] BNO055 y TCS34725 detectados y listos.");
    } else {
        Serial.print("[ADVERTENCIA] Modo sin sensores. BNO055=");
        Serial.print(bnoOk ? "OK" : "FALLO");
        Serial.print("  TCS34725=");
        Serial.println(tcsOk ? "OK" : "FALLO");
    }

    // -------------------------------------------------------------------------
    // 5. Configurar PWM de los motores
    //    analogWriteResolution(8) establece 8 bits → rango 0–255
    //    Debe llamarse ANTES de cualquier analogWrite()
    // -------------------------------------------------------------------------
    analogWriteResolution(8);
    // Asegurarse de que los motores arranquen detenidos
    mover(0, 0);
    Serial.println("[MOTORES] PWM configurado a 8 bits (0-255). Motores en STOP.");

    // -------------------------------------------------------------------------
    // 6. Configurar Access Point WiFi
    //    El robot crea su propia red WiFi a la que se conecta el operador.
    //    Esto permite operar sin necesidad de un router externo.
    // -------------------------------------------------------------------------
    WiFi.softAP("Kubot_UAT_Mante", "mante2026");
    Serial.print("[WIFI] Access Point activo. IP: ");
    Serial.println(WiFi.softAPIP());

    // -------------------------------------------------------------------------
    // 7. Configurar WebSockets
    //    Los eventos posibles son: CONNECT, DISCONNECT, DATA, ERROR, PONG
    // -------------------------------------------------------------------------
    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                  AwsEventType type, void *arg, uint8_t *data, size_t len) {

        if (type == WS_EVT_CONNECT) {
            Serial.printf("[WS] Cliente #%u conectado desde %s\n",
                          client->id(), client->remoteIP().toString().c_str());

        } else if (type == WS_EVT_DISCONNECT) {
            Serial.printf("[WS] Cliente #%u desconectado.\n", client->id());

        } else if (type == WS_EVT_DATA) {
            // Comando recibido desde la interfaz web
            String cmd = String((char*)data).substring(0, len);

            if (cmd == "calibrar") {
                // Capturar los valores actuales del TCS como referencia de blanco.
                // En una implementación completa, estos factores se guardarían
                // en LittleFS para persistir entre reinicios.
                Serial.println("[CALIBRACIÓN] Guardando referencia de blanco...");
                // TODO: leer tcs y guardar en LittleFS como JSON de calibración

            } else if (cmd == "stop") {
                mover(0, 0);
                Serial.println("[CMD] STOP recibido.");

            } else {
                Serial.printf("[WS] Comando desconocido: %s\n", cmd.c_str());
            }
        }
    });

    server.addHandler(&ws);

    // -------------------------------------------------------------------------
    // 8. Configurar rutas del Servidor Web
    //    "/" sirve la página principal desde LittleFS
    //    "/static" sirve todos los demás archivos estáticos
    // -------------------------------------------------------------------------
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/index.html", "text/html");
    });
    server.serveStatic("/", LittleFS, "/");
    server.begin();
    Serial.println("[HTTP] Servidor web iniciado.");

    // -------------------------------------------------------------------------
    // 9. Crear Tareas FreeRTOS
    //    xTaskCreate(función, nombre, stackBytes, parámetro, prioridad, handle)
    //    - TaskControl tiene prioridad más alta (2) para no ser interrumpida
    //      por la telemetría durante maniobras críticas.
    //    - TaskTelemetry tiene prioridad 1 (menor); puede esperar.
    // -------------------------------------------------------------------------
    xTaskCreate(TaskControl,   "Control",   4096, NULL, 2, NULL);
    xTaskCreate(TaskTelemetry, "Telemetry", 4096, NULL, 1, NULL);

    Serial.println("=== KUBOT LISTO ===\n");
}

// =============================================================================
// LOOP - Vacío intencionalmente
// =============================================================================
// En un sistema FreeRTOS el loop() es simplemente otra tarea con prioridad 1.
// Lo mantenemos vacío (con delay) para no interferir con nuestras tareas propias.
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// =============================================================================
// TAREA 1: CONTROL DE NAVEGACIÓN (Prioridad Alta - 50 Hz)
// =============================================================================
// Esta tarea es el "cerebro" del robot. Lee sensores y decide el movimiento.
// Se ejecuta cada 20 ms (50 veces por segundo) para una respuesta fluida.
void TaskControl(void *pvParameters) {
    for (;;) {

        if (sensoresListos) {
            // -----------------------------------------------------------------
            // A. Leer IMU (BNO055)
            //    event.orientation.x → Yaw (rumbo, 0°–360°)
            //    event.orientation.y → Roll (inclinación lateral)
            //    event.orientation.z → Pitch (inclinación frontal)
            // -----------------------------------------------------------------
            sensors_event_t event;
            bno.getEvent(&event);
            float newYaw = event.orientation.x;

            // -----------------------------------------------------------------
            // B. Leer Sensor de Color (TCS34725)
            //    Devuelve valores RAW de 16 bits para R, G, B y C (claro/lux).
            //    El canal C (clear) representa la luminosidad total y se usa
            //    para normalizar los otros canales y compensar la iluminación.
            // -----------------------------------------------------------------
            uint16_t newR, newG, newB, newC;
            tcs.getRawData(&newR, &newG, &newB, &newC);

            // -----------------------------------------------------------------
            // C. Detectar color usando proporciones normalizadas
            //    Dividir cada canal entre C elimina la dependencia de la
            //    intensidad de luz ambiental.
            // -----------------------------------------------------------------
            String newColor = "Buscando...";
            bool hayLuz = (newC > 10); // Evitar división por cero en oscuridad

            if (hayLuz) {
                float rNorm = (float)newR / newC;
                float gNorm = (float)newG / newC;

                if (rNorm > UMBRAL_ROJO_R && gNorm < UMBRAL_ROJO_G) {
                    // Rojo detectado → DETENER
                    newColor = "ROJO - STOP";
                    mover(0, 0);

                } else if (gNorm > UMBRAL_VERDE_G && rNorm < UMBRAL_VERDE_R) {
                    // Verde detectado → GIRAR 90° a la derecha
                    newColor = "VERDE - GIRANDO";
                    // Calcular nuevo heading objetivo (+90° sobre el actual)
                    targetYaw = fmod(newYaw + 90.0f, 360.0f);
                    girarAHeading(targetYaw);

                } else {
                    // Sin tarjeta especial → avanzar en línea recta
                    newColor = "Avanzando";
                    moverConRampa(currentPWM, currentPWM);
                }
            } else {
                // Sin luz suficiente (sensor tapado o en oscuridad)
                newColor = "Sin luz";
                mover(0, 0);
            }

            // -----------------------------------------------------------------
            // D. Actualizar variables compartidas CON MUTEX
            //    Tomamos el mutex, actualizamos, y lo liberamos inmediatamente.
            //    xSemaphoreTake con portMAX_DELAY espera indefinidamente si
            //    TaskTelemetry lo tiene tomado.
            // -----------------------------------------------------------------
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                currentYaw    = newYaw;
                r = newR; g = newG; b = newB; c = newC;
                detectedColor = newColor;
                xSemaphoreGive(dataMutex); // ← SIEMPRE liberar después de tomar
            }

        } else {
            // -----------------------------------------------------------------
            // MODO SEGURO: Sin sensores conectados
            // No intentamos leer I²C (causaría bloqueo o datos basura).
            // El robot se detiene y reporta el estado al cliente web.
            // -----------------------------------------------------------------
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                currentYaw    = 0;
                detectedColor = "Hardware Desconectado";
                xSemaphoreGive(dataMutex);
            }
        }

        // Esperar hasta el próximo ciclo de 20 ms (50 Hz)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// =============================================================================
// TAREA 2: TELEMETRÍA (Prioridad Baja - 5 Hz)
// =============================================================================
// Serializa el estado del robot como JSON y lo envía a todos los clientes
// WebSocket conectados. Se ejecuta a 5 Hz para no saturar el canal WiFi.
void TaskTelemetry(void *pvParameters) {
    for (;;) {
        // ---------------------------------------------------------------------
        // Limpiar clientes WebSocket desconectados para liberar memoria.
        // Sin esto, cada desconexión abrupta acumula recursos hasta el crash.
        // ---------------------------------------------------------------------
        ws.cleanupClients();

        // ---------------------------------------------------------------------
        // Leer variables compartidas CON MUTEX para obtener una instantánea
        // coherente (todos los valores del mismo ciclo de control).
        // ---------------------------------------------------------------------
        float   snapYaw;
        uint16_t snapR, snapG, snapB, snapC;
        String  snapColor;
        int     snapPWM;

        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            snapYaw   = currentYaw;
            snapR = r; snapG = g; snapB = b; snapC = c;
            snapColor = detectedColor;
            snapPWM   = currentPWM;
            xSemaphoreGive(dataMutex);
        } else {
            // Si no se pudo obtener el mutex en 10 ms, saltar este ciclo
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // ---------------------------------------------------------------------
        // Construir el JSON de telemetría
        //    yaw      → orientación actual (°)
        //    r/g/b    → color en 8 bits (dividimos entre 257 ≈ 16 bits → 8 bits)
        //    lux      → canal claro como proxy de luminosidad
        //    colorName → nombre del color detectado
        //    pwm      → velocidad actual
        // ---------------------------------------------------------------------
        StaticJsonDocument<256> doc;
        doc["yaw"]       = snapYaw;
        doc["r"]         = snapR >> 8;   // 16 bits → 8 bits para usar en CSS/canvas
        doc["g"]         = snapG >> 8;
        doc["b"]         = snapB >> 8;
        doc["lux"]       = snapC;
        doc["colorName"] = snapColor;
        doc["pwm"]       = snapPWM;

        String jsonString;
        serializeJson(doc, jsonString);

        // Enviar a todos los clientes conectados (broadcast)
        ws.textAll(jsonString);

        // Esperar 200 ms hasta el próximo envío (5 Hz de refresco)
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// =============================================================================
// FUNCIÓN: mover(speedL, speedR)
// =============================================================================
// Controla los dos motores de forma independiente.
//   speedL/R positivo → avanza
//   speedL/R negativo → retrocede
//   speedL/R = 0      → freno
// Rango esperado: -255 a 255 (resolución PWM de 8 bits)
void mover(int speedL, int speedR) {
    // Limitar al rango válido de 8 bits para evitar comportamiento indefinido
    speedL = constrain(speedL, -PWM_MAX, PWM_MAX);
    speedR = constrain(speedR, -PWM_MAX, PWM_MAX);

    // Motor Izquierdo
    if (speedL >= 0) {
        analogWrite(M1A, speedL);
        analogWrite(M1B, 0);
    } else {
        analogWrite(M1A, 0);
        analogWrite(M1B, abs(speedL));
    }

    // Motor Derecho
    if (speedR >= 0) {
        analogWrite(M2A, speedR);
        analogWrite(M2B, 0);
    } else {
        analogWrite(M2A, 0);
        analogWrite(M2B, abs(speedR));
    }
}

// =============================================================================
// FUNCIÓN: moverConRampa(speedL, speedR)
// =============================================================================
// Incrementa la velocidad gradualmente para evitar picos de corriente
// y proteger los motores y la batería en el arranque.
// La velocidad actual se almacena en variables estáticas internas.
void moverConRampa(int targetL, int targetR) {
    static int actualL = 0;
    static int actualR = 0;

    // Acercar cada motor a su objetivo en pasos de RAMPA_PASO
    if (actualL < targetL) actualL = min(actualL + RAMPA_PASO, targetL);
    else if (actualL > targetL) actualL = max(actualL - RAMPA_PASO, targetL);

    if (actualR < targetR) actualR = min(actualR + RAMPA_PASO, targetR);
    else if (actualR > targetR) actualR = max(actualR - RAMPA_PASO, targetR);

    mover(actualL, actualR);
}

// =============================================================================
// FUNCIÓN: calcularErrorHeading(actual, objetivo)
// =============================================================================
// Calcula el error de orientación teniendo en cuenta la circularidad de los
// ángulos (0° y 360° son la misma dirección).
// Retorna un valor en [-180, +180]: positivo=girar derecha, negativo=girar izq.
float calcularErrorHeading(float actual, float objetivo) {
    float error = objetivo - actual;
    // Normalizar al rango [-180, +180]
    while (error >  180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

// =============================================================================
// FUNCIÓN: girarAHeading(heading)
// =============================================================================
// Control proporcional (P) de orientación usando el BNO055.
// Gira el robot hasta alcanzar el heading objetivo dentro de la tolerancia.
// NOTA: Esta función es BLOQUEANTE dentro de TaskControl. Si se necesita
//       no-bloqueo, convertirla en una máquina de estados.
void girarAHeading(float heading) {
    if (!sensoresListos) return;

    // Límite de iteraciones para evitar bucle infinito si el sensor falla
    int intentos = 0;
    const int MAX_INTENTOS = 200; // 200 × 20ms = 4 segundos máximo

    while (intentos++ < MAX_INTENTOS) {
        sensors_event_t event;
        bno.getEvent(&event);
        float yawActual = event.orientation.x;

        float error = calcularErrorHeading(yawActual, heading);

        // Si estamos dentro de la tolerancia, el giro terminó
        if (abs(error) < TOLERANCIA_HEADING) {
            mover(0, 0);
            return;
        }

        // Control proporcional: mayor error → mayor corrección
        // Clamp para no superar PWM_GIRO
        int correccion = constrain((int)(KP_HEADING * error), -PWM_GIRO, PWM_GIRO);

        // Aplicar corrección diferencial: un motor avanza, el otro retrocede
        mover(correccion, -correccion);

        vTaskDelay(pdMS_TO_TICKS(20)); // Ceder CPU entre iteraciones
    }

    // Si se agotaron los intentos, detenerse de forma segura
    mover(0, 0);
    Serial.println("[GIRO] Timeout: no se alcanzó el heading objetivo.");
}