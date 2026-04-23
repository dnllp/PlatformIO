#include <Arduino.h>
#include <ESP32Encoder.h> // Manejo de PCNT (Hardware Pulse Counter)
#include <PID_v1.h>        // Librería PID clásica

// ==========================================
// CONFIGURACIÓN DE PINES Y HARDWARE
// ==========================================
const int PIN_ENCODER_A = 18;
const int PIN_ENCODER_B = 19;
const int PIN_LPWM = 25; // Control de giro en un sentido
const int PIN_RPWM = 26; // Control de giro en sentido opuesto

// --- Parámetros del Motor ---
// PPR = Pulsos por revolución del encoder * Relación de reducción de la caja
const double PPR = 1200; 

// --- Parámetros de Control ---
const float MAX_ACCEL = 3.0;     // Incremento de RPM por cada ciclo (ajusta la suavidad de la rampa)
const int FILTER_SAMPLES = 5;    // Número de muestras para el filtro de media móvil

// ==========================================
// VARIABLES GLOBALES
// ==========================================
double targetSetpoint = 80.0;  // La velocidad que TÚ quieres alcanzar (RPM)
double currentSetpoint = 0.0; // La velocidad que el PID sigue (la rampa)
double rpmActual = 0, outputPWM = 0;

// Sintonización PID (Kp, Ki, Kd)
double Kp = 1.8, Ki = 5.5, Kd = 0.08; 

// Objetos
ESP32Encoder encoder;
PID myPID(&rpmActual, &outputPWM, &currentSetpoint, Kp, Ki, Kd, DIRECT);

// Variables de tiempo y conteo
unsigned long prevTime = 0;
long prevTicks = 0;

// Filtro de Media Móvil
double rpmHistory[FILTER_SAMPLES] = {0};
int filterIndex = 0;

// ==========================================
// FUNCIONES AUXILIARES
// ==========================================

/**
 * Filtra la lectura de RPM para evitar vibraciones en el PID.
 * Suma las últimas muestras y calcula el promedio.
 */
double applyFilter(double newRpm) {
  rpmHistory[filterIndex] = newRpm;
  filterIndex = (filterIndex + 1) % FILTER_SAMPLES;
  
  double sum = 0;
  for (int i = 0; i < FILTER_SAMPLES; i++) sum += rpmHistory[i];
  return sum / FILTER_SAMPLES;
}

void setup() {
  Serial.begin(115200);
  
  // Configuración del Encoder usando el periférico PCNT del ESP32
  ESP32Encoder::useInternalWeakPullResistors = UP; 
  encoder.attachFullQuad(PIN_ENCODER_A, PIN_ENCODER_B);
  
  // Configuración de PWM para driver BTS7960 / IBT-2
  // Frecuencia de 5kHz es ideal para evitar silbidos audibles y calor excesivo
  ledcSetup(0, 5000, 8); // Canal 0, 5 kHz, resolución de 8 bits
  ledcSetup(1, 5000, 8); // Canal 1,  5 kHz, resolución de 8 bits
  ledcAttachPin(PIN_LPWM, 0); // Asignar canal 0 a LPWM
  ledcAttachPin(PIN_RPWM, 1); // Asignar canal 1 a RPWM


  // Configuración del Controlador PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // Rango completo (negativo = reversa)
  myPID.SetSampleTime(50);          // El PID se calcula cada 50ms
}

void loop() {
  unsigned long currentTime = millis();
  
  // Ejecutar el lazo de control cada 50 milisegundos
  if (currentTime - prevTime >= 50) { 
    
    // --- 1. CÁLCULO DE VELOCIDAD ---
    long currentTicks = encoder.getCount();
    long deltaTicks = currentTicks - prevTicks;
    
    // TPS = Ticks Per Second
    double tps = (deltaTicks / (double)(currentTime - prevTime)) * 1000.0;
    // Cálculo de RPM crudo y filtrado
    double rawRpm = (tps * 60.0) / PPR;
    rpmActual = applyFilter(rawRpm);
    
    // Cálculo opcional en Radianes/segundo
    double rad_s = rpmActual * (2.0 * PI / 60.0);

    // --- 2. GESTIÓN DE LA RAMPA ---
    // Si el setpoint objetivo es diferente al actual, nos acercamos poco a poco
    if (currentSetpoint < targetSetpoint) {
      currentSetpoint = min(currentSetpoint + MAX_ACCEL, targetSetpoint);
    } else if (currentSetpoint > targetSetpoint) {
      currentSetpoint = max(currentSetpoint - MAX_ACCEL, targetSetpoint);
    }

    // --- 3. PROCESAMIENTO PID ---
    myPID.Compute();
    
    // --- 4. SALIDA FÍSICA AL DRIVER (LPWM/RPWM) ---
    // Separamos la salida del PID en dos pines según el signo
    if (outputPWM > 0) {
      ledcWrite(PIN_LPWM, (uint32_t)outputPWM);
      ledcWrite(PIN_RPWM, 0); // Pin opuesto a tierra
    } else if (outputPWM < 0) {
      ledcWrite(PIN_LPWM, 0); 
      ledcWrite(PIN_RPWM, (uint32_t)abs(outputPWM)); // PWM en pin de reversa
    } else {
      ledcWrite(PIN_LPWM, 0);
      ledcWrite(PIN_RPWM, 0);
    }

    // --- 5. DEBUG / MONITORIZACIÓN ---
    // Formato compatible con el "Serial Plotter" de Arduino
    Serial.printf("Target:%.1f SetpointRampa:%.1f RPM_Filtrado:%.1f PWM:%.0f\n", 
                  targetSetpoint, currentSetpoint, rpmActual, outputPWM);

    // Guardar estado para el siguiente ciclo
    prevTicks = currentTicks;
    prevTime = currentTime;
  }

  // Lógica de prueba: Cambia de sentido cada 8 segundos
  if (millis() % 16000 < 8000) {
    targetSetpoint = 100.0;
  } else {
    targetSetpoint = -100.0;
  }
}
