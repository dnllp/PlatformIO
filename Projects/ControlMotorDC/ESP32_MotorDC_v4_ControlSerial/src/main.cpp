#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>

// ==========================================
// CONFIGURACIÓN DE PINES
// ==========================================
const int PIN_ENCODER_A = 18;
const int PIN_ENCODER_B = 19;
const int PIN_LPWM = 25; 
const int PIN_RPWM = 26;

// --- Parámetros Mecánicos ---
const double PPR = 1200; 
const float MAX_ACCEL = 3.0;     
const int FILTER_SAMPLES = 5;    

// --- Variables de Control (Globales para que el PID las vea) ---
double targetSetpoint = 0.0;  
double currentSetpoint = 0.0; 
double rpmActual = 0, outputPWM = 0;

// Constantes PID (Editables por Serial)
double Kp = 1.8, Ki = 5.5, Kd = 0.08; 

// Objetos
ESP32Encoder encoder;
PID myPID(&rpmActual, &outputPWM, &currentSetpoint, Kp, Ki, Kd, DIRECT);

// Estado de tiempo y filtro
unsigned long prevTime = 0;
long prevTicks = 0;
double rpmHistory[FILTER_SAMPLES] = {0};
int filterIndex = 0;

// ==========================================
// LOGICA DE COMANDOS SERIALES
// ==========================================
/**
 * Lee el puerto serial buscando comandos como:
 * 'S100'  -> Cambia Setpoint a 100 RPM
 * 'P2.5'  -> Cambia Kp a 2.5
 * 'I1.0'  -> Cambia Ki a 1.0
 * 'D0.1'  -> Cambia Kd a 0.1
 */
void procesarComandosSeriales() {
  if (Serial.available() > 0) {
    char tipo = Serial.read(); // Lee la letra del comando
    float valor = Serial.parseFloat(); // Lee el número que sigue

    switch (tipo) {
      case 'S': case 's':
        targetSetpoint = valor;
        Serial.printf(">> Nuevo Setpoint: %.2f RPM\n", targetSetpoint);
        break;
      case 'P': case 'p':
        Kp = valor;
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.printf(">> Nuevo Kp: %.2f\n", Kp);
        break;
      case 'I': case 'i':
        Ki = valor;
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.printf(">> Nuevo Ki: %.2f\n", Ki);
        break;
      case 'D': case 'd':
        Kd = valor;
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.printf(">> Nuevo Kd: %.2f\n", Kd);
        break;
    }
  }
}

// Filtro de media móvil para estabilizar la lectura
double applyFilter(double newRpm) {
  rpmHistory[filterIndex] = newRpm;
  filterIndex = (filterIndex + 1) % FILTER_SAMPLES;
  double sum = 0;
  for (int i = 0; i < FILTER_SAMPLES; i++) sum += rpmHistory[i];
  return sum / FILTER_SAMPLES;
}

void setup() {
  Serial.begin(115200);
  Serial.println("--- Sistema de Control de Motor DC Iniciado ---");
  Serial.println("Comandos: S(rpm), P(kp), I(ki), D(kd). Ej: S60 o P2.1");
  
  ESP32Encoder::useInternalWeakPullResistors = UP; 
  encoder.attachFullQuad(PIN_ENCODER_A, PIN_ENCODER_B);
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);

  ledcAttachPin(PIN_LPWM, 0); 
  ledcAttachPin(PIN_RPWM, 1);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(50);
}

void loop() {
  // 1. Escuchar constantemente el puerto serial
  procesarComandosSeriales();

  unsigned long currentTime = millis();
  
  // 2. Lazo de Control cada 50ms
  if (currentTime - prevTime >= 50) { 
    
    // CALCULO VELOCIDAD
    long currentTicks = encoder.getCount();
    double tps = ((currentTicks - prevTicks) / (double)(currentTime - prevTime)) * 1000.0;
    rpmActual = applyFilter((tps * 60.0) / PPR);
    
    // RAMPA DE ACELERACION
    if (currentSetpoint < targetSetpoint) currentSetpoint = min(currentSetpoint + MAX_ACCEL, targetSetpoint);
    else if (currentSetpoint > targetSetpoint) currentSetpoint = max(currentSetpoint - MAX_ACCEL, targetSetpoint);

    // CONTROL PID
    myPID.Compute();
    
    // SALIDA AL DRIVER
    if (outputPWM > 0) {
      ledcWrite(PIN_LPWM, (uint32_t)outputPWM);
      ledcWrite(PIN_RPWM, 0);
    } else {
      ledcWrite(PIN_LPWM, 0); 
      ledcWrite(PIN_RPWM, (uint32_t)abs(outputPWM));
    }

    // TELEMETRIA (Formato para Serial Plotter)
    Serial.printf("Target:%.1f,Actual:%.1f,PWM:%.0f,P:%.1f,I:%.1f,D:%.2f\n", 
                  targetSetpoint, rpmActual, outputPWM, Kp, Ki, Kd);

    prevTicks = currentTicks;
    prevTime = currentTime;
  }
}
