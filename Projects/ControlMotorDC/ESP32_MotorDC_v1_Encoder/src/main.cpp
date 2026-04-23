#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>

// Pines del Driver y Encoder
const int PIN_ENCODER_A = 18;
const int PIN_ENCODER_B = 19;
const int PIN_LPWM = 25; // Giro horario
const int PIN_RPWM = 26; // Giro anti-horario

// Configuración del Motor
const double PPR = 1200; // Pulsos por revolución totales (eje de salida)

// Variables PID
double setpoint = 60.0;  // Velocidad objetivo en RPM
double input = 0, output = 0;
double Kp = 1.5, Ki = 4.0, Kd = 0.05; 

ESP32Encoder encoder;
// El PID debe ser capaz de dar valores negativos para retroceder
// pero lo configuraremos de 0 a 255 y manejaremos el signo manualmente
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

unsigned long prevTime = 0;
long prevTicks = 0;

void setup() {
  Serial.begin(115200);
  
  // 1. Configurar Encoder
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachFullQuad(PIN_ENCODER_A, PIN_ENCODER_B);
  
  // 2. Configurar PWM para el Driver (Lógica 3.x de ESP32 Arduino Core)
  ledcSetup(0, 5000, 8); // Canal 0, 5 kHz, resolución de 8 bits
  ledcSetup(1, 5000, 8); // Canal 1, 5 kHz, resolución de 8 bits
  ledcAttachPin(PIN_LPWM, 0); // 5kHz, 8 bits
  ledcAttachPin(PIN_RPWM, 1);

  // 3. Configurar PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // Permitir rango negativo para reversa
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - prevTime >= 50) { 
    // Cálculo de ticks y velocidad
    long currentTicks = encoder.getCount();
    long deltaTicks = currentTicks - prevTicks;
    
    double tps = (deltaTicks / (double)(currentTime - prevTime)) * 1000.0;
    input = (tps * 60.0) / PPR; // RPM actuales
    double rad_s = input * (2.0 * PI / 60.0);

    myPID.Compute();
    
    // --- Lógica de Control LPWM / RPWM ---
    if (output > 0) {
      ledcWrite(PIN_LPWM, (uint32_t)output);
      ledcWrite(PIN_RPWM, 0);
    } else if (output < 0) {
      ledcWrite(PIN_LPWM, 0);
      ledcWrite(PIN_RPWM, (uint32_t)abs(output));
    } else {
      ledcWrite(PIN_LPWM, 0);
      ledcWrite(PIN_RPWM, 0);
    }

    // Monitorización para Serial Plotter
    Serial.printf("Target:%.1f, Actual:%.1f, Rads:%.2f, PWM:%.0f\n", 
                  setpoint, input, rad_s, output);

    prevTicks = currentTicks;
    prevTime = currentTime;
  }
}
