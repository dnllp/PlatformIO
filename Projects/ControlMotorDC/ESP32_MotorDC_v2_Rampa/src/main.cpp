#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>

// Pines y Configuración
const int PIN_ENCODER_A = 18;
const int PIN_ENCODER_B = 19;
const int PIN_LPWM = 25;
const int PIN_RPWM = 26;

const double PPR = 1200; // Ajusta según tu motor
const float MAX_ACCEL = 5.0; // RPM por cada ciclo de 50ms (ajusta la rampa aquí)

// Variables PID
double targetSetpoint = 100.0; // Velocidad final deseada (RPM)
double currentSetpoint = 0.0; // Velocidad intermedia (rampa)
double input = 0, output = 0;
double Kp = 1.8, Ki = 5.0, Kd = 0.05; 

ESP32Encoder encoder;
PID myPID(&input, &output, &currentSetpoint, Kp, Ki, Kd, DIRECT);

unsigned long prevTime = 0;
long prevTicks = 0;

void setup() {
  Serial.begin(115200);
  
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachFullQuad(PIN_ENCODER_A, PIN_ENCODER_B);
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);  
  ledcAttachPin(PIN_LPWM, 0);
  ledcAttachPin(PIN_RPWM, 1);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(50); // Sincronizado con el loop
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - prevTime >= 50) { 
    // 1. Cálculo de Velocidad (RPM y rad/s)
    long currentTicks = encoder.getCount();
    long deltaTicks = currentTicks - prevTicks;
    double tps = (deltaTicks / (double)(currentTime - prevTime)) * 1000.0;
    input = (tps * 60.0) / PPR;
    double rad_s = input * (2.0 * PI / 60.0);

    // 2. Lógica de Rampa de Aceleración
    if (currentSetpoint < targetSetpoint) {
      currentSetpoint += MAX_ACCEL;
      if (currentSetpoint > targetSetpoint) currentSetpoint = targetSetpoint;
    } else if (currentSetpoint > targetSetpoint) {
      currentSetpoint -= MAX_ACCEL;
      if (currentSetpoint < targetSetpoint) currentSetpoint = targetSetpoint;
    }

    // 3. Cálculo PID
    myPID.Compute();
    
    // 4. Salida al Driver LPWM/RPWM
    if (output > 0) {
      ledcWrite(PIN_LPWM, (uint32_t)output);
      ledcWrite(PIN_RPWM, 0);
    } else {
      ledcWrite(PIN_LPWM, 0);
      ledcWrite(PIN_RPWM, (uint32_t)abs(output));
    }

    // 5. Telemetría para Serial Plotter
    Serial.printf("Target:%.1f CurrentSet:%.1f Actual:%.1f PWM:%.0f\n", 
                  targetSetpoint, currentSetpoint, input, output);

    prevTicks = currentTicks;
    prevTime = currentTime;
  }

  // Ejemplo: Cambiar setpoint cada 5 segundos para probar la rampa
  if (millis() > 10000 && targetSetpoint == 100.0) targetSetpoint = -100.0;
}
