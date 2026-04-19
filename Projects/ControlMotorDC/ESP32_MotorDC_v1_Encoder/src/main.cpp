#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include <esp32-hal-ledc.h>

// Pines y Constantes
const int PIN_ENCODER_A = 18;
const int PIN_ENCODER_B = 19;
const int PIN_PWM = 25;
const int PIN_DIR = 26;
const double PPR = 1200; // Ajusta según tu motor (PPR * Relación de reducción)

// Variables de Control
double setpoint = 50.0;  // Velocidad deseada en RPM
double input = 0, output = 0;
double Kp = 2.0, Ki = 5.0, Kd = 0.1; // Sintonizar según el motor

ESP32Encoder encoder;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

unsigned long prevTime = 0;
long prevTicks = 0;

void setup() {
  Serial.begin(115200);
  
  // Configurar Encoder con PCNT
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachFullQuad(PIN_ENCODER_A, PIN_ENCODER_B);
  
  // Configurar Motor (PWM)
  pinMode(PIN_DIR, OUTPUT);
  ledcSetup(0, 5000, 8); // Canal 0, 5kHz, 8-bit resolución
  ledcAttachPin(PIN_PWM, 0); // 5kHz, 8-bit resolución (Arduino ESP32 Core 3.x)

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= 50) { // Muestreo cada 50ms
    long currentTicks = encoder.getCount();
    long deltaTicks = currentTicks - prevTicks;
    
    // Cálculos de Velocidad
    double tps = (deltaTicks / (double)(currentTime - prevTime)) * 1000.0;
    input = (tps * 60.0) / PPR; // RPM
    double rad_s = input * (2.0 * PI / 60.0);

    // Control PID
    myPID.Compute();
    
    // Aplicar al motor
    digitalWrite(PIN_DIR, setpoint >= 0 ? HIGH : LOW);
    ledcWrite(PIN_PWM, abs(output));

    // Monitorización
    Serial.printf("Setpoint: %.1f | RPM: %.1f | Rad/s: %.2f | PWM: %.0f\n", 
                  setpoint, input, rad_s, output);

    prevTicks = currentTicks;
    prevTime = currentTime;
  }
}


