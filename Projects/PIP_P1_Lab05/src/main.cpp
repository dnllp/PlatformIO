// LAB 5: Control de Servo con Joystick KY-023 
#include <Arduino.h>
#include <Servo.h> 
const int JOY_X   = A0;   // Eje horizontal 
const int JOY_Y   = A1;   // Eje vertical 
const int JOY_BTN = 4;    
// Botón pulsador 
const int SERVO_X = 9;    // Servo eje X 
const int SERVO_Y = 10;   // Servo eje Y 
 
Servo servoX, servoY; 
 
void setup() { 
  servoX.attach(SERVO_X); 
  servoY.attach(SERVO_Y); 
  pinMode(JOY_BTN, INPUT_PULLUP); 
  Serial.begin(9600); 
  Serial.println("JoyX\tJoyY\tServoX\tServoY\tBtn"); 
} 
 
void loop() { 
  int jx = analogRead(JOY_X); 
  int jy = analogRead(JOY_Y); 
  int btn = digitalRead(JOY_BTN); 
 
  // Zona muerta en el centro (evita vibración) 
  int jxAdj = (abs(jx - 512) < 30) ? 512 : jx; 
  int jyAdj = (abs(jy - 512) < 30) ? 512 : jy; 
 
  // Mapear 0-1023 a 0-180 grados 
  int angX = map(jxAdj, 0, 1023, 0, 180); 
  int angY = map(jyAdj, 0, 1023, 0, 180); 
 
  servoX.write(angX); 
  servoY.write(angY); 
 
  // Botón presionado → centrar ambos servos 
  if (btn == LOW) { 
    servoX.write(90); 
    servoY.write(90); 
    Serial.println(">> Servos centrados"); 
  } 
 
  Serial.print(jx); Serial.print("\t"); 
  Serial.print(jy); Serial.print("\t"); 
  Serial.print(angX); Serial.print("\t"); 
  Serial.print(angY); Serial.print("\t"); 
  Serial.println(btn); 
 
  delay(50); 
}