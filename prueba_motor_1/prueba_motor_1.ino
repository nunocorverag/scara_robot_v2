#include "driver/ledc.h"

// ---------------------------
// CONFIGURACIÓN DE PINES
// ---------------------------
#define IN1 25
#define IN2 26
#define ENA 27  // PWM motor
#define ENC_A 34
#define ENC_B 35

volatile long encoderCount = 0;
bool motorEnabled = false;  // Estado del motor

// ---------------------------
// INTERRUPCIÓN FASE A
// ---------------------------
void IRAM_ATTR encoderISR() {
  int A = digitalRead(ENC_A);
  int B = digitalRead(ENC_B);
  if (A == B) encoderCount += 1;
  else        encoderCount -= 1;
}

// ---------------------------
// APAGAR MOTOR
// ---------------------------
void motorStop() {
  ledcWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  Serial.println("Motor APAGADO");
  motorEnabled = false;
}

// ---------------------------
// ARRANCAR SECUENCIA DEL MOTOR
// ---------------------------
void motorEnableSequence() {
  Serial.println("Motor habilitado. Ejecutando pruebas...");
  motorEnabled = true;

  // ------ MOTOR ADELANTE ------
  Serial.println("Motor adelante...");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  encoderCount = 0;
  ledcWrite(ENA, 200);
  delay(1000);
  Serial.print("Pulsos adelante: ");
  Serial.println(encoderCount);

  // ------ MOTOR ATRÁS ------
  Serial.println("Motor atrás...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  encoderCount = 0;
  ledcWrite(ENA, 200);
  delay(1000);
  Serial.print("Pulsos atrás: ");
  Serial.println(encoderCount);

  // APAGAR AL FINAL
  motorStop();
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  ledcAttach(ENA, 20000, 8); // PWM

  Serial.println("Sistema listo. Escribe:");
  Serial.println("   E → habilitar motor y correr prueba");
  Serial.println("   S → apagar motor en cualquier momento");
}

void loop() {

  // ---------------------------
  // LEER COMANDOS DEL SERIAL
  // ---------------------------
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'E' || c == 'e') {
      if (!motorEnabled) motorEnableSequence();
      else Serial.println("El motor ya está habilitado.");
    }

    if (c == 'S' || c == 's') {
      motorStop();
    }
  }

  // ---------------------------
  // SI EL MOTOR ESTÁ APAGADO → NO HACER NADA
  // ---------------------------
}
