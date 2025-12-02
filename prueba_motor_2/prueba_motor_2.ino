#include "driver/ledc.h"

// ---------------------------
// CONFIGURACIÓN DE PINES (MOTOR 2)
// ---------------------------
#define IN3 14
#define IN4 12
#define ENB 13   // PWM motor 2

#define ENC2_A 32
#define ENC2_B 33

// ---------------------------
// VARIABLES DEL ENCODER MOTOR 2
// ---------------------------
volatile long encoder2Count = 0;

// ---------------------------
// INTERRUPCIÓN FASE A MOTOR 2
// ---------------------------
void IRAM_ATTR encoder2ISR() {
  int A = digitalRead(ENC2_A);
  int B = digitalRead(ENC2_B);

  if (A == B) encoder2Count += 1;
  else        encoder2Count -= 1;
}

void setup() {
  Serial.begin(115200);

  // Pines del motor 2
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Pines encoder motor 2
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, CHANGE);

  // PWM para ENB
  ledcAttach(ENB, 20000, 8);   // pin, frecuencia 20kHz, resolución 8 bits

  Serial.println("Motor 2 + Encoder Test listo.");
}

void loop() {
  // MOTOR 2 ADELANTE
  Serial.println("Motor 2 adelante...");
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  encoder2Count = 0;
  ledcWrite(ENB, 200);   // velocidad media
  delay(1000);

  Serial.print("Pulsos Motor 2 adelante: ");
  Serial.println(encoder2Count);

  // MOTOR 2 ATRÁS
  Serial.println("Motor 2 atrás...");
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  encoder2Count = 0;
  ledcWrite(ENB, 200);
  delay(1000);

  Serial.print("Pulsos Motor 2 atrás: ");
  Serial.println(encoder2Count);

  // STOP
  Serial.println("Motor 2 Stop");
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENB, 0);
  delay(2000);
}
