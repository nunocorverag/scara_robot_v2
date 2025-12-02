#include "driver/ledc.h"

// ---------------------------
// CONFIGURACIÓN DE PINES
// ---------------------------
#define IN1 25
#define IN2 26
#define ENA 27  // PWM motor
#define ENC_A 34
#define ENC_B 35

// Variable volátil para el contador del codificador
volatile long encoderCount = 0;
bool motorEnabled = false;  // Estado del motor

// ---------------------------
// INTERRUPCIÓN FASE A (MEJORADA PARA CUADRATURA)
// ---------------------------
// La interrupción se dispara en *cada* cambio de ENC_A.
// La dirección se determina al comparar el estado actual de A con el estado de B.
void IRAM_ATTR encoderISR() {
  // Lee los estados de A y B en el momento de la interrupción.
  // Es crucial que esta lógica sea lo más rápida posible.
  if (digitalRead(ENC_A) != digitalRead(ENC_B)) {
    // Si son diferentes (ejemplo: A=HIGH, B=LOW), va en una dirección
    encoderCount++;
  } else {
    // Si son iguales (ejemplo: A=HIGH, B=HIGH), va en la otra dirección
    encoderCount--;
  }
}

// ---------------------------
// APAGAR MOTOR
// ---------------------------
void motorStop() {
  ledcWrite(ENA, 0); // Detiene el PWM
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  Serial.println("Motor APAGADO");
  motorEnabled = false;
}

// ---------------------------
// ARRANCAR SECUENCIA DEL MOTOR
// ---------------------------
void motorEnableSequence() {
  long startCount = 0; // Para guardar la lectura inicial

  Serial.println("Motor habilitado. Ejecutando pruebas...");
  motorEnabled = true;
  
  // Limpia el contador antes de empezar la prueba
  noInterrupts();
  encoderCount = 0;
  interrupts();
  
  // ------------------------------------
  // ------ 1. MOTOR ADELANTE ------
  // ------------------------------------
  Serial.println("--- Motor adelante (1 segundo) ---");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, 200); // Velocidad del 78% (200/255)
  delay(1000); // Espera 1 segundo para la prueba
  
  // Detiene el motor inmediatamente después de la prueba de avance
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW); 
  ledcWrite(ENA, 0); 

  // Lee el contador de forma segura
  noInterrupts();
  startCount = encoderCount;
  interrupts();
  
  Serial.print("Pulsos adelante totales: ");
  Serial.println(startCount);
  delay(500); // Breve pausa

  // ------------------------------------
  // ------ 2. MOTOR ATRÁS ------
  // ------------------------------------
  Serial.println("--- Motor atrás (1 segundo) ---");
  
  // Reinicia el contador para medir solo el movimiento de regreso
  noInterrupts();
  encoderCount = 0; 
  interrupts();

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(ENA, 200);
  delay(1000); // Espera 1 segundo para la prueba

  // Detiene el motor
  motorStop(); // Usamos la función motorStop para finalizar el ciclo

  // Lee el contador de forma segura
  noInterrupts();
  long endCount = encoderCount;
  interrupts();

  Serial.print("Pulsos atrás totales: ");
  Serial.println(endCount);
  Serial.println("------------------------------------");
}

void setup() {
  // Asegúrate de que esta velocidad (115200) coincida con el Monitor Serial
  Serial.begin(115200);

  // Pines de Control del Motor (Salida)
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Pines del Codificador (Entrada)
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  
  // Configuración de la interrupción en el pin A
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  // CONFIGURACIÓN DEL PWM CORREGIDA:
  // Usa la función 'ledcAttach' que tu IDE parece reconocer.
  ledcAttach(ENA, 20000, 8); 
  
  // Inicialización del motor apagado
  motorStop(); 

  Serial.println("\n*** Sistema de Pruebas de Robot SCARA Listo ***");
  Serial.println("-------------------------------------------------");
  Serial.println("Escribe el comando y presiona ENTER (o Enviar):");
  Serial.println("   E → Habilitar motor y correr secuencia de prueba");
  Serial.println("   S → Apagar el motor inmediatamente");
  Serial.println("-------------------------------------------------");
}

void loop() {

  // ---------------------------
  // LEER COMANDOS DEL SERIAL
  // ---------------------------
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'E' || c == 'e') {
      if (!motorEnabled) {
        motorEnableSequence();
      } else {
        Serial.println("El motor ya está habilitado. Presiona 'S' para detener.");
      }
    }

    if (c == 'S' || c == 's') {
      motorStop();
    }
    
    // Si usas el Monitor Serial con "Newline" o "Both NL & CR", 
    // consume los caracteres de retorno de carro o nueva línea para evitar repeticiones.
    while (Serial.available()) {
      Serial.read();
    }
  }

  // ---------------------------
  // Tareas periódicas (si las hubiera, sin usar delay)
  // ---------------------------
}