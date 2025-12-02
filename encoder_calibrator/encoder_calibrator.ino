#include <Arduino.h> 

// ---------------------------
// Pines del Motor 1 (Potencia y Control L298N)
// ---------------------------
#define IN1 25     // L298N IN1 -> Dirección 1
#define IN2 26     // L298N IN2 -> Dirección 2
#define ENA 27     // L298N ENA -> PWM Velocidad

// ---------------------------
// Pines ENCODER 1
// ---------------------------
#define ENC_A 34   // Encoder Fase A (Interrupción)
#define ENC_B 35   // Encoder Fase B (Lectura de Dirección)

volatile long encoderCount = 0; // Contador de pulsos del encoder
unsigned long lastPrint = 0;

// --- Configuración de PWM ---
const int PWM_FREQ = 20000;     // Frecuencia de PWM (20 kHz)
const int PWM_RESOLUTION = 8;   // Resolución de 8 bits (valores de 0 a 255)
const int MAX_PWM = 255;        // Valor máximo de PWM (para 8 bits)
const int MIN_PWM = 0;          // Valor mínimo de PWM (0)

// --- Configuración de Control de Posición y Velocidad ---
int PWM_SPEED = 100;            // VELOCIDAD BASE AJUSTABLE (PWM de 0 a 255)
const int TINY_SPEED_PWM = 160; // Velocidad fija para los movimientos 'a' y 'd'
const long TOLERANCE = 10;      // Margen de error para considerar alcanzado el "cero" (en ticks)

// Variables para almacenar el estado previo (mejor lectura)
volatile int lastEncA = 0;
volatile int lastEncB = 0;

// ---------------------------------------
// Rutina de Servicio de Interrupción (ISR) - CORREGIDA
// ---------------------------------------
void IRAM_ATTR encoderISR() {
  int currentA = digitalRead(ENC_A);
  int currentB = digitalRead(ENC_B);
  
  // Tabla de estados para decodificación cuadratura
  // Si A y B cambian en sentido horario: incrementa
  // Si A y B cambian en sentido antihorario: decrementa
  
  if (lastEncA == LOW && currentA == HIGH) {
    // Flanco de subida en A
    if (currentB == LOW) {
      encoderCount++;  // Giro en una dirección
    } else {
      encoderCount--;  // Giro en dirección opuesta
    }
  }
  else if (lastEncA == HIGH && currentA == LOW) {
    // Flanco de bajada en A
    if (currentB == HIGH) {
      encoderCount++;  // Giro en una dirección
    } else {
      encoderCount--;  // Giro en dirección opuesta
    }
  }
  
  lastEncA = currentA;
  lastEncB = currentB;
}

// Si la dirección sigue invertida, usa esta versión alternativa:
/*
void IRAM_ATTR encoderISR() {
  int currentA = digitalRead(ENC_A);
  int currentB = digitalRead(ENC_B);
  
  if (lastEncA == LOW && currentA == HIGH) {
    if (currentB == HIGH) {  // INVERTIDO
      encoderCount++;
    } else {
      encoderCount--;
    }
  }
  else if (lastEncA == HIGH && currentA == LOW) {
    if (currentB == LOW) {  // INVERTIDO
      encoderCount++;
    } else {
      encoderCount--;
    }
  }
  
  lastEncA = currentA;
  lastEncB = currentB;
}
*/

// ---------------------------------------
// Funciones de Movimiento
// ---------------------------------------

void setMotorSpeed(int speed) {
  ledcWrite(ENA, speed); 
}

void motorForwardTiny() {
  Serial.println("Movimiento Positivo (+)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  setMotorSpeed(TINY_SPEED_PWM); 
  delay(100);            
  setMotorSpeed(0);   
}

void motorBackwardTiny() {
  Serial.println("Movimiento Negativo (-)");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  setMotorSpeed(TINY_SPEED_PWM); 
  delay(100);
  setMotorSpeed(0);
}

void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  setMotorSpeed(0); 
}

// ---------------------------------------
// Función para ir a Posición Cero - MEJORADA
// ---------------------------------------
void returnToZero() {
  const long TARGET_TICKS = 0; 
  long currentTicks;
  long lastTicks = encoderCount;
  unsigned long stuckTime = 0;
  const unsigned long STUCK_TIMEOUT = 2000; // 2 segundos sin cambio = atascado

  Serial.print("Iniciando movimiento a Posición Cero. Velocidad actual: ");
  Serial.println(PWM_SPEED);

  while (abs(encoderCount - TARGET_TICKS) > TOLERANCE) {
    
    noInterrupts();
    currentTicks = encoderCount;
    interrupts();

    // Detectar si está atascado
    if (currentTicks == lastTicks) {
      if (stuckTime == 0) {
        stuckTime = millis();
      } else if (millis() - stuckTime > STUCK_TIMEOUT) {
        Serial.println("ADVERTENCIA: Motor atascado o sin movimiento. Abortando...");
        break;
      }
    } else {
      stuckTime = 0; // Reiniciar timeout si hay movimiento
      lastTicks = currentTicks;
    }

    if (currentTicks > TARGET_TICKS) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      setMotorSpeed(PWM_SPEED);
    } 
    else if (currentTicks < TARGET_TICKS) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      setMotorSpeed(PWM_SPEED);
    }
    
    Serial.print("Ticks: "); Serial.println(currentTicks);
    delay(10); 
  }

  motorStop(); 
  Serial.print("Posición Cero alcanzada. Ticks finales: "); Serial.println(encoderCount);
}

// ---------------------------------------
// Setup - MEJORADO
// ---------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // Leer estados iniciales
  lastEncA = digitalRead(ENC_A);
  lastEncB = digitalRead(ENC_B);

  // Configuración del PWM (para ESP32)
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION); 
  
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  motorStop();

  Serial.println("=== SCARA Motor Test (Control de Posición y Velocidad Manual) ===");
  Serial.println("Controles:");
  Serial.println("  d = paso adelante (+)");
  Serial.println("  a = paso atrás (-)");
  Serial.println("  0 = IR A POSICIÓN CERO (usa la velocidad ajustable)");
  Serial.println("  r = reiniciar encoder (establecer POSICIÓN CERO)");
  Serial.println("  p[XXX] = ESTABLECER PWM MANUALMENTE (ej: p150, p255, p0)");
  Serial.println("  i = INVERTIR dirección del encoder");
  Serial.println("----------------------------------------");
  Serial.print("Velocidad PWM inicial: "); Serial.println(PWM_SPEED);
}

// ---------------------------------------
// Loop
// ---------------------------------------
void loop() {

  // Imprimir encoder cada 1 segundo
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    noInterrupts();
    long val = encoderCount;
    interrupts();
    
    Serial.print("Encoder = ");
    Serial.print(val);
    Serial.print(" | PWM Actual = ");
    Serial.println(PWM_SPEED);
  }

  // Leer comandos del usuario
  if (Serial.available()) {
    char command = Serial.read();
    
    if (command == 'd') {
      motorForwardTiny();
    }
    else if (command == 'a') {
      motorBackwardTiny();
    }
    else if (command == 'r') {
      noInterrupts();
      encoderCount = 0; 
      interrupts();
      Serial.println("POSICIÓN CERO establecida.");
    }
    else if (command == '0') {
      returnToZero(); 
    }
    else if (command == 'p') {
      int newPwm = -1;
      
      if (Serial.peek() >= '0' && Serial.peek() <= '9') {
        newPwm = Serial.parseInt();
      }
      
      while (Serial.available()) {
        Serial.read();
      }
      
      if (newPwm >= MIN_PWM && newPwm <= MAX_PWM) {
        PWM_SPEED = newPwm;
        Serial.print("PWM establecido. Nueva velocidad: ");
        Serial.println(PWM_SPEED);
      } else {
        Serial.print("Error: Valor PWM (");
        Serial.print(newPwm);
        Serial.println(") fuera del rango 0-255. Intente: p150.");
      }
    } 
    else {
      while (Serial.available()) {
        Serial.read();
      }
    }
  }
}