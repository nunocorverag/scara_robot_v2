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

// --- Configuración de Control de Posición ---
const int PWM_SPEED = 100;      // Velocidad lenta para el movimiento de retorno
const long TOLERANCE = 10;      // Margen de error para considerar alcanzado el "cero" (en ticks)

// ---------------------------------------
// Rutina de Servicio de Interrupción (ISR) - Lógica de Cuadratura CORREGIDA
// ---------------------------------------
void IRAM_ATTR encoderISR() {
  // Leemos el estado del Pin B en el momento exacto en que A cambia.
  int estadoB = digitalRead(ENC_B);
  
  // Si A y B son diferentes (A=LOW, B=HIGH o A=HIGH, B=LOW), el conteo es positivo.
  if (digitalRead(ENC_A) != estadoB) {
    encoderCount++; 
  } 
  // Si A y B son iguales, el conteo es negativo.
  else {
    encoderCount--; 
  }
}

// ---------------------------------------
// Funciones de Movimiento
// ---------------------------------------

void motorForwardTiny() {
  Serial.println("Movimiento Positivo (+)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 160);
  delay(100);            
  analogWrite(ENA, 0);   
}

void motorBackwardTiny() {
  Serial.println("Movimiento Negativo (-)");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 160);
  delay(100);
  analogWrite(ENA, 0);
}

void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0); 
}

// ---------------------------------------
// Función para ir a Posición Cero (Control Básico de Posición)
// ---------------------------------------
void returnToZero() {
  const long TARGET_TICKS = 0; // El cero es la última posición donde se presionó 'r'
  long currentTicks;

  Serial.println("Iniciando movimiento a Posición Cero...");

  // Bucle de control: se ejecuta mientras estemos fuera de la tolerancia
  // abs() calcula el valor absoluto de la diferencia.
  while (abs(encoderCount - TARGET_TICKS) > TOLERANCE) {
    
    // Lectura segura del contador
    noInterrupts();
    currentTicks = encoderCount;
    interrupts();

    // Lógica para mover el motor en la dirección correcta
    if (currentTicks > TARGET_TICKS) {
      // Si estamos en un valor positivo, debemos movernos hacia atrás (Decremento)
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, PWM_SPEED);
    } 
    else if (currentTicks < TARGET_TICKS) {
      // Si estamos en un valor negativo, debemos movernos hacia adelante (Incremento)
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, PWM_SPEED);
    }
    
    // Imprime la posición en tiempo real para seguimiento
    Serial.print("Ticks: "); Serial.println(currentTicks);

    // Pequeña pausa para permitir que el bucle se ejecute
    delay(10); 
  }

  motorStop(); 
  Serial.print("Posición Cero alcanzada. Ticks finales: "); Serial.println(encoderCount);
}


// ---------------------------------------
// Setup
// ---------------------------------------
void setup() {
  Serial.begin(115200);

  // 1. Configuración de Pines de Control de Dirección
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // 2. Configuración de Pines del Encoder: Usar PULLUP para evitar ruido.
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // 3. Configuración del PWM (Compatible con versiones antiguas del Core)
  // ledcAttach asocia el pin ENA a un canal de PWM con la Frecuencia y Resolución.
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION); 
  
  // 4. Configuración de la Interrupción
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  motorStop();

  Serial.println("=== SCARA Motor Test (Control de Posición) ===");
  Serial.println("Controles:");
  Serial.println("  d = paso adelante (+)");
  Serial.println("  a = paso atrás (-)");
  Serial.println("  0 = IR A POSICIÓN CERO (último 'r')");
  Serial.println("  r = reiniciar encoder (establecer POSICIÓN CERO)");
  Serial.println("----------------------------------------");
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
    Serial.println(val);
  }

  // Leer comandos del usuario
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read(); // limpiar buffer
    
    if (c == 'd') {
      motorForwardTiny();
    }
    else if (c == 'a') {
      motorBackwardTiny();
    }
    else if (c == 's') {
      motorStop();
      Serial.println("Motor detenido.");
    }
    else if (c == 'r') {
      noInterrupts();
      encoderCount = 0; // ESTABLECE el nuevo CERO (0)
      interrupts();
      Serial.println("POSICIÓN CERO establecida.");
    }
    else if (c == '0') {
      returnToZero(); // MUEVE el motor a la última posición CERO
    }
  }
}