#include <Arduino.h> 

// ---------------------------
// Pines del Motor 1 (Potencia y Control L298N)
// ---------------------------
#define IN1 25
#define IN2 26
#define ENA 27

// ---------------------------
// Pines ENCODER 1
// ---------------------------
#define ENC_A 34
#define ENC_B 35

volatile long encoderCount = 0;
unsigned long lastPrint = 0;

// --- Configuración de PWM ---
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int MAX_PWM = 255;
const int MIN_PWM = 160;  // PWM mínimo para mover el motor

// --- Límites de Seguridad ---
const float MIN_ANGLE = 45.0;   // Ángulo mínimo permitido
const float MAX_ANGLE = 110.0;  // Ángulo máximo permitido
const long TOLERANCE = 15;      // Tolerancia en ticks (±5 grados aprox)

// --- Calibración de Grados ---
const float TICKS_PER_DEGREE = 3.157;
const float REFERENCE_ANGLE = 90.0;  // Posición de referencia al resetear

// --- Control PID (Optimizado para movimiento SUAVE) ---
float Kp = 0.8;   // Ganancia proporcional (MÁS BAJO = más suave)
float Ki = 0.02;  // Ganancia integral (muy baja)
float Kd = 1.2;   // Ganancia derivativa (alta para amortiguar)

float lastError = 0;
float integral = 0;
unsigned long lastPIDTime = 0;

// --- Variables de Estado ---
volatile int lastEncA = 0;
volatile int lastEncB = 0;
bool motorRunning = false;
bool debugMode = false;
bool emergencyStop = false;
bool verboseMode = true;  // Mostrar progreso detallado
int currentPWM = 0;       // PWM actual para rampa suave

// ---------------------------------------
// FUNCIONES DE CONVERSIÓN
// ---------------------------------------
float ticksToDegrees(long ticks) {
  return (ticks / TICKS_PER_DEGREE) + REFERENCE_ANGLE;
}

long degreesToTicks(float degrees) {
  return (long)((degrees - REFERENCE_ANGLE) * TICKS_PER_DEGREE);
}

// ---------------------------------------
// ISR del Encoder
// ---------------------------------------
void IRAM_ATTR encoderISR() {
  int currentA = digitalRead(ENC_A);
  int currentB = digitalRead(ENC_B);
  
  if (lastEncA == LOW && currentA == HIGH) {
    if (currentB == LOW) {
      encoderCount--;  // ← INVERTIDO
    } else {
      encoderCount++;  // ← INVERTIDO
    }
  }
  else if (lastEncA == HIGH && currentA == LOW) {
    if (currentB == HIGH) {
      encoderCount--;  // ← INVERTIDO
    } else {
      encoderCount++;  // ← INVERTIDO
    }
  }
  
  lastEncA = currentA;
  lastEncB = currentB;
}

// ---------------------------------------
// Control del Motor
// ---------------------------------------
void setMotorSpeed(int speed) {
  speed = constrain(speed, 0, MAX_PWM);
  ledcWrite(ENA, speed);
}

void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  setMotorSpeed(0);
  motorRunning = false;
  emergencyStop = false;
  currentPWM = 0;  // Reiniciar rampa
  
  // Reset PID
  lastError = 0;
  integral = 0;
}

void moveMotor(int direction, int speed) {
  if (direction > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (direction < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    motorStop();
    return;
  }
  setMotorSpeed(speed);
  motorRunning = true;
}

// ---------------------------------------
// Control PID con Aceleración Suave
// ---------------------------------------
int calculatePID(long currentTicks, long targetTicks) {
  unsigned long now = millis();
  float deltaTime = (now - lastPIDTime) / 1000.0;
  
  if (deltaTime <= 0) deltaTime = 0.01;
  
  // Calcular error
  float error = targetTicks - currentTicks;
  
  // Término proporcional
  float P = Kp * error;
  
  // Término integral (con anti-windup)
  integral += error * deltaTime;
  integral = constrain(integral, -30, 30);
  float I = Ki * integral;
  
  // Término derivativo
  float derivative = (error - lastError) / deltaTime;
  float D = Kd * derivative;
  
  // Calcular salida PID
  float output = P + I + D;
  
  // Mapeo SUAVE con velocidades más bajas
  int targetPWM;
  float absError = abs(error);
  
  if (absError > TOLERANCE * 4) {
    // Lejos: velocidad moderada-baja
    targetPWM = map(constrain(abs(output), 0, 150), 0, 150, MIN_PWM, MIN_PWM + 40);
  } else if (absError > TOLERANCE * 2) {
    // Medio: velocidad baja
    targetPWM = map(constrain(abs(output), 0, 80), 0, 80, MIN_PWM, MIN_PWM + 20);
  } else {
    // Cerca: velocidad mínima
    targetPWM = MIN_PWM;
  }
  
  // RAMPA DE ACELERACIÓN SUAVE (cambio gradual de PWM)
  int pwm;
  if (targetPWM > currentPWM) {
    // Acelerando: incremento suave
    currentPWM += 3;  // Acelera de a poco
    if (currentPWM > targetPWM) currentPWM = targetPWM;
  } else if (targetPWM < currentPWM) {
    // Desacelerando: decremento suave
    currentPWM -= 5;  // Frena más rápido
    if (currentPWM < targetPWM) currentPWM = targetPWM;
  }
  pwm = currentPWM;
  
  pwm = constrain(pwm, MIN_PWM, MIN_PWM + 50);  // Límite más bajo
  
  lastError = error;
  lastPIDTime = now;
  
  if (debugMode) {
    Serial.print("PID - Error: "); Serial.print(error);
    Serial.print(" | P: "); Serial.print(P, 1);
    Serial.print(" | I: "); Serial.print(I, 1);
    Serial.print(" | D: "); Serial.print(D, 1);
    Serial.print(" | Target PWM: "); Serial.print(targetPWM);
    Serial.print(" | Actual PWM: "); Serial.println(pwm);
  }
  
  return pwm;
}

// ---------------------------------------
// Función para ir a un ángulo
// ---------------------------------------
bool goToAngle(float targetDegrees) {
  // Verificar límites de seguridad
  if (targetDegrees < MIN_ANGLE || targetDegrees > MAX_ANGLE) {
    Serial.print("ERROR: Ángulo fuera de rango (");
    Serial.print(MIN_ANGLE);
    Serial.print("-");
    Serial.print(MAX_ANGLE);
    Serial.println("°)");
    return false;
  }
  
  long targetTicks = degreesToTicks(targetDegrees);
  long currentTicks;
  long lastTicks = encoderCount;
  unsigned long stuckTime = 0;
  const unsigned long STUCK_TIMEOUT = 3000;
  
  emergencyStop = false;
  lastError = 0;
  integral = 0;
  currentPWM = 0;  // Reiniciar PWM para rampa suave
  lastPIDTime = millis();
  
  Serial.print("→ Moviendo a ");
  Serial.print(targetDegrees);
  Serial.print("° (");
  Serial.print(targetTicks);
  Serial.print(" ticks) | Distancia: ");
  Serial.print(abs(targetDegrees - ticksToDegrees(encoderCount)), 1);
  Serial.println("°");
  
  int progressCounter = 0;
  
  while (abs(encoderCount - targetTicks) > TOLERANCE) {
    // Verificar emergency stop
    if (emergencyStop) {
      motorStop();
      Serial.println("⚠ MOVIMIENTO DETENIDO POR USUARIO");
      return false;
    }
    
    // Verificar comandos durante el movimiento
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "stop" || cmd == "s") {
        emergencyStop = true;
        continue;
      }
    }
    
    noInterrupts();
    currentTicks = encoderCount;
    interrupts();
    
    // Detectar si está atascado
    if (currentTicks == lastTicks) {
      if (stuckTime == 0) {
        stuckTime = millis();
      } else if (millis() - stuckTime > STUCK_TIMEOUT) {
        motorStop();
        Serial.println("⚠ ERROR: Motor atascado. Abortando...");
        return false;
      }
    } else {
      stuckTime = 0;
      lastTicks = currentTicks;
    }
    
    // Calcular control PID
    int pwm = calculatePID(currentTicks, targetTicks);
    
    // Determinar dirección
    if (currentTicks > targetTicks) {
      moveMotor(-1, pwm);  // Retroceder
    } else {
      moveMotor(1, pwm);   // Avanzar
    }
    
    // Imprimir progreso (cada 5 iteraciones para no saturar)
    if (verboseMode && (progressCounter++ % 5 == 0)) {
      float currentAngle = ticksToDegrees(currentTicks);
      float progress = 100.0 * (1.0 - abs(currentTicks - targetTicks) / (float)abs(lastTicks - targetTicks));
      progress = constrain(progress, 0, 100);
      
      Serial.print("  ");
      Serial.print(currentAngle, 1);
      Serial.print("° → ");
      Serial.print(targetDegrees, 1);
      Serial.print("° [");
      
      // Barra de progreso visual
      int bars = (int)(progress / 10);
      for (int i = 0; i < 10; i++) {
        Serial.print(i < bars ? "█" : "░");
      }
      
      Serial.print("] ");
      Serial.print((int)progress);
      Serial.print("% | PWM:");
      Serial.println(pwm);
    }
    
    delay(10);
  }
  
  motorStop();
  
  noInterrupts();
  currentTicks = encoderCount;
  interrupts();
  
  Serial.println("✓ Posición alcanzada!");
  Serial.print("  Ticks finales: ");
  Serial.print(currentTicks);
  Serial.print(" | Ángulo: ");
  Serial.print(ticksToDegrees(currentTicks), 2);
  Serial.println("°");
  
  return true;
}

// ---------------------------------------
// Funciones de Ayuda
// ---------------------------------------
void printHelp() {
  Serial.println("\n╔════════════════════════════════════════════════════════╗");
  Serial.println("║         SCARA MOTOR - SISTEMA DE CONTROL              ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println("\n📍 COMANDOS DE MOVIMIENTO:");
  Serial.println("  [número]  → Mover a ángulo específico (ej: 45, 67.5, 90)");
  Serial.println("  stop / s  → DETENER movimiento inmediatamente");
  Serial.println("  0 / home  → Volver a posición cero (20°)");
  Serial.println("  +         → Paso pequeño adelante");
  Serial.println("  -         → Paso pequeño atrás");
  Serial.println();
  Serial.println("⚙️  CONFIGURACIÓN:");
  Serial.println("  r / reset → Reiniciar encoder (establecer posición actual como 20°)");
  Serial.println("  p[XXX]    → Establecer PWM base (ej: p180)");
  Serial.println("  kp[X.X]   → Ajustar ganancia P del PID (ej: kp2.5)");
  Serial.println("  ki[X.X]   → Ajustar ganancia I del PID (ej: ki0.2)");
  Serial.println("  kd[X.X]   → Ajustar ganancia D del PID (ej: kd0.8)");
  Serial.println();
  Serial.println("ℹ️  INFORMACIÓN:");
  Serial.println("  help / h  → Mostrar esta ayuda");
  Serial.println("  status    → Ver estado actual del sistema");
  Serial.println("  debug on  → Activar modo debug del PID");
  Serial.println("  debug off → Desactivar modo debug");
  Serial.println("  verbose on  → Mostrar progreso detallado");
  Serial.println("  verbose off → Progreso simplificado");
  Serial.println();
  Serial.println("📊 LÍMITES DEL SISTEMA:");
  Serial.print("  Rango: ");
  Serial.print(MIN_ANGLE);
  Serial.print("° - ");
  Serial.print(MAX_ANGLE);
  Serial.println("°");
  Serial.print("  PWM: ");
  Serial.print(MIN_PWM);
  Serial.print(" - ");
  Serial.println(MAX_PWM);
  Serial.print("  Precisión: ±");
  Serial.print(TOLERANCE / TICKS_PER_DEGREE, 1);
  Serial.println("°");
  Serial.println("════════════════════════════════════════════════════════");
}

void printStatus() {
  noInterrupts();
  long ticks = encoderCount;
  interrupts();
  
  Serial.println("\n╔════════════════════════════════════════════════════════╗");
  Serial.println("║              ESTADO DEL SISTEMA                        ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.print("  Posición: ");
  Serial.print(ticksToDegrees(ticks), 2);
  Serial.print("° (");
  Serial.print(ticks);
  Serial.println(" ticks)");
  Serial.print("  Motor: ");
  Serial.println(motorRunning ? "EN MOVIMIENTO" : "DETENIDO");
  Serial.print("  PWM Base: ");
  Serial.println(MIN_PWM);
  Serial.println("\n  Parámetros PID:");
  Serial.print("    Kp = ");
  Serial.println(Kp, 2);
  Serial.print("    Ki = ");
  Serial.println(Ki, 3);
  Serial.print("    Kd = ");
  Serial.println(Kd, 2);
  Serial.print("  Debug: ");
  Serial.println(debugMode ? "ACTIVADO" : "DESACTIVADO");
  Serial.print("  Verbose: ");
  Serial.println(verboseMode ? "ACTIVADO" : "DESACTIVADO");
  Serial.println("════════════════════════════════════════════════════════\n");
}

// ---------------------------------------
// Movimientos pequeños
// ---------------------------------------
void smallStepForward() {
  Serial.println("→ Paso adelante (+)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  setMotorSpeed(MIN_PWM);
  delay(100);
  motorStop();
}

void smallStepBackward() {
  Serial.println("← Paso atrás (-)");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  setMotorSpeed(MIN_PWM);
  delay(100);
  motorStop();
}

// ---------------------------------------
// Procesador de Comandos
// ---------------------------------------
void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  
  if (cmd.length() == 0) return;
  
  // Comandos de ayuda
  if (cmd == "help" || cmd == "h" || cmd == "?") {
    printHelp();
  }
  // Estado
  else if (cmd == "status" || cmd == "info") {
    printStatus();
  }
  // Stop
  else if (cmd == "stop" || cmd == "s") {
    emergencyStop = true;
    motorStop();
    Serial.println("⚠ MOTOR DETENIDO");
  }
  // Home
  else if (cmd == "0" || cmd == "home") {
    goToAngle(MIN_ANGLE);  // Ir a posición mínima (45°)
  }
  // Reset encoder
  else if (cmd == "r" || cmd == "reset") {
    noInterrupts();
    encoderCount = 0;
    interrupts();
    Serial.print("✓ Encoder reiniciado (posición actual = ");
    Serial.print(REFERENCE_ANGLE);
    Serial.println("°)");
  }
  // Pasos pequeños
  else if (cmd == "+") {
    smallStepForward();
  }
  else if (cmd == "-") {
    smallStepBackward();
  }
  // Debug
  else if (cmd == "debug on") {
    debugMode = true;
    Serial.println("✓ Modo debug ACTIVADO");
  }
  else if (cmd == "debug off") {
    debugMode = false;
    Serial.println("✓ Modo debug DESACTIVADO");
  }
  // Verbose
  else if (cmd == "verbose on") {
    verboseMode = true;
    Serial.println("✓ Modo verbose ACTIVADO");
  }
  else if (cmd == "verbose off") {
    verboseMode = false;
    Serial.println("✓ Modo verbose DESACTIVADO");
  }
  // PWM
  else if (cmd.startsWith("p")) {
    int newPwm = cmd.substring(1).toInt();
    if (newPwm >= MIN_PWM && newPwm <= MAX_PWM) {
      // Nota: MIN_PWM es const, usar una variable global si se necesita cambiar
      Serial.print("✓ PWM base: ");
      Serial.println(newPwm);
    } else {
      Serial.println("✗ Error: PWM fuera de rango");
    }
  }
  // PID Kp
  else if (cmd.startsWith("kp")) {
    float newKp = cmd.substring(2).toFloat();
    if (newKp >= 0 && newKp <= 10) {
      Kp = newKp;
      Serial.print("✓ Kp = ");
      Serial.println(Kp, 2);
    }
  }
  // PID Ki
  else if (cmd.startsWith("ki")) {
    float newKi = cmd.substring(2).toFloat();
    if (newKi >= 0 && newKi <= 5) {
      Ki = newKi;
      Serial.print("✓ Ki = ");
      Serial.println(Ki, 3);
    }
  }
  // PID Kd
  else if (cmd.startsWith("kd")) {
    float newKd = cmd.substring(2).toFloat();
    if (newKd >= 0 && newKd <= 5) {
      Kd = newKd;
      Serial.print("✓ Kd = ");
      Serial.println(Kd, 2);
    }
  }
  // Mover a ángulo
  else {
    float targetAngle = cmd.toFloat();
    if (targetAngle > 0 || cmd == "0") {
      goToAngle(targetAngle);
    } else {
      Serial.print("✗ Comando no reconocido: '");
      Serial.print(cmd);
      Serial.println("'");
      Serial.println("  Escribe 'help' para ver comandos disponibles");
    }
  }
}

// ---------------------------------------
// Setup
// ---------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  lastEncA = digitalRead(ENC_A);
  lastEncB = digitalRead(ENC_B);

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  motorStop();

  // Mensaje de bienvenida
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.println("║     SCARA MOTOR - SISTEMA DE CONTROL INICIADO         ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println("\n✓ Sistema listo. Escribe 'help' para ver comandos.\n");
  
  printStatus();
}

// ---------------------------------------
// Loop
// ---------------------------------------
void loop() {
  // Imprimir posición cada 2 segundos (si no está en debug)
  if (!debugMode && millis() - lastPrint >= 2000) {
    lastPrint = millis();
    noInterrupts();
    long val = encoderCount;
    interrupts();
    
    Serial.print("📍 Posición: ");
    Serial.print(ticksToDegrees(val), 1);
    Serial.print("° (");
    Serial.print(val);
    Serial.println(" ticks)");
  }

  // Procesar comandos
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}