#include <Arduino.h> 

// ---------------------------
// Pines del Motor 2 (Potencia y Control L298N)
// ---------------------------
#define IN3 14    // L298N IN3 -> GPIO 14
#define IN4 12    // L298N IN4 -> GPIO 12
#define ENB 13    // L298N ENB (PWM) -> GPIO 13

// ---------------------------
// Pines ENCODER 2
// ---------------------------
#define ENC_A 32  // Encoder Fase A (Interrupción)
#define ENC_B 33  // Encoder Fase B (Lectura de Dirección)

volatile long encoderCount = 0;
unsigned long lastPrint = 0;

// --- Configuración de PWM ---
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int MAX_PWM = 255;
const int MIN_PWM = 150;  // PWM mínimo para Motor 2

// --- Límites de Seguridad MOTOR 2 ---
const float MIN_ANGLE = 55.0;   // Ángulo mínimo permitido (90 - 35)
const float MAX_ANGLE = 125.0;  // Ángulo máximo permitido (90 + 35)
const long TOLERANCE = 15;      // Tolerancia en ticks (±5 grados aprox)

// --- Calibración de Grados MOTOR 2 ---
const float TICKS_PER_DEGREE = 2.743;  // Calculado: 96 ticks / 35 grados
const float REFERENCE_ANGLE = 90.0;    // Posición de referencia al resetear

// --- Control PID (Optimizado para movimiento SUAVE) ---
float Kp = 0.8;   // Ganancia proporcional
float Ki = 0.02;  // Ganancia integral
float Kd = 1.2;   // Ganancia derivativa

float lastError = 0;
float integral = 0;
unsigned long lastPIDTime = 0;

// --- Variables de Estado ---
volatile int lastEncA = 0;
volatile int lastEncB = 0;
bool motorRunning = false;
bool debugMode = false;
bool emergencyStop = false;
bool verboseMode = true;
int currentPWM = 0;

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
// ISR del Encoder Motor 2
// ---------------------------------------
void IRAM_ATTR encoderISR() {
  int currentA = digitalRead(ENC_A);
  int currentB = digitalRead(ENC_B);
  
  // NOTA: Puede que necesites invertir esta lógica según tu cableado
  // Si el encoder cuenta al revés, intercambia los ++ y --
  if (lastEncA == LOW && currentA == HIGH) {
    if (currentB == LOW) {
      encoderCount--;  
    } else {
      encoderCount++;  
    }
  }
  else if (lastEncA == HIGH && currentA == LOW) {
    if (currentB == HIGH) {
      encoderCount--;  
    } else {
      encoderCount++;  
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
  ledcWrite(ENB, speed);
}

void motorStop() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  setMotorSpeed(0);
  motorRunning = false;
  emergencyStop = false;
  currentPWM = 0;
  
  // Reset PID
  lastError = 0;
  integral = 0;
}

void moveMotor(int direction, int speed) {
  if (direction > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (direction < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
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
  
  float output = P + I + D;
  
  // Mapeo SUAVE con velocidades más bajas
  int targetPWM;
  float absError = abs(error);
  
  if (absError > TOLERANCE * 4) {
    targetPWM = map(constrain(abs(output), 0, 150), 0, 150, MIN_PWM, MIN_PWM + 40);
  } else if (absError > TOLERANCE * 2) {
    targetPWM = map(constrain(abs(output), 0, 80), 0, 80, MIN_PWM, MIN_PWM + 20);
  } else {
    targetPWM = MIN_PWM;
  }
  
  // RAMPA DE ACELERACIÓN SUAVE
  int pwm;
  if (targetPWM > currentPWM) {
    currentPWM += 3;
    if (currentPWM > targetPWM) currentPWM = targetPWM;
  } else if (targetPWM < currentPWM) {
    currentPWM -= 5;
    if (currentPWM < targetPWM) currentPWM = targetPWM;
  }
  pwm = currentPWM;
  
  pwm = constrain(pwm, MIN_PWM, MIN_PWM + 50);
  
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
  currentPWM = 0;
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
    if (emergencyStop) {
      motorStop();
      Serial.println("⚠ MOVIMIENTO DETENIDO POR USUARIO");
      return false;
    }
    
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
    
    int pwm = calculatePID(currentTicks, targetTicks);
    
    if (currentTicks > targetTicks) {
      moveMotor(-1, pwm);
    } else {
      moveMotor(1, pwm);
    }
    
    if (verboseMode && (progressCounter++ % 5 == 0)) {
      float currentAngle = ticksToDegrees(currentTicks);
      float progress = 100.0 * (1.0 - abs(currentTicks - targetTicks) / (float)abs(lastTicks - targetTicks));
      progress = constrain(progress, 0, 100);
      
      Serial.print("  ");
      Serial.print(currentAngle, 1);
      Serial.print("° → ");
      Serial.print(targetDegrees, 1);
      Serial.print("° [");
      
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
  Serial.println("║      SCARA MOTOR 2 - SISTEMA DE CONTROL               ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println("\n📍 COMANDOS DE MOVIMIENTO:");
  Serial.println("  [número]  → Mover a ángulo específico (ej: 55, 90, 125)");
  Serial.println("  stop / s  → DETENER movimiento inmediatamente");
  Serial.println("  0 / home  → Volver a posición central (90°)");
  Serial.println("  +         → Paso pequeño adelante");
  Serial.println("  -         → Paso pequeño atrás");
  Serial.println();
  Serial.println("⚙️  CONFIGURACIÓN:");
  Serial.println("  r / reset → Reiniciar encoder (establecer posición actual como 90°)");
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
  Serial.println("📊 LÍMITES DEL SISTEMA (MOTOR 2):");
  Serial.print("  Rango: ");
  Serial.print(MIN_ANGLE);
  Serial.print("° - ");
  Serial.print(MAX_ANGLE);
  Serial.print("° (±35° desde 90°)");
  Serial.println();
  Serial.print("  PWM: ");
  Serial.print(MIN_PWM);
  Serial.print(" - ");
  Serial.println(MAX_PWM);
  Serial.print("  Calibración: ");
  Serial.print(TICKS_PER_DEGREE, 3);
  Serial.println(" ticks/grado");
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
  Serial.println("║           ESTADO DEL SISTEMA (MOTOR 2)                ║");
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
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setMotorSpeed(MIN_PWM);
  delay(100);
  motorStop();
}

void smallStepBackward() {
  Serial.println("← Paso atrás (-)");
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
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
  
  if (cmd == "help" || cmd == "h" || cmd == "?") {
    printHelp();
  }
  else if (cmd == "status" || cmd == "info") {
    printStatus();
  }
  else if (cmd == "stop" || cmd == "s") {
    emergencyStop = true;
    motorStop();
    Serial.println("⚠ MOTOR DETENIDO");
  }
  else if (cmd == "0" || cmd == "home") {
    goToAngle(REFERENCE_ANGLE);  // Ir a 90°
  }
  else if (cmd == "r" || cmd == "reset") {
    noInterrupts();
    encoderCount = 0;
    interrupts();
    Serial.print("✓ Encoder reiniciado (posición actual = ");
    Serial.print(REFERENCE_ANGLE);
    Serial.println("°)");
  }
  else if (cmd == "+") {
    smallStepForward();
  }
  else if (cmd == "-") {
    smallStepBackward();
  }
  else if (cmd == "debug on") {
    debugMode = true;
    Serial.println("✓ Modo debug ACTIVADO");
  }
  else if (cmd == "debug off") {
    debugMode = false;
    Serial.println("✓ Modo debug DESACTIVADO");
  }
  else if (cmd == "verbose on") {
    verboseMode = true;
    Serial.println("✓ Modo verbose ACTIVADO");
  }
  else if (cmd == "verbose off") {
    verboseMode = false;
    Serial.println("✓ Modo verbose DESACTIVADO");
  }
  else if (cmd.startsWith("p")) {
    int newPwm = cmd.substring(1).toInt();
    if (newPwm >= MIN_PWM && newPwm <= MAX_PWM) {
      Serial.print("✓ PWM base: ");
      Serial.println(newPwm);
    } else {
      Serial.println("✗ Error: PWM fuera de rango");
    }
  }
  else if (cmd.startsWith("kp")) {
    float newKp = cmd.substring(2).toFloat();
    if (newKp >= 0 && newKp <= 10) {
      Kp = newKp;
      Serial.print("✓ Kp = ");
      Serial.println(Kp, 2);
    }
  }
  else if (cmd.startsWith("ki")) {
    float newKi = cmd.substring(2).toFloat();
    if (newKi >= 0 && newKi <= 5) {
      Ki = newKi;
      Serial.print("✓ Ki = ");
      Serial.println(Ki, 3);
    }
  }
  else if (cmd.startsWith("kd")) {
    float newKd = cmd.substring(2).toFloat();
    if (newKd >= 0 && newKd <= 5) {
      Kd = newKd;
      Serial.print("✓ Kd = ");
      Serial.println(Kd, 2);
    }
  }
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

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  lastEncA = digitalRead(ENC_A);
  lastEncB = digitalRead(ENC_B);

  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  motorStop();

  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.println("║   SCARA MOTOR 2 - SISTEMA DE CONTROL INICIADO         ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println("\n✓ Sistema listo. Escribe 'help' para ver comandos.\n");
  
  printStatus();
}

// ---------------------------------------
// Loop
// ---------------------------------------
void loop() {
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

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}