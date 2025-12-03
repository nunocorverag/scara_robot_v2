#include <Arduino.h> 

// ============================================
// MOTOR 1 - Configuración
// ============================================
#define M1_IN1 25
#define M1_IN2 26
#define M1_ENA 27
#define M1_ENC_A 34
#define M1_ENC_B 35

const float M1_MIN_ANGLE = 75.0;
const float M1_MAX_ANGLE = 105.0;
const float M1_TICKS_PER_DEGREE = 3.157;
const float M1_REFERENCE_ANGLE = 90.0;
const int M1_MIN_PWM = 160;
const float M1_TOLERANCE_MARGIN = 10.0;

// ============================================
// MOTOR 2 - Configuración
// ============================================
#define M2_IN3 14
#define M2_IN4 12
#define M2_ENB 13
#define M2_ENC_A 32
#define M2_ENC_B 33

const float M2_MIN_ANGLE = 75.0;
const float M2_MAX_ANGLE = 105.0;
const float M2_TICKS_PER_DEGREE = 2.743;
const float M2_REFERENCE_ANGLE = 90.0;
const int M2_MIN_PWM = 150;
const float M2_TOLERANCE_MARGIN = 10.0;

// ============================================
// Variables Globales
// ============================================
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int MAX_PWM = 255;
const long TOLERANCE = 15;

// Encoders
volatile long m1_encoderCount = 0;
volatile long m2_encoderCount = 0;
volatile int m1_lastEncA = 0;
volatile int m1_lastEncB = 0;
volatile int m2_lastEncA = 0;
volatile int m2_lastEncB = 0;

// Control PID Motor 1
float m1_Kp = 0.8, m1_Ki = 0.02, m1_Kd = 1.2;
float m1_lastError = 0, m1_integral = 0;
unsigned long m1_lastPIDTime = 0;
int m1_currentPWM = 0;

// Control PID Motor 2
float m2_Kp = 0.8, m2_Ki = 0.02, m2_Kd = 1.2;
float m2_lastError = 0, m2_integral = 0;
unsigned long m2_lastPIDTime = 0;
int m2_currentPWM = 0;

// Estado
bool emergencyStop = false;
bool verboseMode = true;
unsigned long lastPrint = 0;

// ============================================
// Funciones de Conversión
// ============================================
float m1_ticksToDegrees(long ticks) {
  return (ticks / M1_TICKS_PER_DEGREE) + M1_REFERENCE_ANGLE;
}

long m1_degreesToTicks(float degrees) {
  return (long)((degrees - M1_REFERENCE_ANGLE) * M1_TICKS_PER_DEGREE);
}

float m2_ticksToDegrees(long ticks) {
  return (ticks / M2_TICKS_PER_DEGREE) + M2_REFERENCE_ANGLE;
}

long m2_degreesToTicks(float degrees) {
  return (long)((degrees - M2_REFERENCE_ANGLE) * M2_TICKS_PER_DEGREE);
}

// ============================================
// ISR Encoders
// ============================================
void IRAM_ATTR m1_encoderISR() {
  int currentA = digitalRead(M1_ENC_A);
  int currentB = digitalRead(M1_ENC_B);
  
  if (m1_lastEncA == LOW && currentA == HIGH) {
    if (currentB == LOW) m1_encoderCount--;
    else m1_encoderCount++;
  }
  else if (m1_lastEncA == HIGH && currentA == LOW) {
    if (currentB == HIGH) m1_encoderCount--;
    else m1_encoderCount++;
  }
  
  m1_lastEncA = currentA;
  m1_lastEncB = currentB;
}

void IRAM_ATTR m2_encoderISR() {
  int currentA = digitalRead(M2_ENC_A);
  int currentB = digitalRead(M2_ENC_B);
  
  if (m2_lastEncA == LOW && currentA == HIGH) {
    if (currentB == LOW) m2_encoderCount--;
    else m2_encoderCount++;
  }
  else if (m2_lastEncA == HIGH && currentA == LOW) {
    if (currentB == HIGH) m2_encoderCount--;
    else m2_encoderCount++;
  }
  
  m2_lastEncA = currentA;
  m2_lastEncB = currentB;
}

// ============================================
// Control de Motores
// ============================================
void m1_setSpeed(int speed) {
  ledcWrite(M1_ENA, constrain(speed, 0, MAX_PWM));
}

void m2_setSpeed(int speed) {
  ledcWrite(M2_ENB, constrain(speed, 0, MAX_PWM));
}

void m1_stop() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  m1_setSpeed(0);
  m1_currentPWM = 0;
  m1_lastError = 0;
  m1_integral = 0;
}

void m2_stop() {
  digitalWrite(M2_IN3, LOW);
  digitalWrite(M2_IN4, LOW);
  m2_setSpeed(0);
  m2_currentPWM = 0;
  m2_lastError = 0;
  m2_integral = 0;
}

void stopAllMotors() {
  m1_stop();
  m2_stop();
  emergencyStop = false;
}

void m1_move(int direction, int speed) {
  if (direction > 0) {
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
  } else if (direction < 0) {
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
  } else {
    m1_stop();
    return;
  }
  m1_setSpeed(speed);
}

void m2_move(int direction, int speed) {
  if (direction > 0) {
    digitalWrite(M2_IN3, HIGH);
    digitalWrite(M2_IN4, LOW);
  } else if (direction < 0) {
    digitalWrite(M2_IN3, LOW);
    digitalWrite(M2_IN4, HIGH);
  } else {
    m2_stop();
    return;
  }
  m2_setSpeed(speed);
}

// ============================================
// Control PID
// ============================================
int m1_calculatePID(long currentTicks, long targetTicks) {
  unsigned long now = millis();
  float deltaTime = (now - m1_lastPIDTime) / 1000.0;
  if (deltaTime <= 0) deltaTime = 0.01;
  
  float error = targetTicks - currentTicks;
  float P = m1_Kp * error;
  
  m1_integral += error * deltaTime;
  m1_integral = constrain(m1_integral, -30, 30);
  float I = m1_Ki * m1_integral;
  
  float derivative = (error - m1_lastError) / deltaTime;
  float D = m1_Kd * derivative;
  
  float output = P + I + D;
  float absError = abs(error);
  
  int targetPWM;
  if (absError > TOLERANCE * 4) {
    targetPWM = map(constrain(abs(output), 0, 150), 0, 150, M1_MIN_PWM, M1_MIN_PWM + 40);
  } else if (absError > TOLERANCE * 2) {
    targetPWM = map(constrain(abs(output), 0, 80), 0, 80, M1_MIN_PWM, M1_MIN_PWM + 20);
  } else {
    targetPWM = M1_MIN_PWM;
  }
  
  if (targetPWM > m1_currentPWM) {
    m1_currentPWM += 3;
    if (m1_currentPWM > targetPWM) m1_currentPWM = targetPWM;
  } else if (targetPWM < m1_currentPWM) {
    m1_currentPWM -= 5;
    if (m1_currentPWM < targetPWM) m1_currentPWM = targetPWM;
  }
  
  int pwm = constrain(m1_currentPWM, M1_MIN_PWM, M1_MIN_PWM + 50);
  
  m1_lastError = error;
  m1_lastPIDTime = now;
  
  return pwm;
}

int m2_calculatePID(long currentTicks, long targetTicks) {
  unsigned long now = millis();
  float deltaTime = (now - m2_lastPIDTime) / 1000.0;
  if (deltaTime <= 0) deltaTime = 0.01;
  
  float error = targetTicks - currentTicks;
  float P = m2_Kp * error;
  
  m2_integral += error * deltaTime;
  m2_integral = constrain(m2_integral, -30, 30);
  float I = m2_Ki * m2_integral;
  
  float derivative = (error - m2_lastError) / deltaTime;
  float D = m2_Kd * derivative;
  
  float output = P + I + D;
  float absError = abs(error);
  
  int targetPWM;
  if (absError > TOLERANCE * 4) {
    targetPWM = map(constrain(abs(output), 0, 150), 0, 150, M2_MIN_PWM, M2_MIN_PWM + 40);
  } else if (absError > TOLERANCE * 2) {
    targetPWM = map(constrain(abs(output), 0, 80), 0, 80, M2_MIN_PWM, M2_MIN_PWM + 20);
  } else {
    targetPWM = M2_MIN_PWM;
  }
  
  if (targetPWM > m2_currentPWM) {
    m2_currentPWM += 3;
    if (m2_currentPWM > targetPWM) m2_currentPWM = targetPWM;
  } else if (targetPWM < m2_currentPWM) {
    m2_currentPWM -= 5;
    if (m2_currentPWM < targetPWM) m2_currentPWM = targetPWM;
  }
  
  int pwm = constrain(m2_currentPWM, M2_MIN_PWM, M2_MIN_PWM + 50);
  
  m2_lastError = error;
  m2_lastPIDTime = now;
  
  return pwm;
}

// ============================================
// Movimiento Coordinado
// ============================================
bool goToPosition(float m1_targetDegrees, float m2_targetDegrees) {
  // Validar límites
  if (m1_targetDegrees < M1_MIN_ANGLE - M1_TOLERANCE_MARGIN || 
      m1_targetDegrees > M1_MAX_ANGLE + M1_TOLERANCE_MARGIN) {
    Serial.println("❌ ERROR: Motor 1 fuera de rango!");
    return false;
  }
  if (m2_targetDegrees < M2_MIN_ANGLE - M2_TOLERANCE_MARGIN || 
      m2_targetDegrees > M2_MAX_ANGLE + M2_TOLERANCE_MARGIN) {
    Serial.println("❌ ERROR: Motor 2 fuera de rango!");
    return false;
  }
  
  long m1_targetTicks = m1_degreesToTicks(m1_targetDegrees);
  long m2_targetTicks = m2_degreesToTicks(m2_targetDegrees);
  
  emergencyStop = false;
  m1_lastError = 0; m1_integral = 0; m1_currentPWM = 0;
  m2_lastError = 0; m2_integral = 0; m2_currentPWM = 0;
  m1_lastPIDTime = m2_lastPIDTime = millis();
  
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║          MOVIMIENTO COORDINADO INICIADO             ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.print("  Motor 1: "); Serial.print(m1_ticksToDegrees(m1_encoderCount), 1);
  Serial.print("° → "); Serial.print(m1_targetDegrees, 1); Serial.println("°");
  Serial.print("  Motor 2: "); Serial.print(m2_ticksToDegrees(m2_encoderCount), 1);
  Serial.print("° → "); Serial.print(m2_targetDegrees, 1); Serial.println("°");
  
  unsigned long startTime = millis();
  int progressCounter = 0;
  
  while (true) {
    // Verificar stop
    if (emergencyStop || Serial.available()) {
      if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "stop" || cmd == "s") {
          stopAllMotors();
          Serial.println("\n⚠  MOVIMIENTO DETENIDO POR USUARIO");
          return false;
        }
      }
    }
    
    noInterrupts();
    long m1_current = m1_encoderCount;
    long m2_current = m2_encoderCount;
    interrupts();
    
    // Verificar si ambos llegaron
    bool m1_arrived = abs(m1_current - m1_targetTicks) <= TOLERANCE;
    bool m2_arrived = abs(m2_current - m2_targetTicks) <= TOLERANCE;
    
    if (m1_arrived && m2_arrived) {
      break;
    }
    
    // Control Motor 1
    if (!m1_arrived) {
      int pwm1 = m1_calculatePID(m1_current, m1_targetTicks);
      m1_move((m1_current > m1_targetTicks) ? -1 : 1, pwm1);
    } else {
      m1_stop();
    }
    
    // Control Motor 2
    if (!m2_arrived) {
      int pwm2 = m2_calculatePID(m2_current, m2_targetTicks);
      m2_move((m2_current > m2_targetTicks) ? -1 : 1, pwm2);
    } else {
      m2_stop();
    }
    
    // Progreso
    if (verboseMode && (progressCounter++ % 10 == 0)) {
      float m1_angle = m1_ticksToDegrees(m1_current);
      float m2_angle = m2_ticksToDegrees(m2_current);
      
      Serial.print("  M1: "); Serial.print(m1_angle, 1); Serial.print("° ");
      Serial.print(m1_arrived ? "✓" : "→");
      Serial.print(" | M2: "); Serial.print(m2_angle, 1); Serial.print("° ");
      Serial.println(m2_arrived ? "✓" : "→");
    }
    
    delay(10);
  }
  
  stopAllMotors();
  
  unsigned long elapsed = millis() - startTime;
  
  Serial.println("\n✅ POSICIÓN ALCANZADA!");
  Serial.print("  Tiempo: "); Serial.print(elapsed / 1000.0, 1); Serial.println(" s");
  Serial.print("  Motor 1: "); Serial.print(m1_ticksToDegrees(m1_encoderCount), 2); Serial.println("°");
  Serial.print("  Motor 2: "); Serial.print(m2_ticksToDegrees(m2_encoderCount), 2); Serial.println("°");
  
  return true;
}

// ============================================
// Movimiento Secuencial (Motor por Motor)
// ============================================
bool goToPositionSequential(float m1_target, float m2_target) {
  Serial.println("  ⚙  Modo SECUENCIAL activado (evitar colisión)");
  
  // Primero mover Motor 2
  Serial.print("  🟢 Paso 1: Motor 2 → "); Serial.print(m2_target, 1); Serial.println("°");
  if (!goToPosition(m1_ticksToDegrees(m1_encoderCount), m2_target)) return false;
  delay(500);
  
  // Luego mover Motor 1
  Serial.print("  🔵 Paso 2: Motor 1 → "); Serial.print(m1_target, 1); Serial.println("°");
  if (!goToPosition(m1_target, m2_ticksToDegrees(m2_encoderCount))) return false;
  
  return true;
}

// ============================================
// Comandos de Mapeo de Cuadrantes
// ============================================
void mapCorners() {
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║         MAPEO DE CUADRANTES - 4 ESQUINAS            ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.println("\nEste proceso visitará las 4 esquinas del espacio:");
  Serial.println("  1️⃣  Esquina INFERIOR IZQUIERDA (ambos mínimos)");
  Serial.println("  2️⃣  Esquina INFERIOR DERECHA (M1 máx, M2 mín)");
  Serial.println("  3️⃣  Esquina SUPERIOR IZQUIERDA (M1 mín, M2 máx)"); // c4 antes de c3
  Serial.println("  4️⃣  Esquina SUPERIOR DERECHA (ambos máximos)"); // c3 después de c4
  Serial.println("\n⚠  Presiona 's' para detener en cualquier momento");
  Serial.println("════════════════════════════════════════════════════════\n");
  
  delay(2000);
  
  // Primero ir al centro para posición segura
  Serial.println("🏠 Posicionando en centro (posición segura)...");
  if (!goToPosition(M1_REFERENCE_ANGLE, M2_REFERENCE_ANGLE)) return;
  delay(1000);
  
  // Esquina 1: Ambos mínimos (SECUENCIAL)
  Serial.println("\n1️⃣  ESQUINA INFERIOR IZQUIERDA");
  Serial.print("   (M1="); Serial.print(M1_MIN_ANGLE); 
  Serial.print("°, M2="); Serial.print(M2_MIN_ANGLE); Serial.println("°)");
  if (!goToPositionSequential(M1_MIN_ANGLE, M2_MIN_ANGLE)) return;
  delay(1500);
  
  // Esquina 2: M1 máximo, M2 mínimo
  Serial.println("\n2️⃣  ESQUINA INFERIOR DERECHA");
  Serial.print("   (M1="); Serial.print(M1_MAX_ANGLE); 
  Serial.print("°, M2="); Serial.print(M2_MIN_ANGLE); Serial.println("°)");
  if (!goToPosition(M1_MAX_ANGLE, M2_MIN_ANGLE)) return;
  delay(1500);
  
  // Esquina 4: M1 mínimo, M2 máximo (NUEVO ORDEN)
  Serial.println("\n3️⃣  ESQUINA SUPERIOR IZQUIERDA");
  Serial.print("   (M1="); Serial.print(M1_MIN_ANGLE); 
  Serial.print("°, M2="); Serial.print(120.0); Serial.println("°)"); // Usamos 120.0 como objetivo
  if (!goToPosition(M1_MIN_ANGLE, 120.0)) return;
  delay(1500);

// Esquina 3: Ambos máximos (SECUENCIAL) (NUEVO ORDEN)
  Serial.println("\n4️⃣  ESQUINA SUPERIOR DERECHA");
  Serial.print("   (M1="); Serial.print(M1_MAX_ANGLE); 
  Serial.print("°, M2="); Serial.print(120.0); Serial.println("°)"); // Usamos 120.0 como objetivo
  if (!goToPositionSequential(M1_MAX_ANGLE, 120.0)) return;
  delay(1500);
  
  // Volver al centro
  Serial.println("\n🏠 REGRESANDO AL CENTRO");
  goToPosition(M1_REFERENCE_ANGLE, M2_REFERENCE_ANGLE);
  
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║          MAPEO DE CUADRANTES COMPLETADO             ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
}

// ============================================
// Ayuda y Estado
// ============================================
void printHelp() {
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║      SCARA DUAL MOTOR - SISTEMA DE CONTROL           ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.println("\n🎯 COMANDOS DE MAPEO:");
  Serial.println("  map       → Recorrer las 4 esquinas del cuadrante");
  Serial.println("  c1        → Ir a esquina 1 (M1 mín, M2 mín)");
  Serial.println("  c2        → Ir a esquina 2 (M1 máx, M2 mín)");
  Serial.println("  c3        → Ir a esquina 3 (M1 máx, M2 máx)");
  Serial.println("  c4        → Ir a esquina 4 (M1 mín, M2 máx)");
  Serial.println("\n📍 COMANDOS DE MOVIMIENTO:");
  Serial.println("  [ángulo1] [ángulo2]  → Mover ambos motores");
  Serial.println("                          Ejemplo: 60 80");
  Serial.println("  home / 0             → Volver al centro (90°, 90°)");
  Serial.println("  stop / s             → DETENER movimiento");
  Serial.println("\n⚙  CONFIGURACIÓN:");
  Serial.println("  r1 / r2    → Reset encoder Motor 1 / Motor 2");
  Serial.println("  reset      → Reset ambos encoders");
  Serial.println("\nℹ  INFORMACIÓN:");
  Serial.println("  help / h   → Mostrar esta ayuda");
  Serial.println("  status     → Ver estado de ambos motores");
  Serial.println("\n📊 LÍMITES:");
  Serial.print("  Motor 1: "); Serial.print(M1_MIN_ANGLE); 
  Serial.print("° - "); Serial.print(M1_MAX_ANGLE); Serial.println("°");
  Serial.print("  Motor 2: "); Serial.print(M2_MIN_ANGLE); 
  Serial.print("° - "); Serial.print(M2_MAX_ANGLE); Serial.println("°");
  Serial.println("════════════════════════════════════════════════════════");
}

void printStatus() {
  noInterrupts();
  long m1_ticks = m1_encoderCount;
  long m2_ticks = m2_encoderCount;
  interrupts();
  
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║              ESTADO DEL SISTEMA                      ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.println("\n🔵 MOTOR 1:");
  Serial.print("  Posición: "); Serial.print(m1_ticksToDegrees(m1_ticks), 2);
  Serial.print("° ("); Serial.print(m1_ticks); Serial.println(" ticks)");
  Serial.print("  Rango: "); Serial.print(M1_MIN_ANGLE); 
  Serial.print("° - "); Serial.print(M1_MAX_ANGLE); Serial.println("°");
  
  Serial.println("\n🟢 MOTOR 2:");
  Serial.print("  Posición: "); Serial.print(m2_ticksToDegrees(m2_ticks), 2);
  Serial.print("° ("); Serial.print(m2_ticks); Serial.println(" ticks)");
  Serial.print("  Rango: "); Serial.print(M2_MIN_ANGLE); 
  Serial.print("° - "); Serial.print(M2_MAX_ANGLE); Serial.println("°");
  Serial.println("════════════════════════════════════════════════════════\n");
}

// ============================================
// Procesador de Comandos
// ============================================
void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  
  if (cmd.length() == 0) return;
  
  if (cmd == "help" || cmd == "h") {
    printHelp();
  }
  else if (cmd == "status") {
    printStatus();
  }
  else if (cmd == "stop" || cmd == "s") {
    stopAllMotors();
    Serial.println("⚠  MOTORES DETENIDOS");
  }
  else if (cmd == "map") {
    mapCorners();
  }
  else if (cmd == "c1") {
    goToPositionSequential(M1_MIN_ANGLE, M2_MIN_ANGLE);
  }
  else if (cmd == "c2") {
    goToPosition(M1_MAX_ANGLE, M2_MIN_ANGLE);
  }
  else if (cmd == "c3") {
    goToPositionSequential(M1_MAX_ANGLE, M2_MAX_ANGLE);
  }
  else if (cmd == "c4") {
    goToPosition(M1_MIN_ANGLE, M2_MAX_ANGLE);
  }
  else if (cmd == "home" || cmd == "0") {
    goToPosition(M1_REFERENCE_ANGLE, M2_REFERENCE_ANGLE);
  }
  else if (cmd == "r1") {
    noInterrupts();
    m1_encoderCount = 0;
    interrupts();
    Serial.println("✓ Encoder Motor 1 reiniciado");
  }
  else if (cmd == "r2") {
    noInterrupts();
    m2_encoderCount = 0;
    interrupts();
    Serial.println("✓ Encoder Motor 2 reiniciado");
  }
  else if (cmd == "reset") {
    noInterrupts();
    m1_encoderCount = 0;
    m2_encoderCount = 0;
    interrupts();
    Serial.println("✓ Ambos encoders reiniciados");
  }
  else {
    // Intentar parsear como dos ángulos
    int spaceIndex = cmd.indexOf(' ');
    if (spaceIndex > 0) {
      float angle1 = cmd.substring(0, spaceIndex).toFloat();
      float angle2 = cmd.substring(spaceIndex + 1).toFloat();
      goToPosition(angle1, angle2);
    } else {
      Serial.print("✗ Comando no reconocido: '");
      Serial.print(cmd);
      Serial.println("'");
      Serial.println("  Escribe 'help' para ver comandos disponibles");
    }
  }
}

// ============================================
// Setup
// ============================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Motor 1
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M1_ENC_B, INPUT_PULLUP);
  m1_lastEncA = digitalRead(M1_ENC_A);
  m1_lastEncB = digitalRead(M1_ENC_B);
  ledcAttach(M1_ENA, PWM_FREQ, PWM_RESOLUTION);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), m1_encoderISR, CHANGE);
  
  // Motor 2
  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_B, INPUT_PULLUP);
  m2_lastEncA = digitalRead(M2_ENC_A);
  m2_lastEncB = digitalRead(M2_ENC_B);
  ledcAttach(M2_ENB, PWM_FREQ, PWM_RESOLUTION);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), m2_encoderISR, CHANGE);

  stopAllMotors();

  Serial.println("\n\n");
  Serial.println("╔══════════════════════════════════════════════════════╗");
  Serial.println("║   SCARA DUAL MOTOR - SISTEMA INICIADO               ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.println("\n✅ Sistema listo. Escribe 'help' para ver comandos.");
  Serial.println("🎯 Escribe 'map' para mapear las 4 esquinas.\n");
  
  printStatus();
}

// ============================================
// Loop
// ============================================
void loop() {
  // Imprimir posición cada 3 segundos
  if (millis() - lastPrint >= 3000) {
    lastPrint = millis();
    noInterrupts();
    long m1_val = m1_encoderCount;
    long m2_val = m2_encoderCount;
    interrupts();
    
    Serial.print("📍 M1: "); Serial.print(m1_ticksToDegrees(m1_val), 1); Serial.print("° | ");
    Serial.print("M2: "); Serial.print(m2_ticksToDegrees(m2_val), 1); Serial.println("°");
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}