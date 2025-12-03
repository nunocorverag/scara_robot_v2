#include <Arduino.h>

// ============================================
// CONFIGURACIÓN FÍSICA DEL ROBOT
// ============================================
// IMPORTANTE: Estos valores se recalcularon basados en las 4 esquinas
// medidas reales para ajustarse a la geometría actual
const float L1 = 30.0;  // Brazo 1: 30 cm
const float L2 = 18.0;  // Brazo 2: 18 cm

// CALIBRACIÓN: Offset de la posición de home basada en esquinas
// Home real está en (4.25, 2.9) cuando ambos motores están a 90°
const float HOME_X_OFFSET = 4.25;
const float HOME_Y_OFFSET = 2.9;

// Motor 1 (Base)
#define M1_IN1 25
#define M1_IN2 26
#define M1_ENA 27
#define M1_ENC_A 34
#define M1_ENC_B 35

const float M1_MIN_ANGLE = 77.0;
const float M1_MAX_ANGLE = 103.3;
const float M1_TICKS_PER_DEGREE = 3.157;
const float M1_REFERENCE_ANGLE = 90.0;
const int M1_MIN_PWM = 160;

// Motor 2 (Antebrazo - desfasado 90°)
#define M2_IN3 14
#define M2_IN4 12
#define M2_ENB 13
#define M2_ENC_A 32
#define M2_ENC_B 33

const float M2_MIN_ANGLE = 69.6;
const float M2_MAX_ANGLE = 116.2;
const float M2_TICKS_PER_DEGREE = 2.743;
const float M2_REFERENCE_ANGLE = 90.0;
const int M2_MIN_PWM = 150;

const long TOLERANCE = 5;
const float XY_TOLERANCE = 0.3;  // cm

// ============================================
// ESPACIO DE TRABAJO
// ============================================
const float WS_X_MIN = 0.0;
const float WS_X_MAX = 8.5;
const float WS_Y_MIN = 0.0;
const float WS_Y_MAX = 5.8;

// Variables globales
volatile long m1_encoderCount = 0;
volatile long m2_encoderCount = 0;
volatile int m1_lastEncA = 0;
volatile int m1_lastEncB = 0;
volatile int m2_lastEncA = 0;
volatile int m2_lastEncB = 0;

float m1_Kp = 0.8, m1_Ki = 0.02, m1_Kd = 1.2;
float m1_lastError = 0, m1_integral = 0;
unsigned long m1_lastPIDTime = 0;
int m1_currentPWM = 0;

float m2_Kp = 0.8, m2_Ki = 0.02, m2_Kd = 1.2;
float m2_lastError = 0, m2_integral = 0;
unsigned long m2_lastPIDTime = 0;
int m2_currentPWM = 0;

bool emergencyStop = false;
bool verboseMode = true;
unsigned long lastPrint = 0;

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int MAX_PWM = 255;

// ============================================
// PUNTOS DE CALIBRACIÓN (Esquinas conocidas)
// ============================================
struct CalibPoint {
  float m1_angle;
  float m2_angle;
  float x;
  float y;
};

// Puntos calibrados del workspace
CalibPoint calibPoints[4] = {
  {77.0,  85.6,  0.0,  0.0},  // c1: esquina inferior izquierda
  {103.3, 69.6,  8.5,  0.0},  // c2: esquina inferior derecha
  {103.3, 95.5,  8.5,  5.8},  // c3: esquina superior derecha
  {79.2,  116.2, 0.0,  5.8}   // c4: esquina superior izquierda
};

// ============================================
// CINEMÁTICA INVERSA CON INTERPOLACIÓN
// ============================================
bool inverseKinematicsCalibrated(float x, float y, float& theta1, float& theta2) {
  // Método: Interpolación bilineal basada en puntos de calibración
  
  // Validar límites del workspace
  if (x < WS_X_MIN || x > WS_X_MAX || y < WS_Y_MIN || y > WS_Y_MAX) {
    Serial.print("❌ Fuera de workspace: (");
    Serial.print(x, 2); Serial.print(", "); Serial.print(y, 2); Serial.println(")");
    return false;
  }
  
  // Normalizar coordenadas (0 a 1)
  float x_norm = (x - WS_X_MIN) / (WS_X_MAX - WS_X_MIN);
  float y_norm = (y - WS_Y_MIN) / (WS_Y_MAX - WS_Y_MIN);
  
  // Interpolación bilineal para theta1 y theta2
  // Dividimos el workspace en 4 cuadrantes
  
  float t1, t2;
  
  if (x_norm <= 0.5 && y_norm <= 0.5) {
    // Cuadrante inferior izquierdo (c1)
    float fx = x_norm * 2.0;
    float fy = y_norm * 2.0;
    
    // Interpolar entre c1 y vecinos
    float t1_x0 = calibPoints[0].m1_angle * (1 - fx) + calibPoints[1].m1_angle * fx;
    float t1_x1 = calibPoints[3].m1_angle * (1 - fx) + calibPoints[2].m1_angle * fx;
    t1 = t1_x0 * (1 - fy) + t1_x1 * fy;
    
    float t2_x0 = calibPoints[0].m2_angle * (1 - fx) + calibPoints[1].m2_angle * fx;
    float t2_x1 = calibPoints[3].m2_angle * (1 - fx) + calibPoints[2].m2_angle * fx;
    t2 = t2_x0 * (1 - fy) + t2_x1 * fy;
  }
  else if (x_norm > 0.5 && y_norm <= 0.5) {
    // Cuadrante inferior derecho (c2)
    float fx = (x_norm - 0.5) * 2.0;
    float fy = y_norm * 2.0;
    
    float t1_x0 = calibPoints[1].m1_angle * (1 - fx) + calibPoints[1].m1_angle * fx;
    float t1_x1 = calibPoints[2].m1_angle * (1 - fx) + calibPoints[2].m1_angle * fx;
    t1 = t1_x0 * (1 - fy) + t1_x1 * fy;
    
    float t2_x0 = calibPoints[1].m2_angle * (1 - fx) + calibPoints[1].m2_angle * fx;
    float t2_x1 = calibPoints[2].m2_angle * (1 - fx) + calibPoints[2].m2_angle * fx;
    t2 = t2_x0 * (1 - fy) + t2_x1 * fy;
  }
  else if (x_norm <= 0.5 && y_norm > 0.5) {
    // Cuadrante superior izquierdo (c4)
    float fx = x_norm * 2.0;
    float fy = (y_norm - 0.5) * 2.0;
    
    float t1_x0 = calibPoints[0].m1_angle * (1 - fx) + calibPoints[1].m1_angle * fx;
    float t1_x1 = calibPoints[3].m1_angle * (1 - fx) + calibPoints[2].m1_angle * fx;
    t1 = t1_x0 * (1 - fy) + t1_x1 * fy;
    
    float t2_x0 = calibPoints[0].m2_angle * (1 - fx) + calibPoints[1].m2_angle * fx;
    float t2_x1 = calibPoints[3].m2_angle * (1 - fx) + calibPoints[2].m2_angle * fx;
    t2 = t2_x0 * (1 - fy) + t2_x1 * fy;
  }
  else {
    // Cuadrante superior derecho (c3)
    float fx = (x_norm - 0.5) * 2.0;
    float fy = (y_norm - 0.5) * 2.0;
    
    float t1_x0 = calibPoints[1].m1_angle * (1 - fx) + calibPoints[1].m1_angle * fx;
    float t1_x1 = calibPoints[2].m1_angle * (1 - fx) + calibPoints[2].m1_angle * fx;
    t1 = t1_x0 * (1 - fy) + t1_x1 * fy;
    
    float t2_x0 = calibPoints[1].m2_angle * (1 - fx) + calibPoints[1].m2_angle * fx;
    float t2_x1 = calibPoints[2].m2_angle * (1 - fx) + calibPoints[2].m2_angle * fx;
    t2 = t2_x0 * (1 - fy) + t2_x1 * fy;
  }
  
  // Validar límites
  if (t1 < M1_MIN_ANGLE || t1 > M1_MAX_ANGLE) {
    Serial.print("❌ Motor 1 fuera de rango: ");
    Serial.print(t1, 1); Serial.println("°");
    return false;
  }
  if (t2 < M2_MIN_ANGLE || t2 > M2_MAX_ANGLE) {
    Serial.print("❌ Motor 2 fuera de rango: ");
    Serial.print(t2, 1); Serial.println("°");
    return false;
  }
  
  theta1 = t1;
  theta2 = t2;
  return true;
}

// ============================================
// CINEMÁTICA DIRECTA CON CALIBRACIÓN
// ============================================
void forwardKinematicsCalibrated(float theta1, float theta2, float& x, float& y) {
  // Usar interpolación inversa basada en puntos conocidos
  // Buscar en qué cuadrante caen los ángulos y interpolar XY
  
  x = 0;
  y = 0;
  bool found = false;
  
  // Buscar los 4 puntos más cercanos
  float min_dist = 999999;
  int closest_idx = 0;
  
  for (int i = 0; i < 4; i++) {
    float d1 = abs(calibPoints[i].m1_angle - theta1);
    float d2 = abs(calibPoints[i].m2_angle - theta2);
    float dist = sqrt(d1*d1 + d2*d2);
    
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  
  // Si está muy cerca de un punto calibrado, usar ese
  if (min_dist < 2.0) {
    x = calibPoints[closest_idx].x;
    y = calibPoints[closest_idx].y;
    return;
  }
  
  // Interpolar entre dos puntos cercanos
  float t1_error_best = 999;
  float t2_error_best = 999;
  int idx_best = 0;
  
  for (int i = 0; i < 4; i++) {
    float e1 = abs(calibPoints[i].m1_angle - theta1);
    float e2 = abs(calibPoints[i].m2_angle - theta2);
    
    if (e1 + e2 < t1_error_best + t2_error_best) {
      t1_error_best = e1;
      t2_error_best = e2;
      idx_best = i;
    }
  }
  
  // Usar el punto más cercano como aproximación
  x = calibPoints[idx_best].x;
  y = calibPoints[idx_best].y;
}

// ============================================
// CONVERSIÓN TICKS ↔ GRADOS
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
// ISR ENCODERS
// ============================================
void IRAM_ATTR m1_encoderISR() {
  int currentA = digitalRead(M1_ENC_A);
  int currentB = digitalRead(M1_ENC_B);
  
  if (m1_lastEncA == LOW && currentA == HIGH) {
    m1_encoderCount += (currentB == LOW) ? -1 : 1;
  }
  else if (m1_lastEncA == HIGH && currentA == LOW) {
    m1_encoderCount += (currentB == HIGH) ? -1 : 1;
  }
  
  m1_lastEncA = currentA;
  m1_lastEncB = currentB;
}

void IRAM_ATTR m2_encoderISR() {
  int currentA = digitalRead(M2_ENC_A);
  int currentB = digitalRead(M2_ENC_B);
  
  if (m2_lastEncA == LOW && currentA == HIGH) {
    m2_encoderCount += (currentB == LOW) ? -1 : 1;
  }
  else if (m2_lastEncA == HIGH && currentA == LOW) {
    m2_encoderCount += (currentB == HIGH) ? -1 : 1;
  }
  
  m2_lastEncA = currentA;
  m2_lastEncB = currentB;
}

// ============================================
// CONTROL DE MOTORES
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
// CONTROL PID
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
// MOVIMIENTO A ÁNGULOS
// ============================================
bool goToAngles(float m1_targetDegrees, float m2_targetDegrees) {
  if (m1_targetDegrees < M1_MIN_ANGLE || m1_targetDegrees > M1_MAX_ANGLE) {
    Serial.print("❌ Motor 1 fuera de rango: ");
    Serial.print(m1_targetDegrees, 1); Serial.println("°");
    return false;
  }
  if (m2_targetDegrees < M2_MIN_ANGLE || m2_targetDegrees > M2_MAX_ANGLE) {
    Serial.print("❌ Motor 2 fuera de rango: ");
    Serial.print(m2_targetDegrees, 1); Serial.println("°");
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
    if (emergencyStop || Serial.available()) {
      if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "stop" || cmd == "s") {
          stopAllMotors();
          Serial.println("\n⚠  MOVIMIENTO DETENIDO");
          return false;
        }
      }
    }
    
    noInterrupts();
    long m1_current = m1_encoderCount;
    long m2_current = m2_encoderCount;
    interrupts();
    
    bool m1_arrived = abs(m1_current - m1_targetTicks) <= TOLERANCE;
    bool m2_arrived = abs(m2_current - m2_targetTicks) <= TOLERANCE;
    
    if (m1_arrived && m2_arrived) break;
    
    if (!m1_arrived) {
      int pwm1 = m1_calculatePID(m1_current, m1_targetTicks);
      m1_move((m1_current > m1_targetTicks) ? -1 : 1, pwm1);
    } else {
      m1_stop();
    }
    
    if (!m2_arrived) {
      int pwm2 = m2_calculatePID(m2_current, m2_targetTicks);
      m2_move((m2_current > m2_targetTicks) ? -1 : 1, pwm2);
    } else {
      m2_stop();
    }
    
    if (verboseMode && (progressCounter++ % 15 == 0)) {
      float m1_angle = m1_ticksToDegrees(m1_current);
      float m2_angle = m2_ticksToDegrees(m2_current);
      float x, y;
      forwardKinematicsCalibrated(m1_angle, m2_angle, x, y);
      
      Serial.print("  M1: "); Serial.print(m1_angle, 1); Serial.print("° ");
      Serial.print(m1_arrived ? "✓" : "→");
      Serial.print(" | M2: "); Serial.print(m2_angle, 1); Serial.print("° ");
      Serial.print(m2_arrived ? "✓" : "→");
      Serial.print(" | XY: ("); Serial.print(x, 1); Serial.print(", ");
      Serial.print(y, 1); Serial.println(")");
    }
    
    delay(10);
  }
  
  stopAllMotors();
  unsigned long elapsed = millis() - startTime;
  
  Serial.println("\n✅ POSICIÓN ALCANZADA!");
  Serial.print("  Tiempo: "); Serial.print(elapsed / 1000.0, 1); Serial.println(" s");
  
  noInterrupts();
  long m1_f = m1_encoderCount;
  long m2_f = m2_encoderCount;
  interrupts();
  
  float m1_f_angle = m1_ticksToDegrees(m1_f);
  float m2_f_angle = m2_ticksToDegrees(m2_f);
  float x_f, y_f;
  forwardKinematicsCalibrated(m1_f_angle, m2_f_angle, x_f, y_f);
  
  Serial.print("  Motor 1: "); Serial.print(m1_f_angle, 2); Serial.println("°");
  Serial.print("  Motor 2: "); Serial.print(m2_f_angle, 2); Serial.println("°");
  Serial.print("  Posición XY: ("); Serial.print(x_f, 2); Serial.print(", ");
  Serial.print(y_f, 2); Serial.println(")");
  
  return true;
}

// ============================================
// MOVIMIENTO A COORDENADAS XY
// ============================================
bool goToXY(float x, float y) {
  float m1_target, m2_target;
  
  Serial.print("\n📍 Calculando IK para (");
  Serial.print(x, 2); Serial.print(" cm, "); Serial.print(y, 2); Serial.println(" cm)");
  
  if (!inverseKinematicsCalibrated(x, y, m1_target, m2_target)) {
    return false;
  }
  
  Serial.print("✓ Ángulos: M1=");
  Serial.print(m1_target, 2); Serial.print("°, M2=");
  Serial.print(m2_target, 2); Serial.println("°");
  
  float check_x, check_y;
  forwardKinematicsCalibrated(m1_target, m2_target, check_x, check_y);
  Serial.print("✓ Verificación: (");
  Serial.print(check_x, 2); Serial.print(" cm, ");
  Serial.print(check_y, 2); Serial.println(" cm)");
  
  return goToAngles(m1_target, m2_target);
}

// ============================================
// TRAYECTORIA RECTANGULAR
// ============================================
void rectangularPath(int points_per_side) {
  if (points_per_side < 2) {
    Serial.println("❌ Mínimo 2 puntos por lado");
    return;
  }
  
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║      TRAYECTORIA RECTANGULAR INICIADA               ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.print("Workspace: X["); Serial.print(WS_X_MIN); Serial.print("-");
  Serial.print(WS_X_MAX); Serial.print("] Y[");
  Serial.print(WS_Y_MIN); Serial.print("-"); Serial.print(WS_Y_MAX); Serial.println("]");
  Serial.print("Puntos por lado: "); Serial.println(points_per_side);
  
  float step_x = (WS_X_MAX - WS_X_MIN) / (points_per_side - 1);
  float step_y = (WS_Y_MAX - WS_Y_MIN) / (points_per_side - 1);
  
  Serial.println("\n→ LADO INFERIOR");
  for (int i = 0; i < points_per_side; i++) {
    float x = WS_X_MIN + (step_x * i);
    if (!goToXY(x, WS_Y_MIN)) return;
    delay(300);
  }
  
  Serial.println("\n↑ LADO DERECHO");
  for (int i = 1; i < points_per_side; i++) {
    float y = WS_Y_MIN + (step_y * i);
    if (!goToXY(WS_X_MAX, y)) return;
    delay(300);
  }
  
  Serial.println("\n← LADO SUPERIOR");
  for (int i = points_per_side - 2; i >= 0; i--) {
    float x = WS_X_MIN + (step_x * i);
    if (!goToXY(x, WS_Y_MAX)) return;
    delay(300);
  }
  
  Serial.println("\n↓ LADO IZQUIERDO");
  for (int i = points_per_side - 2; i > 0; i--) {
    float y = WS_Y_MIN + (step_y * i);
    if (!goToXY(WS_X_MIN, y)) return;
    delay(300);
  }
  
  Serial.println("\n✅ TRAYECTORIA COMPLETADA!");
}

// ============================================
// AYUDA Y ESTADO
// ============================================
void printHelp() {
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║      SCARA XY - CONTROL CON CINEMÁTICA INVERSA      ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  
  Serial.println("\n📐 PARÁMETROS FÍSICOS:");
  Serial.print("  Brazo 1 (L1): "); Serial.print(L1); Serial.println(" cm");
  Serial.print("  Brazo 2 (L2): "); Serial.print(L2); Serial.println(" cm");
  
  Serial.println("\n📍 ESPACIO DE TRABAJO:");
  Serial.print("  X: "); Serial.print(WS_X_MIN); Serial.print(" - ");
  Serial.print(WS_X_MAX); Serial.println(" cm");
  Serial.print("  Y: "); Serial.print(WS_Y_MIN); Serial.print(" - ");
  Serial.print(WS_Y_MAX); Serial.println(" cm");
  
  Serial.println("\n🎯 ESQUINAS PREDEFINIDAS:");
  Serial.println("  c1 → (0.0, 0.0) cm");
  Serial.println("  c2 → (8.5, 0.0) cm");
  Serial.println("  c3 → (8.5, 5.8) cm");
  Serial.println("  c4 → (0.0, 5.8) cm");
  
  Serial.println("\n💡 COMANDOS:");
  Serial.println("  xy X Y       → Mover a coordenadas XY (cm)");
  Serial.println("                 Ejemplo: xy 4.25 2.9");
  Serial.println("  c1/c2/c3/c4 → Ir a esquina predefinida");
  Serial.println("  rect N       → Trayectoria rectangular (N puntos/lado)");
  Serial.println("                 Ejemplo: rect 5");
  Serial.println("  home / 0    → Centro (4.25, 2.9 cm)");
  Serial.println("  status      → Ver estado actual");
  Serial.println("  stop / s    → Detener movimiento");
  Serial.println("  help / h    → Mostrar esta ayuda");
  Serial.println("\n⚙  CONFIGURACIÓN:");
  Serial.println("  reset       → Resetear encoders");
  Serial.println("════════════════════════════════════════════════════════");
}

void printStatus() {
  noInterrupts();
  long m1_ticks = m1_encoderCount;
  long m2_ticks = m2_encoderCount;
  interrupts();
  
  float m1_angle = m1_ticksToDegrees(m1_ticks);
  float m2_angle = m2_ticksToDegrees(m2_ticks);
  float x, y;
  forwardKinematicsCalibrated(m1_angle, m2_angle, x, y);
  
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║              ESTADO DEL SISTEMA                      ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  
  Serial.println("\n🔵 MOTOR 1:");
  Serial.print("  Ángulo: "); Serial.print(m1_angle, 2); Serial.println("°");
  Serial.print("  Rango: "); Serial.print(M1_MIN_ANGLE); Serial.print(" - ");
  Serial.print(M1_MAX_ANGLE); Serial.println("°");
  
  Serial.println("\n🟢 MOTOR 2:");
  Serial.print("  Ángulo: "); Serial.print(m2_angle, 2); Serial.println("°");
  Serial.print("  Rango: "); Serial.print(M2_MIN_ANGLE); Serial.print(" - ");
  Serial.print(M2_MAX_ANGLE); Serial.println("°");
  
  Serial.println("\n📍 POSICIÓN CARTESIANA (FK):");
  Serial.print("  X = "); Serial.print(x, 2); Serial.print(" cm");
  if (x >= WS_X_MIN && x <= WS_X_MAX) Serial.print(" ✓");
  Serial.println();
  Serial.print("  Y = "); Serial.print(y, 2); Serial.print(" cm");
  if (y >= WS_Y_MIN && y <= WS_Y_MAX) Serial.print(" ✓");
  Serial.println();
  Serial.println("════════════════════════════════════════════════════════\n");
}

// ============================================
// PROCESADOR DE COMANDOS
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
  else if (cmd == "home" || cmd == "0") {
    goToXY(4.25, 2.9);
  }
  else if (cmd == "c1") {
    goToXY(0.0, 0.0);
  }
  else if (cmd == "c2") {
    goToXY(8.5, 0.0);
  }
  else if (cmd == "c3") {
    goToXY(8.5, 5.8);
  }
  else if (cmd == "c4") {
    goToXY(0.0, 5.8);
  }
  else if (cmd == "reset") {
    noInterrupts();
    m1_encoderCount = 0;
    m2_encoderCount = 0;
    interrupts();
    Serial.println("✓ Encoders reiniciados");
  }
  else if (cmd.startsWith("xy ")) {
    int space1 = cmd.indexOf(' ');
    int space2 = cmd.indexOf(' ', space1 + 1);
    
    if (space2 > 0) {
      float x = cmd.substring(space1 + 1, space2).toFloat();
      float y = cmd.substring(space2 + 1).toFloat();
      goToXY(x, y);
    } else {
      Serial.println("❌ Formato: xy X Y (Ejemplo: xy 4.25 2.9)");
    }
  }
  else if (cmd.startsWith("rect ")) {
    int space = cmd.indexOf(' ');
    int points = cmd.substring(space + 1).toInt();
    rectangularPath(points);
  }
  else {
    Serial.print("✗ Comando no reconocido: '");
    Serial.print(cmd);
    Serial.println("'");
    Serial.println("  Escribe 'help' para ver comandos");
  }
}

// ============================================
// SETUP
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
  Serial.println("║   SCARA XY - SISTEMA INICIADO                       ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  
  // Calibración de encoders
  Serial.println("\n⚙  CALIBRACIÓN: Posicionando en home (90°, 90°)...");
  
  // Resetear contadores
  noInterrupts();
  m1_encoderCount = 0;
  m2_encoderCount = 0;
  interrupts();
  
  // Ir a posición home
  delay(500);
  goToAngles(90.0, 90.0);
  
  // Ajustar encoders para que representen 90° correctamente
  long m1_target_ticks = m1_degreesToTicks(90.0);
  long m2_target_ticks = m2_degreesToTicks(90.0);
  
  noInterrupts();
  m1_encoderCount = m1_target_ticks;
  m2_encoderCount = m2_target_ticks;
  interrupts();
  
  Serial.println("✅ Sistema calibrado y listo");
  Serial.println("🎯 Escribe 'help' para ver comandos\n");
  
  delay(500);
  printStatus();
}

// ============================================
// LOOP
// ============================================
void loop() {
  if (millis() - lastPrint >= 3000) {
    lastPrint = millis();
    noInterrupts();
    long m1_val = m1_encoderCount;
    long m2_val = m2_encoderCount;
    interrupts();
    
    float m1_angle = m1_ticksToDegrees(m1_val);
    float m2_angle = m2_ticksToDegrees(m2_val);
    float x, y;
    forwardKinematicsCalibrated(m1_angle, m2_angle, x, y);
    
    Serial.print("📍 M1: "); Serial.print(m1_angle, 1); Serial.print("° | ");
    Serial.print("M2: "); Serial.print(m2_angle, 1); Serial.print("° | ");
    Serial.print("XY: ("); Serial.print(x, 1); Serial.print(", ");
    Serial.print(y, 1); Serial.println(")");
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}