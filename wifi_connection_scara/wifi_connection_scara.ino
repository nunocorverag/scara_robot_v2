#include <Arduino.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <SPIFFS.h>

// ============================================
// CONFIGURACIÓN FÍSICA DEL ROBOT
// ============================================
const float L1 = 30.0;
const float L2 = 18.0;

// Motor 1 (Base)
#define M1_IN1 25
#define M1_IN2 26
#define M1_ENA 27
#define M1_ENC_A 34
#define M1_ENC_B 35

const float M1_MIN_ANGLE = 20;
const float M1_MAX_ANGLE = 140;
const float M1_TICKS_PER_DEGREE = 3.157;
const float M1_REFERENCE_ANGLE = 90.0;
const int M1_MIN_PWM = 160;

// Motor 2
#define M2_IN3 14
#define M2_IN4 12
#define M2_ENB 13
#define M2_ENC_A 32
#define M2_ENC_B 33

const float M2_MIN_ANGLE = 20;
const float M2_MAX_ANGLE = 140;
const float M2_TICKS_PER_DEGREE = 2.743;
const float M2_REFERENCE_ANGLE = 90.0;
const int M2_MIN_PWM = 150;

const long TOLERANCE = 5;
const float XY_TOLERANCE = 0.3;

#define EEPROM_SIZE 2048
#define EEPROM_DENSE_MAP_ADDR 512

// ============================================
// ESPACIO DE TRABAJO
// ============================================
const float WS_X_MIN = -5.0;
const float WS_X_MAX = 5.0;
const float WS_Y_MIN = -5.0;
const float WS_Y_MAX = 5.0;

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

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int MAX_PWM = 255;

// ============================================
// WIFI
// ============================================
const char* ssid = "SCARANet";
const char* password = "robot12345";
WiFiServer server(8080);
WiFiClient client;

volatile int targetX_WiFi = 150;
volatile int targetY_WiFi = 150;
volatile bool newDataAvailable = false;
unsigned long lastWiFiData = 0;
const unsigned long WIFI_TIMEOUT = 5000;

bool wifiConnected = false;
unsigned long lastDebugPrint = 0;

// ============================================
// ESTRUCTURAS
// ============================================
struct CalibrationPoint {
  float x;
  float y;
  float m1_angle;
  float m2_angle;
};

CalibrationPoint calibrationMap[121];
int calibrationPointCount = 0;
bool usePreciseCalibration = false;

// ============================================
// FORWARD DECLARATIONS
// ============================================
float m1_ticksToDegrees(long ticks);
float m2_ticksToDegrees(long ticks);
long m1_degreesToTicks(float degrees);
long m2_degreesToTicks(float degrees);
bool inverseKinematicsFromCalibration(float x, float y, float& theta1, float& theta2);
void forwardKinematics(float theta1, float theta2, float& x, float& y);
bool goToAngles(float m1_targetDegrees, float m2_targetDegrees);
bool goToXY(float x, float y);
void exhaustiveManualCalibration(float step);
void loadCalibration();
void verifyCalibration();
void connectWiFi();
void printHelp();
void printWiFiMenu();

// ============================================
// CONVERSIÓN TICKS/GRADOS
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
// CINEMÁTICA INVERSA
// ============================================
bool inverseKinematicsFromCalibration(float x, float y, float& theta1, float& theta2) {
  if (x < WS_X_MIN || x > WS_X_MAX || y < WS_Y_MIN || y > WS_Y_MAX) {
    return false;
  }

  if (!usePreciseCalibration || calibrationPointCount == 0) {
    return false;
  }

  float distances[121];
  for (int i = 0; i < calibrationPointCount; i++) {
    float dx = calibrationMap[i].x - x;
    float dy = calibrationMap[i].y - y;
    distances[i] = sqrt(dx * dx + dy * dy);
  }

  int closest[4] = {0, 1, 2, 3};
  
  for (int i = 0; i < calibrationPointCount; i++) {
    if (distances[i] < distances[closest[3]]) {
      if (distances[i] < distances[closest[0]]) {
        closest[3] = closest[2];
        closest[2] = closest[1];
        closest[1] = closest[0];
        closest[0] = i;
      } else if (distances[i] < distances[closest[1]]) {
        closest[3] = closest[2];
        closest[2] = closest[1];
        closest[1] = i;
      } else if (distances[i] < distances[closest[2]]) {
        closest[3] = closest[2];
        closest[2] = i;
      } else {
        closest[3] = i;
      }
    }
  }

  float weights[4];
  float totalWeight = 0;

  for (int i = 0; i < 4; i++) {
    float dist = distances[closest[i]];
    if (dist < 0.01) dist = 0.01;
    weights[i] = 1.0 / dist;
    totalWeight += weights[i];
  }

  for (int i = 0; i < 4; i++) {
    weights[i] /= totalWeight;
  }

  theta1 = 0;
  theta2 = 0;
  for (int i = 0; i < 4; i++) {
    theta1 += calibrationMap[closest[i]].m1_angle * weights[i];
    theta2 += calibrationMap[closest[i]].m2_angle * weights[i];
  }

  if (theta1 < M1_MIN_ANGLE || theta1 > M1_MAX_ANGLE ||
      theta2 < M2_MIN_ANGLE || theta2 > M2_MAX_ANGLE) {
    return false;
  }

  return true;
}

void forwardKinematics(float theta1, float theta2, float& x, float& y) {
  x = L1 * cos(theta1 * PI / 180.0) + L2 * cos((theta1 + theta2) * PI / 180.0);
  y = L1 * sin(theta1 * PI / 180.0) + L2 * sin((theta1 + theta2) * PI / 180.0);
}

// ============================================
// ISR ENCODERS
// ============================================
void IRAM_ATTR m1_encoderISR() {
  int currentA = digitalRead(M1_ENC_A);
  int currentB = digitalRead(M1_ENC_B);

  if (m1_lastEncA == LOW && currentA == HIGH) {
    m1_encoderCount += (currentB == LOW) ? -1 : 1;
  } else if (m1_lastEncA == HIGH && currentA == LOW) {
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
  } else if (m2_lastEncA == HIGH && currentA == LOW) {
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
// PID
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
  if (m1_targetDegrees < M1_MIN_ANGLE || m1_targetDegrees > M1_MAX_ANGLE ||
      m2_targetDegrees < M2_MIN_ANGLE || m2_targetDegrees > M2_MAX_ANGLE) {
    Serial.println("ERROR Angulos fuera de rango");
    return false;
  }

  long m1_targetTicks = m1_degreesToTicks(m1_targetDegrees);
  long m2_targetTicks = m2_degreesToTicks(m2_targetDegrees);

  emergencyStop = false;
  m1_lastError = 0;
  m1_integral = 0;
  m1_currentPWM = 0;
  m2_lastError = 0;
  m2_integral = 0;
  m2_currentPWM = 0;
  m1_lastPIDTime = m2_lastPIDTime = millis();

  Serial.println("\n>> Movimiento iniciado");
  Serial.print("   M1: ");
  Serial.print(m1_ticksToDegrees(m1_encoderCount), 1);
  Serial.print("° -> ");
  Serial.print(m1_targetDegrees, 1);
  Serial.println("°");
  Serial.print("   M2: ");
  Serial.print(m2_ticksToDegrees(m2_encoderCount), 1);
  Serial.print("° -> ");
  Serial.print(m2_targetDegrees, 1);
  Serial.println("°");

  int progressCounter = 0;

  while (true) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "stop" || cmd == "s") {
        stopAllMotors();
        return false;
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

      Serial.print("   M1: ");
      Serial.print(m1_angle, 1);
      Serial.print("° M2: ");
      Serial.print(m2_angle, 1);
      Serial.println("°");
    }

    delay(10);
  }

  stopAllMotors();
  Serial.println("[OK] Posicion alcanzada");
  return true;
}

// ============================================
// MOVIMIENTO A XY
// ============================================
bool goToXY(float x, float y) {
  float m1_target, m2_target;

  if (!inverseKinematicsFromCalibration(x, y, m1_target, m2_target)) {
    Serial.println("[ERROR] Posicion sin calibracion o inalcanzable");
    return false;
  }

  return goToAngles(m1_target, m2_target);
}

// ============================================
// CALIBRACIÓN EXHAUSTIVA MANUAL
// ============================================
void exhaustiveManualCalibration(float step) {
  if (step <= 0.1 || step > 5.0) {
    Serial.println("[ERROR] Step debe ser entre 0.1 y 5.0");
    return;
  }

  int points_per_axis = (int)((WS_X_MAX - WS_X_MIN) / step) + 1;
  int totalPoints = points_per_axis * points_per_axis;

  if (totalPoints > 121) {
    Serial.println("[ERROR] Demasiados puntos (maximo 121)");
    return;
  }

  Serial.println("\n[CALIBRACION EXHAUSTIVA - CARTESIANA]");
  Serial.print("Step: ");
  Serial.print(step, 1);
  Serial.print(" cm -> ");
  Serial.print(points_per_axis);
  Serial.print("x");
  Serial.print(points_per_axis);
  Serial.print(" = ");
  Serial.print(totalPoints);
  Serial.println(" PUNTOS");

  Serial.println("(1) YO TE DIGO donde poner el brazo");
  Serial.println("(2) TU MUEVES manualmente");
  Serial.println("(3) Escribes 's' cuando este posicionado");
  Serial.println("(4) YO MIDO los angulos");
  Serial.println("\nPresiona 's' cuando este LISTO:");
  Serial.print("> ");

  while (true) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      input.toLowerCase();
      if (input == "s") {
        break;
      } else if (input == "q") {
        Serial.println("[CANCELADO]");
        return;
      }
    }
  }

  calibrationPointCount = 0;
  Serial.println("\n[INICIANDO CALIBRACION]\n");

  int pointIndex = 0;
  float y_target = WS_Y_MAX;

  while (y_target >= WS_Y_MIN - 0.01) {
    float x_target = WS_X_MIN;

    while (x_target <= WS_X_MAX + 0.01) {
      pointIndex++;

      int progressPercent = (pointIndex * 100) / totalPoints;
      Serial.print("[");
      Serial.print(progressPercent);
      Serial.print("%] PUNTO ");
      Serial.print(pointIndex);
      Serial.print("/");
      Serial.print(totalPoints);
      Serial.print(" -> X=");
      Serial.print(x_target, 1);
      Serial.print(" Y=");
      Serial.print(y_target, 1);
      Serial.println("");

      Serial.println("Pon brazo en esta posicion y escribe 's':");
      Serial.print("> ");

      bool measured = false;
      while (!measured) {
        if (Serial.available()) {
          String input = Serial.readStringUntil('\n');
          input.trim();
          input.toLowerCase();

          if (input == "s") {
            measured = true;
          } else if (input == "q") {
            Serial.println("[CANCELADO]");
            return;
          } else if (input == "skip") {
            Serial.println("[SALTADO]");
            measured = true;
            x_target += step;
            continue;
          }
        }
        delay(50);
      }

      noInterrupts();
      long m1_f = m1_encoderCount;
      long m2_f = m2_encoderCount;
      interrupts();

      float m1_real = m1_ticksToDegrees(m1_f);
      float m2_real = m2_ticksToDegrees(m2_f);

      calibrationMap[calibrationPointCount].x = x_target;
      calibrationMap[calibrationPointCount].y = y_target;
      calibrationMap[calibrationPointCount].m1_angle = m1_real;
      calibrationMap[calibrationPointCount].m2_angle = m2_real;

      Serial.print("[OK] Guardado: M1=");
      Serial.print(m1_real, 2);
      Serial.print(" M2=");
      Serial.println(m2_real, 2);

      calibrationPointCount++;

      if (pointIndex < totalPoints) {
        Serial.println("Presiona ENTER para siguiente:");
        Serial.print("> ");
        while (!Serial.available()) delay(100);
        while (Serial.available()) Serial.read();
      }

      x_target += step;
    }

    y_target -= step;
  }

  Serial.println("\n[GUARDANDO EN EEPROM]");
  EEPROM.writeInt(EEPROM_DENSE_MAP_ADDR, calibrationPointCount);
  for (int i = 0; i < calibrationPointCount; i++) {
    int addr = EEPROM_DENSE_MAP_ADDR + 4 + (i * sizeof(CalibrationPoint));
    uint8_t* ptr = (uint8_t*)&calibrationMap[i];
    EEPROM.writeBytes(addr, ptr, sizeof(CalibrationPoint));
  }
  EEPROM.commit();

  usePreciseCalibration = true;

  Serial.println("\n[OK] CALIBRACION COMPLETADA");
  Serial.print("Puntos: ");
  Serial.println(calibrationPointCount);
}

// ============================================
// VERIFICACIÓN DETALLADA DE CALIBRACIÓN
// ============================================
void verifyCalibrationDetailed() {
  if (!usePreciseCalibration || calibrationPointCount == 0) {
    Serial.println("\n[ERROR] Sin calibracion para verificar");
    return;
  }

  Serial.println("\n[COMPARACION DETALLADA - TODOS LOS PUNTOS]");
  Serial.print("Verificando ");
  Serial.print(calibrationPointCount);
  Serial.println(" puntos...\n");

  Serial.println("PT | XY TARGET    | M1 GUARDADO | M1 REAL | ERROR M1 | M2 GUARDADO | M2 REAL | ERROR M2 | TOTAL");
  Serial.println("---+--------------+-------------+---------+----------+-------------+---------+----------+-------");

  int correctPoints = 0;
  float avgErrorM1 = 0, avgErrorM2 = 0;
  float maxErrorM1 = 0, maxErrorM2 = 0;

  // Guardar a archivo CSV tambien
  File file = SPIFFS.open("/comparacion_calibracion.csv", "w");
  if (file) {
    file.println("Punto,X_cm,Y_cm,M1_guardado_deg,M1_real_deg,Error_M1_deg,M2_guardado_deg,M2_real_deg,Error_M2_deg,Error_Total_deg");
  }

  for (int i = 0; i < calibrationPointCount; i++) {
    float targetM1 = calibrationMap[i].m1_angle;
    float targetM2 = calibrationMap[i].m2_angle;

    if (!goToAngles(targetM1, targetM2)) {
      continue;
    }

    delay(200);

    noInterrupts();
    long m1_real_ticks = m1_encoderCount;
    long m2_real_ticks = m2_encoderCount;
    interrupts();

    float m1_real = m1_ticksToDegrees(m1_real_ticks);
    float m2_real = m2_ticksToDegrees(m2_real_ticks);

    float errorM1 = abs(m1_real - targetM1);
    float errorM2 = abs(m2_real - targetM2);
    float totalError = errorM1 + errorM2;

    avgErrorM1 += errorM1;
    avgErrorM2 += errorM2;

    if (errorM1 > maxErrorM1) maxErrorM1 = errorM1;
    if (errorM2 > maxErrorM2) maxErrorM2 = errorM2;

    if (totalError < 2.0) {
      correctPoints++;
    }

    // Imprimir fila en consola
    if (i < 10) Serial.print("0");
    Serial.print(i);
    Serial.print(" | ");
    
    // XY TARGET
    if (calibrationMap[i].x >= 0) Serial.print("+");
    Serial.print(calibrationMap[i].x, 1);
    Serial.print(",");
    if (calibrationMap[i].y >= 0) Serial.print("+");
    Serial.print(calibrationMap[i].y, 1);
    
    Serial.print(" | ");
    Serial.print(targetM1, 2);
    Serial.print(" | ");
    Serial.print(m1_real, 2);
    Serial.print(" | ");
    Serial.print(errorM1, 3);
    Serial.print(" | ");
    Serial.print(targetM2, 2);
    Serial.print(" | ");
    Serial.print(m2_real, 2);
    Serial.print(" | ");
    Serial.print(errorM2, 3);
    Serial.print(" | ");
    Serial.println(totalError, 3);

    // Guardar a archivo CSV
    if (file) {
      file.print(i);
      file.print(",");
      file.print(calibrationMap[i].x, 1);
      file.print(",");
      file.print(calibrationMap[i].y, 1);
      file.print(",");
      file.print(targetM1, 2);
      file.print(",");
      file.print(m1_real, 2);
      file.print(",");
      file.print(errorM1, 3);
      file.print(",");
      file.print(targetM2, 2);
      file.print(",");
      file.print(m2_real, 2);
      file.print(",");
      file.print(errorM2, 3);
      file.print(",");
      file.println(totalError, 3);
    }

    if ((i + 1) % 20 == 0) {
      delay(50);
    }
  }

  avgErrorM1 /= calibrationPointCount;
  avgErrorM2 /= calibrationPointCount;

  int pct = (correctPoints * 100) / calibrationPointCount;

  // Guardar estadisticas en CSV
  if (file) {
    file.println("");
    file.println("ESTADISTICAS");
    file.print("Puntos correctos,");
    file.print(correctPoints);
    file.print(",");
    file.print(calibrationPointCount);
    file.print(",");
    file.print(pct);
    file.println("%");
    file.print("Error promedio M1 deg,");
    file.println(avgErrorM1, 3);
    file.print("Error maximo M1 deg,");
    file.println(maxErrorM1, 3);
    file.print("Error promedio M2 deg,");
    file.println(avgErrorM2, 3);
    file.print("Error maximo M2 deg,");
    file.println(maxErrorM2, 3);
    file.print("Resultado,");
    if (pct >= 90) {
      file.println("EXCELENTE - Muy precisa");
    } else if (pct >= 70) {
      file.println("BUENA - Aceptable");
    } else if (pct >= 50) {
      file.println("REGULAR - Recalibrar");
    } else {
      file.println("MALA - Recalibrar urgente");
    }
    file.close();
  }

  Serial.println("\n[ESTADISTICAS]");
  Serial.print("Puntos correctos: ");
  Serial.print(correctPoints);
  Serial.print("/");
  Serial.print(calibrationPointCount);
  Serial.print(" (");
  Serial.print(pct);
  Serial.println("%)");

  Serial.print("Error promedio M1: ");
  Serial.print(avgErrorM1, 3);
  Serial.print("° | Maximo: ");
  Serial.print(maxErrorM1, 3);
  Serial.println("°");

  Serial.print("Error promedio M2: ");
  Serial.print(avgErrorM2, 3);
  Serial.print("° | Maximo: ");
  Serial.print(maxErrorM2, 3);
  Serial.println("°");

  Serial.print("\n[RESULTADO] ");
  if (pct >= 90) {
    Serial.println("EXCELENTE - Muy precisa");
  } else if (pct >= 70) {
    Serial.println("BUENA - Aceptable");
  } else if (pct >= 50) {
    Serial.println("REGULAR - Recalibrar");
  } else {
    Serial.println("MALA - Recalibrar urgente");
  }

  Serial.println("\n[OK] Tabla guardada en: comparacion_calibracion.csv");
  Serial.println("Usa 'cal download compare' para descargarla");
}

// ============================================
// VERIFICACIÓN DE CALIBRACIÓN
// ============================================
void verifyCalibration() {
  if (!usePreciseCalibration || calibrationPointCount == 0) {
    Serial.println("\n[ERROR] Sin calibracion para verificar");
    return;
  }

  Serial.println("\n[VERIFICANDO CALIBRACION]");
  Serial.print("Chequeando ");
  Serial.print(calibrationPointCount);
  Serial.println(" puntos...\n");

  int correctPoints = 0;
  float avgErrorM1 = 0, avgErrorM2 = 0;
  float maxErrorM1 = 0, maxErrorM2 = 0;

  Serial.println("PT | XY TARGET | M1 REAL | M2 REAL | ERROR");
  Serial.println("---+----------+---------+---------+-------");

  for (int i = 0; i < calibrationPointCount; i++) {
    float targetM1 = calibrationMap[i].m1_angle;
    float targetM2 = calibrationMap[i].m2_angle;

    if (!goToAngles(targetM1, targetM2)) {
      continue;
    }

    delay(200);

    noInterrupts();
    long m1_real_ticks = m1_encoderCount;
    long m2_real_ticks = m2_encoderCount;
    interrupts();

    float m1_real = m1_ticksToDegrees(m1_real_ticks);
    float m2_real = m2_ticksToDegrees(m2_real_ticks);

    float errorM1 = abs(m1_real - targetM1);
    float errorM2 = abs(m2_real - targetM2);
    float totalError = errorM1 + errorM2;

    avgErrorM1 += errorM1;
    avgErrorM2 += errorM2;

    if (errorM1 > maxErrorM1) maxErrorM1 = errorM1;
    if (errorM2 > maxErrorM2) maxErrorM2 = errorM2;

    if (totalError < 2.0) {
      correctPoints++;
    }

    if (i < 10) Serial.print("0");
    Serial.print(i);
    Serial.print(" | ");
    Serial.print(calibrationMap[i].x, 1);
    Serial.print(",");
    Serial.print(calibrationMap[i].y, 1);
    Serial.print(" | ");
    Serial.print(m1_real, 2);
    Serial.print(" | ");
    Serial.print(m2_real, 2);
    Serial.print(" | ");
    Serial.print(errorM1, 2);
    Serial.print("/");
    Serial.println(errorM2, 2);

    if ((i + 1) % 10 == 0) {
      delay(50);
    }
  }

  avgErrorM1 /= calibrationPointCount;
  avgErrorM2 /= calibrationPointCount;

  Serial.println("\n[ESTADISTICAS]");
  Serial.print("OK: ");
  Serial.print(correctPoints);
  Serial.print("/");
  Serial.print(calibrationPointCount);
  Serial.print(" (");
  Serial.print((correctPoints * 100) / calibrationPointCount);
  Serial.println("%)");

  Serial.print("Promedio M1: ");
  Serial.print(avgErrorM1, 3);
  Serial.print("° | Maximo: ");
  Serial.print(maxErrorM1, 3);
  Serial.println("°");

  Serial.print("Promedio M2: ");
  Serial.print(avgErrorM2, 3);
  Serial.print("° | Maximo: ");
  Serial.print(maxErrorM2, 3);
  Serial.println("°");

  int pct = (correctPoints * 100) / calibrationPointCount;
  Serial.print("\n[RESULTADO] ");
  if (pct >= 90) {
    Serial.println("EXCELENTE - Muy precisa");
  } else if (pct >= 70) {
    Serial.println("BUENA - Aceptable");
  } else if (pct >= 50) {
    Serial.println("REGULAR - Recalibrar");
  } else {
    Serial.println("MALA - Recalibrar urgente");
  }
}

// ============================================
// CARGAR CALIBRACIÓN
// ============================================
void loadCalibration() {
  Serial.println("Cargando calibracion...");

  int count = EEPROM.readInt(EEPROM_DENSE_MAP_ADDR);
  if (count > 0 && count <= 121) {
    calibrationPointCount = count;
    for (int i = 0; i < count; i++) {
      int addr = EEPROM_DENSE_MAP_ADDR + 4 + (i * sizeof(CalibrationPoint));
      uint8_t buffer[sizeof(CalibrationPoint)];
      EEPROM.readBytes(addr, buffer, sizeof(CalibrationPoint));
      calibrationMap[i] = *(CalibrationPoint*)buffer;
    }
    usePreciseCalibration = true;
    Serial.print("[OK] ");
    Serial.print(count);
    Serial.println(" puntos cargados");
  } else {
    Serial.println("[INFO] Sin calibracion guardada");
  }
}

// ============================================
// WiFi
// ============================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
}

void convertWiFiToSCARA(int wifi_x, int wifi_y, float& scara_x, float& scara_y) {
  int centered_x = wifi_x - 150;
  int centered_y = wifi_y - 150;

  scara_x = (float)centered_x / 30.0;
  scara_y = (float)centered_y / 30.0;

  scara_x = constrain(scara_x, -5.0, 5.0);
  scara_y = constrain(scara_y, -5.0, 5.0);
}

void parseWiFiData(String data) {
  data.trim();

  if (data.startsWith("$") && data.indexOf('*') > 0) {
    int asteriskIndex = data.indexOf('*');
    String content = data.substring(1, asteriskIndex);

    int commaIndex = content.indexOf(',');
    if (commaIndex > 0) {
      String xStr = content.substring(0, commaIndex);
      String yStr = content.substring(commaIndex + 1);

      int x = xStr.toInt();
      int y = yStr.toInt();

      if (x >= 0 && x <= 300 && y >= 0 && y <= 300) {
        targetX_WiFi = x;
        targetY_WiFi = y;
        newDataAvailable = true;
        lastWiFiData = millis();
      }
    }
  }
}

void handleWiFi() {
  if (server.hasClient()) {
    if (!client || !client.connected()) {
      if (client) client.stop();
      client = server.available();
      wifiConnected = true;
    }
  }

  if (client && client.connected()) {
    if (client.available()) {
      String inputString = "";
      while (client.available()) {
        char inChar = (char)client.read();
        inputString += inChar;

        if (inChar == '\n') {
          parseWiFiData(inputString);
          inputString = "";
        }
      }
    }
  } else {
    wifiConnected = false;
  }

  if (wifiConnected && (millis() - lastWiFiData > WIFI_TIMEOUT)) {
    wifiConnected = false;
  }
}

// ============================================
// MENÚ Y COMANDOS
// ============================================
enum OperationMode {
  MENU_WIFI,
  DEBUG_MODE,
  WIFI_ACTIVE
};

OperationMode currentMode = MENU_WIFI;

void printWiFiMenu() {
  Serial.println("\n[MENU PRINCIPAL]");
  Serial.println("(1) c  - Conectar WiFi");
  Serial.println("(2) i  - Mostrar IP");
  Serial.println("(3) d  - Modo Debug");
  Serial.println("(4) k  - Calibracion Exhaustiva");
  Serial.println("(?) h  - Ayuda");
  Serial.println("(B) m  - Menu");
}

void processWiFiMenu(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) return;

  if (cmd == "c") {
    Serial.println("Conectando WiFi...");
    connectWiFi();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("[OK] WiFi");
      currentMode = WIFI_ACTIVE;
      server.begin();
    } else {
      Serial.println("[ERROR]");
    }
  } else if (cmd == "i") {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("[ERROR] WiFi desconectado");
    }
  } else if (cmd == "d") {
    Serial.println("[DEBUG MODE]");
    currentMode = DEBUG_MODE;
    printHelp();
  } else if (cmd == "k") {
    Serial.println("Step (cm)? ");
    while (!Serial.available()) delay(100);
    String stepStr = Serial.readStringUntil('\n');
    float step = stepStr.toFloat();

    if (step > 0.1 && step <= 5.0) {
      currentMode = DEBUG_MODE;
      exhaustiveManualCalibration(step);
    } else {
      Serial.println("[ERROR] Step invalido");
    }
  } else if (cmd == "h") {
    printWiFiMenu();
  } else {
    Serial.println("[ERROR] Invalido");
  }
}

void printCalibrationMap() {
  if (!usePreciseCalibration || calibrationPointCount == 0) {
    Serial.println("[ERROR] Sin calibracion");
    return;
  }

  Serial.print("\nPuntos: ");
  Serial.println(calibrationPointCount);
  Serial.println("PT | X | Y | M1 | M2");
  Serial.println("---+---+---+----+----");

  for (int i = 0; i < calibrationPointCount; i++) {
    if (i < 10) Serial.print("0");
    Serial.print(i);
    Serial.print(" | ");
    Serial.print(calibrationMap[i].x, 1);
    Serial.print(" | ");
    Serial.print(calibrationMap[i].y, 1);
    Serial.print(" | ");
    Serial.print(calibrationMap[i].m1_angle, 1);
    Serial.print(" | ");
    Serial.println(calibrationMap[i].m2_angle, 1);

    if ((i + 1) % 10 == 0) {
      delay(10);
    }
  }
}

void downloadCalibrationCSV() {
  if (!SPIFFS.exists("/calibracion_scara.csv")) {
    Serial.println("\n[ERROR] Sin calibracion. Primero: cal save");
    return;
  }

  Serial.println("\n[DESCARGANDO CALIBRACION]");
  Serial.println("(Copia entre INICIO y FIN)\n");

  File file = SPIFFS.open("/calibracion_scara.csv", "r");
  if (!file) {
    Serial.println("[ERROR] Archivo no encontrado");
    return;
  }

  Serial.println("===== INICIO CSV =====\n");

  while (file.available()) {
    String line = file.readStringUntil('\n');
    Serial.println(line);
    delay(2);
  }

  file.close();

  Serial.println("\n===== FIN CSV =====\n");
}

void downloadComparisonCSV() {
  if (!SPIFFS.exists("/comparacion_calibracion.csv")) {
    Serial.println("\n[ERROR] Sin comparacion. Primero: cal compare");
    return;
  }

  Serial.println("\n[DESCARGANDO COMPARACION]");
  Serial.println("(Copia entre INICIO y FIN)\n");

  File file = SPIFFS.open("/comparacion_calibracion.csv", "r");
  if (!file) {
    Serial.println("[ERROR] Archivo no encontrado");
    return;
  }

  Serial.println("===== INICIO CSV =====\n");

  while (file.available()) {
    String line = file.readStringUntil('\n');
    Serial.println(line);
    delay(2);
  }

  file.close();

  Serial.println("\n===== FIN CSV =====\n");
}

void printHelp() {
  Serial.println("\n[COMANDOS]");
  Serial.println("MOVIMIENTO:");
  Serial.println("  xy X Y         - Mover XY");
  Serial.println("  c1/c2/c3/c4    - Esquinas");
  Serial.println("  home, 0        - Centro");

  Serial.println("CONTROL:");
  Serial.println("  stop, s        - Detener");

  Serial.println("CALIBRACION:");
  Serial.println("  cal step       - Calibrar");
  Serial.println("  cal load       - Cargar");
  Serial.println("  cal status     - Estado");
  Serial.println("  cal list       - Listar");
  Serial.println("  cal verify     - Verificar RESUMIDO");
  Serial.println("  cal compare    - Verificar DETALLADO + guardar CSV");
  Serial.println("  cal download calibracion  - Descargar calibracion.csv");
  Serial.println("  cal download compare      - Descargar comparacion.csv");
  Serial.println("  cal erase      - Borrar");

  Serial.println("INFO:");
  Serial.println("  status         - Estado");
  Serial.println("  help, h        - Ayuda");
  Serial.println("  reset          - Reset");
  Serial.println("  m              - Menu");
}

void printStatus() {
  noInterrupts();
  long m1_ticks = m1_encoderCount;
  long m2_ticks = m2_encoderCount;
  interrupts();

  float m1_angle = m1_ticksToDegrees(m1_ticks);
  float m2_angle = m2_ticksToDegrees(m2_ticks);

  Serial.print("[STATUS] M1=");
  Serial.print(m1_angle, 2);
  Serial.print(" M2=");
  Serial.print(m2_angle, 2);
  if (usePreciseCalibration) {
    Serial.print(" CAL=OK(");
    Serial.print(calibrationPointCount);
    Serial.println(")");
  } else {
    Serial.println(" CAL=NONE");
  }
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) return;

  if (cmd == "help" || cmd == "h") {
    printHelp();
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "stop" || cmd == "s") {
    stopAllMotors();
  } else if (cmd == "home" || cmd == "0") {
    goToAngles(90.0, 90.0);
  } else if (cmd == "c1") {
    goToXY(-5.0, -5.0);
  } else if (cmd == "c2") {
    goToXY(5.0, -5.0);
  } else if (cmd == "c3") {
    goToXY(5.0, 5.0);
  } else if (cmd == "c4") {
    goToXY(-5.0, 5.0);
  } else if (cmd == "reset") {
    noInterrupts();
    m1_encoderCount = 0;
    m2_encoderCount = 0;
    interrupts();
    Serial.println("[OK] Reset");
  } else if (cmd.startsWith("xy ")) {
    int space1 = cmd.indexOf(' ');
    int space2 = cmd.indexOf(' ', space1 + 1);
    if (space2 > 0) {
      float x = cmd.substring(space1 + 1, space2).toFloat();
      float y = cmd.substring(space2 + 1).toFloat();
      goToXY(x, y);
    }
  } else if (cmd.startsWith("cal ")) {
    String subCmd = cmd.substring(4);
    if (subCmd == "load") {
      loadCalibration();
    } else if (subCmd == "status") {
      Serial.print("CAL: ");
      Serial.print(usePreciseCalibration ? "OK " : "NONE ");
      Serial.print(calibrationPointCount);
      Serial.println(" pts");
    } else if (subCmd == "list") {
      printCalibrationMap();
    } else if (subCmd == "verify") {
      verifyCalibration();
    } else if (subCmd == "compare") {
      verifyCalibrationDetailed();
    } else if (subCmd.startsWith("download")) {
      String type = subCmd.substring(8);
      type.trim();
      type.toLowerCase();
      if (type == "" || type == "calibracion") {
        downloadCalibrationCSV();
      } else if (type == "compare") {
        downloadComparisonCSV();
      } else {
        Serial.println("[ERROR] cal download calibracion|compare");
      }
    } else if (subCmd == "erase") {
      for (int i = 0; i < EEPROM_SIZE; i++) {
        EEPROM.write(i, 0xFF);
      }
      EEPROM.commit();
      calibrationPointCount = 0;
      usePreciseCalibration = false;
      Serial.println("[OK] Borrado");
    } else {
      float step = subCmd.toFloat();
      if (step > 0.1 && step <= 5.0) {
        exhaustiveManualCalibration(step);
      }
    }
  } else {
    Serial.println("[ERROR] No reconocido");
  }
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!SPIFFS.begin(true)) {
    Serial.println("[ERROR] SPIFFS");
  }

  EEPROM.begin(EEPROM_SIZE);
  delay(100);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M1_ENC_B, INPUT_PULLUP);
  m1_lastEncA = digitalRead(M1_ENC_A);
  m1_lastEncB = digitalRead(M1_ENC_B);
  ledcAttach(M1_ENA, PWM_FREQ, PWM_RESOLUTION);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), m1_encoderISR, CHANGE);

  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_B, INPUT_PULLUP);
  m2_lastEncA = digitalRead(M2_ENC_A);
  m2_lastEncB = digitalRead(M2_ENC_B);
  ledcAttach(M2_ENB, PWM_FREQ, PWM_RESOLUTION);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), m2_encoderISR, CHANGE);

  stopAllMotors();

  Serial.println("\n[SCARA XY INIT]");
  loadCalibration();

  Serial.println("Homing...");
  noInterrupts();
  m1_encoderCount = 0;
  m2_encoderCount = 0;
  interrupts();

  goToAngles(90.0, 90.0);

  long m1_target_ticks = m1_degreesToTicks(90.0);
  long m2_target_ticks = m2_degreesToTicks(90.0);

  noInterrupts();
  m1_encoderCount = m1_target_ticks;
  m2_encoderCount = m2_target_ticks;
  interrupts();

  Serial.println("[OK] Ready\n");

  printWiFiMenu();
  lastDebugPrint = millis();
}

// ============================================
// LOOP
// ============================================
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "m") {
      currentMode = MENU_WIFI;
      printWiFiMenu();
      return;
    }

    if (currentMode == MENU_WIFI) {
      processWiFiMenu(cmd);
    } else if (currentMode == DEBUG_MODE) {
      processCommand(cmd);
    } else if (currentMode == WIFI_ACTIVE) {
      if (cmd == "d") {
        currentMode = DEBUG_MODE;
        printHelp();
      } else {
        processCommand(cmd);
      }
    }
  }

  if (currentMode == WIFI_ACTIVE) {
    handleWiFi();

    if (millis() - lastDebugPrint >= 3000) {
      lastDebugPrint = millis();

      noInterrupts();
      long m1_val = m1_encoderCount;
      long m2_val = m2_encoderCount;
      interrupts();

      float m1_angle = m1_ticksToDegrees(m1_val);
      float m2_angle = m2_ticksToDegrees(m2_val);

      Serial.print("[POS] M1=");
      Serial.print(m1_angle, 1);
      Serial.print(" M2=");
      Serial.print(m2_angle, 1);
      Serial.print(" ");
      Serial.println(wifiConnected ? "[OK]" : "[X]");
    }

    if (newDataAvailable) {
      newDataAvailable = false;

      float scara_x, scara_y;
      convertWiFiToSCARA(targetX_WiFi, targetY_WiFi, scara_x, scara_y);

      goToXY(scara_x, scara_y);
    }
  } else if (currentMode == DEBUG_MODE) {
    if (millis() - lastDebugPrint >= 3000) {
      lastDebugPrint = millis();

      noInterrupts();
      long m1_val = m1_encoderCount;
      long m2_val = m2_encoderCount;
      interrupts();

      float m1_angle = m1_ticksToDegrees(m1_val);
      float m2_angle = m2_ticksToDegrees(m2_val);

      Serial.print("[POS] M1=");
      Serial.print(m1_angle, 1);
      Serial.print(" M2=");
      Serial.print(m2_angle, 1);
      Serial.println("");
    }
  }
}