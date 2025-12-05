#include <Arduino.h>

// ============================================
// CONFIGURACIÓN FÍSICA DEL ROBOT
// ============================================
const float L1 = 30.0;  // Brazo 1: 30 cm
const float L2 = 18.0;  // Brazo 2: 18 cm

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

// Motor 2 (Antebrazo)
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
const float XY_TOLERANCE = 0.3;

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
bool verboseMode = false;  // Reducir verbosidad en modo WiFi
unsigned long lastPrint = 0;

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int MAX_PWM = 255;

// ============================================
// WIFI CONFIGURATION
// ============================================
#include <WiFi.h>

const char* ssid = "SCARANet";
const char* password = "robot12345";
WiFiServer server(8080);
WiFiClient client;

// Variables WiFi - AHORA FLOAT para coordenadas directas
volatile float targetX_WiFi = 0.0;
volatile float targetY_WiFi = 0.0;
volatile bool newDataAvailable = false;
unsigned long lastWiFiData = 0;
const unsigned long WIFI_TIMEOUT = 5000;

bool wifiConnected = false;
unsigned long lastDebugPrint = 0;

// Modo de operación
enum OperationMode {
  MENU_WIFI,
  DEBUG_MODE,
  WIFI_ACTIVE
};

OperationMode currentMode = MENU_WIFI;

// ============================================
// PUNTOS DE CALIBRACIÓN
// ============================================
struct CalibPoint {
  float m1_angle;
  float m2_angle;
  float x;
  float y;
};

CalibPoint calibPoints[4] = {
  {96.0,  61.6,  -5.0, -5.0},
  {83.7,  83.1,   5.0, -5.0},
  {86.5, 123.2,   5.0,  5.0},
  {91.6, 100.6,  -5.0,  5.0}
};

// ============================================
// CINEMÁTICA INVERSA CON INTERPOLACIÓN
// ============================================
bool inverseKinematicsCalibrated(float x, float y, float& theta1, float& theta2) {
  if (x < WS_X_MIN || x > WS_X_MAX || y < WS_Y_MIN || y > WS_Y_MAX) {
    Serial.print("❌ Fuera de workspace: (");
    Serial.print(x, 2); Serial.print(", "); Serial.print(y, 2); Serial.println(")");
    return false;
  }

  float x_norm = (x - (-5.0)) / 10.0;
  float y_norm = (y - (-5.0)) / 10.0;

  float m1_bottom = 103.3 + (x_norm * (77.0 - 103.3));
  float m2_bottom = 69.6 + (x_norm * (85.6 - 69.6));

  float m1_top = 103.3 + (x_norm * (79.2 - 103.3));
  float m2_top = 95.5 + (x_norm * (116.2 - 95.5));

  float t1 = m1_bottom + (y_norm * (m1_top - m1_bottom));
  float t2 = m2_bottom + (y_norm * (m2_top - m2_bottom));

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
  float distances[4];
  float weights[4];
  float totalWeight = 0;

  distances[0] = sqrt(pow(theta1 - 77.0, 2) + pow(theta2 - 85.6, 2));
  distances[1] = sqrt(pow(theta1 - 103.3, 2) + pow(theta2 - 69.6, 2));
  distances[2] = sqrt(pow(theta1 - 103.3, 2) + pow(theta2 - 95.5, 2));
  distances[3] = sqrt(pow(theta1 - 79.2, 2) + pow(theta2 - 116.2, 2));

  for (int i = 0; i < 4; i++) {
    weights[i] = 1.0 / (distances[i] + 0.1);
    totalWeight += weights[i];
  }

  for (int i = 0; i < 4; i++) {
    weights[i] = weights[i] / totalWeight;
  }

  float calib_x[4] = {-5.0, 5.0, 5.0, -5.0};
  float calib_y[4] = {-5.0, -5.0, 5.0, 5.0};

  x = 0;
  y = 0;
  for (int i = 0; i < 4; i++) {
    x += calib_x[i] * weights[i];
    y += calib_y[i] * weights[i];
  }
}

// ============================================
// MENÚ INICIAL WiFi
// ============================================
void printWiFiMenu() {
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║         MENÚ INICIAL - SCARA ROBOT                  ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.println("\n1️⃣  c  → Conectar WiFi (SCARANet)");
  Serial.println("2️⃣  i  → Mostrar IP actual");
  Serial.println("3️⃣  d  → Modo Debug (comandos locales)");
  Serial.println("?️  h  → Mostrar este menú");
  Serial.println("🔙 m  → Menú (desde cualquier modo)");
  Serial.println("\n════════════════════════════════════════════════════════");
}

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

void processWiFiMenu(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) return;

  Serial.print("\n➜ Comando: '"); Serial.print(cmd); Serial.println("'");

  if (cmd == "c") {
    Serial.println("📡 Intentando conectar a WiFi...");
    connectWiFi();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("✅ WiFi conectado!");
      Serial.print("🌐 IP: ");
      Serial.println(WiFi.localIP());
      Serial.println("📡 Puerto: 8080");
      Serial.println("\n⏳ Esperando conexión de Raspberry Pi...");
      currentMode = WIFI_ACTIVE;
      server.begin();
    } else {
      Serial.println("❌ Error de conexión");
    }
  }
  else if (cmd == "i") {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("🌐 IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("⚠  WiFi no conectado");
    }
  }
  else if (cmd == "d") {
    Serial.println("🔧 Entrando a DEBUG MODE");
    currentMode = DEBUG_MODE;
    printHelp();
  }
  else if (cmd == "h") {
    printWiFiMenu();
  }
  else {
    Serial.println("❌ Comando inválido");
    printWiFiMenu();
  }
}

// ============================================
// PARSEO DE DATOS WiFi - FORMATO SIMPLE
// ============================================
void parseWiFiData(String data) {
  data.trim();
  
  // Ignorar líneas vacías
  if (data.length() == 0) return;

  // ===== COMANDOS ESPECIALES =====
  if (data == "PING") {
    Serial.println("📡 PING recibido");
    if (client && client.connected()) {
      client.println("PONG");
    }
    return;
  }

  if (data == "STOP") {
    Serial.println("🛑 STOP recibido - Deteniendo motores");
    stopAllMotors();
    return;
  }

  // ===== PARSEO DE COORDENADAS =====
  // Formato esperado: "X.XX,Y.YY" (ejemplo: "-2.50,3.75")
  int commaIndex = data.indexOf(',');

  if (commaIndex > 0) {
    String xStr = data.substring(0, commaIndex);
    String yStr = data.substring(commaIndex + 1);

    float x = xStr.toFloat();
    float y = yStr.toFloat();

    // Validar que sean números válidos (no 0,0 por error de parseo)
    bool validParse = true;
    if (x == 0.0 && xStr != "0" && xStr != "0.0" && xStr != "0.00") {
      if (xStr.indexOf('0') != 0) validParse = false;
    }
    if (y == 0.0 && yStr != "0" && yStr != "0.0" && yStr != "0.00") {
      if (yStr.indexOf('0') != 0) validParse = false;
    }

    // Validar rango (-5 a 5)
    if (validParse && x >= WS_X_MIN && x <= WS_X_MAX && 
        y >= WS_Y_MIN && y <= WS_Y_MAX) {
      
      targetX_WiFi = x;
      targetY_WiFi = y;
      newDataAvailable = true;
      lastWiFiData = millis();

      Serial.print("📡 Coordenadas recibidas: X=");
      Serial.print(x, 2); Serial.print(" Y=");
      Serial.println(y, 2);
    } else {
      Serial.print("❌ Coordenadas inválidas o fuera de rango: ");
      Serial.println(data);
    }
  } else {
    Serial.print("⚠ Formato no reconocido: ");
    Serial.println(data);
  }
}

// ============================================
// GESTIÓN DE CONEXIÓN WiFi
// ============================================
void handleWiFi() {
  // Verificar si hay cliente nuevo
  if (server.hasClient()) {
    if (!client || !client.connected()) {
      if (client) client.stop();
      client = server.available();
      Serial.println("\n✅ Cliente (Raspberry Pi) conectado");
      wifiConnected = true;
      lastWiFiData = millis();
    }
  }

  // Procesar datos del cliente actual
  if (client && client.connected()) {
    String inputBuffer = "";
    
    while (client.available()) {
      char inChar = (char)client.read();
      
      if (inChar == '\n') {
        parseWiFiData(inputBuffer);
        inputBuffer = "";
      } else if (inChar != '\r') {
        inputBuffer += inChar;
      }
    }
    
    // Procesar último buffer si tiene datos
    if (inputBuffer.length() > 0) {
      parseWiFiData(inputBuffer);
    }
  } else if (wifiConnected) {
    Serial.println("\n⚠ Cliente desconectado");
    wifiConnected = false;
  }

  // Detectar timeout
  if (wifiConnected && (millis() - lastWiFiData > WIFI_TIMEOUT)) {
    Serial.println("\n⚠ Timeout WiFi: Sin datos por 5s");
    // No desconectar, solo advertir
  }
}

// ============================================
// FUNCIONES DE CONVERSIÓN
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
// MOVIMIENTO A ÁNGULOS (NO BLOQUEANTE PARA WIFI)
// ============================================
bool goToAnglesNonBlocking(float m1_targetDegrees, float m2_targetDegrees) {
  // Validar límites
  if (m1_targetDegrees < M1_MIN_ANGLE || m1_targetDegrees > M1_MAX_ANGLE) {
    return false;
  }
  if (m2_targetDegrees < M2_MIN_ANGLE || m2_targetDegrees > M2_MAX_ANGLE) {
    return false;
  }

  long m1_targetTicks = m1_degreesToTicks(m1_targetDegrees);
  long m2_targetTicks = m2_degreesToTicks(m2_targetDegrees);

  noInterrupts();
  long m1_current = m1_encoderCount;
  long m2_current = m2_encoderCount;
  interrupts();

  bool m1_arrived = abs(m1_current - m1_targetTicks) <= TOLERANCE;
  bool m2_arrived = abs(m2_current - m2_targetTicks) <= TOLERANCE;

  if (m1_arrived && m2_arrived) {
    stopAllMotors();
    return true;
  }

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

  return false;  // Aún no llegó
}

// ============================================
// MOVIMIENTO A ÁNGULOS (BLOQUEANTE - PARA DEBUG)
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

  Serial.print("🎯 Moviendo: M1="); Serial.print(m1_targetDegrees, 1);
  Serial.print("° M2="); Serial.print(m2_targetDegrees, 1); Serial.println("°");

  unsigned long startTime = millis();

  while (true) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "stop" || cmd == "s") {
        stopAllMotors();
        Serial.println("⚠ MOVIMIENTO DETENIDO");
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

    delay(10);
  }

  stopAllMotors();
  Serial.println("✅ Posición alcanzada");
  return true;
}

// ============================================
// MOVIMIENTO A COORDENADAS XY
// ============================================
bool goToXY(float x, float y) {
  float m1_target, m2_target;

  if (!inverseKinematicsCalibrated(x, y, m1_target, m2_target)) {
    return false;
  }

  return goToAngles(m1_target, m2_target);
}

// Variables para movimiento continuo WiFi
float currentTargetM1 = 90.0;
float currentTargetM2 = 90.0;
bool isMoving = false;

bool goToXYNonBlocking(float x, float y) {
  float m1_target, m2_target;

  if (!inverseKinematicsCalibrated(x, y, m1_target, m2_target)) {
    return false;
  }

  currentTargetM1 = m1_target;
  currentTargetM2 = m2_target;
  isMoving = true;

  return goToAnglesNonBlocking(m1_target, m2_target);
}

// ============================================
// AYUDA Y ESTADO
// ============================================
void printHelp() {
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║      SCARA XY - CONTROL CON CINEMÁTICA INVERSA      ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.println("\n💡 COMANDOS:");
  Serial.println("  xy X Y    → Mover a coordenadas (ej: xy 2.5 3.0)");
  Serial.println("  c1-c4     → Ir a esquinas");
  Serial.println("  home / 0  → Centro (0, 0)");
  Serial.println("  stop / s  → Detener motores");
  Serial.println("  status    → Ver estado actual");
  Serial.println("  m         → Volver al menú");
  Serial.println("════════════════════════════════════════════════════════\n");
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

  Serial.println("\n📊 ESTADO:");
  Serial.print("  M1: "); Serial.print(m1_angle, 2); Serial.println("°");
  Serial.print("  M2: "); Serial.print(m2_angle, 2); Serial.println("°");
  Serial.print("  XY: ("); Serial.print(x, 2); Serial.print(", ");
  Serial.print(y, 2); Serial.println(")");
}

// ============================================
// PROCESADOR DE COMANDOS (DEBUG)
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
    Serial.println("⚠ MOTORES DETENIDOS");
  }
  else if (cmd == "home" || cmd == "0") {
    goToAngles(90.0, 90.0);
  }
  else if (cmd == "c1") {
    goToXY(-5.0, -5.0);
  }
  else if (cmd == "c2") {
    goToXY(5.0, -5.0);
  }
  else if (cmd == "c3") {
    goToXY(5.0, 5.0);
  }
  else if (cmd == "c4") {
    goToXY(-5.0, 5.0);
  }
  else if (cmd.startsWith("xy ")) {
    int space1 = cmd.indexOf(' ');
    int space2 = cmd.indexOf(' ', space1 + 1);

    if (space2 > 0) {
      float x = cmd.substring(space1 + 1, space2).toFloat();
      float y = cmd.substring(space2 + 1).toFloat();
      goToXY(x, y);
    } else {
      Serial.println("❌ Formato: xy X Y");
    }
  }
  else {
    Serial.println("❌ Comando no reconocido. Escribe 'help'");
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

  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║   SCARA XY - SISTEMA INICIADO                       ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");

  // Calibración
  noInterrupts();
  m1_encoderCount = 0;
  m2_encoderCount = 0;
  interrupts();

  delay(500);
  goToAngles(90.0, 90.0);

  long m1_target_ticks = m1_degreesToTicks(90.0);
  long m2_target_ticks = m2_degreesToTicks(90.0);

  noInterrupts();
  m1_encoderCount = m1_target_ticks;
  m2_encoderCount = m2_target_ticks;
  interrupts();

  Serial.println("✅ Sistema calibrado");
  printWiFiMenu();
  lastDebugPrint = millis();
}

// ============================================
// LOOP
// ============================================
void loop() {
  // Procesar entrada serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    // Comando global para volver al menú
    if (cmd == "m" || cmd == "M") {
      stopAllMotors();
      isMoving = false;
      currentMode = MENU_WIFI;
      printWiFiMenu();
      return;
    }

    if (cmd == "s" || cmd == "stop" || cmd == "S" || cmd == "STOP") {
      stopAllMotors();
      isMoving = false;
      Serial.println("🛑 MOTORES DETENIDOS");
      return;
    }

    if (currentMode == MENU_WIFI) {
      processWiFiMenu(cmd);
    }
    else if (currentMode == DEBUG_MODE) {
      processCommand(cmd);
    }
    else if (currentMode == WIFI_ACTIVE) {
      if (cmd == "d" || cmd == "D") {
        currentMode = DEBUG_MODE;
        Serial.println("🔧 Modo Debug activado");
        printHelp();
      } else {
        processCommand(cmd);
      }
    }
  }

  // ===== MODO WIFI ACTIVO =====
  if (currentMode == WIFI_ACTIVE) {
    // Gestionar conexiones WiFi
    handleWiFi();

    // Procesar nuevos datos de coordenadas
    if (newDataAvailable) {
      newDataAvailable = false;
      
      Serial.print("🚀 Moviendo a (");
      Serial.print(targetX_WiFi, 2); Serial.print(", ");
      Serial.print(targetY_WiFi, 2); Serial.println(")");

      // Calcular cinemática y comenzar movimiento
      float m1_target, m2_target;
      if (inverseKinematicsCalibrated(targetX_WiFi, targetY_WiFi, m1_target, m2_target)) {
        currentTargetM1 = m1_target;
        currentTargetM2 = m2_target;
        isMoving = true;
        
        // Resetear PID
        m1_lastError = 0; m1_integral = 0; m1_currentPWM = 0;
        m2_lastError = 0; m2_integral = 0; m2_currentPWM = 0;
        m1_lastPIDTime = m2_lastPIDTime = millis();
      }
    }

    // Continuar movimiento si está activo
    if (isMoving) {
      bool arrived = goToAnglesNonBlocking(currentTargetM1, currentTargetM2);
      if (arrived) {
        isMoving = false;
        Serial.println("✅ Posición alcanzada");
      }
    }

    // Mostrar estado periódicamente
    if (millis() - lastDebugPrint >= 2000) {
      lastDebugPrint = millis();

      noInterrupts();
      long m1_val = m1_encoderCount;
      long m2_val = m2_encoderCount;
      interrupts();

      float m1_angle = m1_ticksToDegrees(m1_val);
      float m2_angle = m2_ticksToDegrees(m2_val);
      float x, y;
      forwardKinematicsCalibrated(m1_angle, m2_angle, x, y);

      Serial.print("📍 XY("); Serial.print(x, 1);
      Serial.print(","); Serial.print(y, 1); Serial.print(")");
      Serial.print(wifiConnected ? " 🟢" : " 🔴");
      if (isMoving) Serial.print(" 🔄");
      Serial.println();
    }
  }
  // ===== MODO DEBUG =====
  else if (currentMode == DEBUG_MODE) {
    if (millis() - lastDebugPrint >= 3000) {
      lastDebugPrint = millis();

      noInterrupts();
      long m1_val = m1_encoderCount;
      long m2_val = m2_encoderCount;
      interrupts();

      float m1_angle = m1_ticksToDegrees(m1_val);
      float m2_angle = m2_ticksToDegrees(m2_val);
      float x, y;
      forwardKinematicsCalibrated(m1_angle, m2_angle, x, y);

      Serial.print("📍 M1="); Serial.print(m1_angle, 1);
      Serial.print("° M2="); Serial.print(m2_angle, 1);
      Serial.print("° XY("); Serial.print(x, 1);
      Serial.print(","); Serial.print(y, 1); Serial.println(")");
    }
  }
}