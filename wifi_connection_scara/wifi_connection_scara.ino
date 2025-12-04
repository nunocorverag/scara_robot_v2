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

// Variables WiFi
volatile int targetX_WiFi = 150;  // Centro por defecto
volatile int targetY_WiFi = 150;  // Centro por defecto
volatile bool newDataAvailable = false;
unsigned long lastWiFiData = 0;
const unsigned long WIFI_TIMEOUT = 5000;  // 5 segundos sin datos = desconexión

// Variables de control
bool wifiConnected = false;
unsigned long lastDebugPrint = 0;

// ============================================
// PUNTOS DE CALIBRACIÓN (Esquinas conocidas)
// ============================================
struct CalibPoint {
  float m1_angle;
  float m2_angle;
  float x;
  float y;
};

// Puntos calibrados del workspace (centrados en origen -5 a 5)
CalibPoint calibPoints[4] = {
  {96.0,  61.6,  -5.0, -5.0},  // c1: inferior izquierda (CORREGIDO)
  {83.7,  83.1,   5.0, -5.0},  // c2: inferior derecha (CORREGIDO)
  {86.5, 123.2,   5.0,  5.0},  // c3: superior derecha (CORREGIDO)
  {91.6, 100.6,  -5.0,  5.0}   // c4: superior izquierda (CORREGIDO)
};

// ============================================
// CINEMÁTICA INVERSA CON INTERPOLACIÓN
// ============================================
bool inverseKinematicsCalibrated(float x, float y, float& theta1, float& theta2) {
  // Validar límites del workspace
  if (x < WS_X_MIN || x > WS_X_MAX || y < WS_Y_MIN || y > WS_Y_MAX) {
    Serial.print("❌ Fuera de workspace: (");
    Serial.print(x, 2); Serial.print(", "); Serial.print(y, 2); Serial.println(")");
    return false;
  }
  
  // Usar interpolación LINEAL basada en los 4 puntos reales:
  // c1: (-5, -5) → M1=77.0,  M2=85.6
  // c2: (5, -5)  → M1=103.3, M2=69.6
  // c3: (5, 5)   → M1=103.3, M2=95.5
  // c4: (-5, 5)  → M1=79.2,  M2=116.2
  
  // Normalizar a [0, 1]
  float x_norm = (x - (-5.0)) / 10.0;  // -5→0, 5→1
  float y_norm = (y - (-5.0)) / 10.0;  // -5→0, 5→1
  
  // ⭐️ CORRECCIÓN DEL EJE X (INVERSIÓN DE REFERENCIA) ⭐️
  // Si tu robot se va a X negativo cuando pides X positivo,
  // intercambiamos los ángulos de origen y destino en la interpolación horizontal.
  
  // 1. Interpolación para el Lado Inferior (y_norm=0)
  // ANTES: c1 (77.0°) a c2 (103.3°)
  // AHORA: c2 (103.3°) a c1 (77.0°)
  float m1_bottom = 103.3 + (x_norm * (77.0 - 103.3)); // M1 de 103.3 (X=5) a 77.0 (X=-5)
  float m2_bottom = 69.6 + (x_norm * (85.6 - 69.6));   // M2 de 69.6 (X=5) a 85.6 (X=-5)
  
  // 2. Interpolación para el Lado Superior (y_norm=1)
  // ANTES: c4 (79.2°) a c3 (103.3°)
  // AHORA: c3 (103.3°) a c4 (79.2°)
  float m1_top = 103.3 + (x_norm * (79.2 - 103.3)); // M1 de 103.3 (X=5) a 79.2 (X=-5)
  float m2_top = 95.5 + (x_norm * (116.2 - 95.5));  // M2 de 95.5 (X=5) a 116.2 (X=-5)
  
  // 3. Interpolar verticalmente
  float t1 = m1_bottom + (y_norm * (m1_top - m1_bottom));
  float t2 = m2_bottom + (y_norm * (m2_top - m2_bottom));
  
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
  // Usar interpolación ponderada basada en los 4 puntos calibrados
  // Puntos reales:
  // c1: M1=77.0,  M2=85.6   → X=-5, Y=-5
  // c2: M1=103.3, M2=69.6   → X=5,  Y=-5
  // c3: M1=103.3, M2=95.5   → X=5,  Y=5
  // c4: M1=79.2,  M2=116.2  → X=-5, Y=5
  
  // Calcular distancia a cada punto en espacio de ángulos
  float distances[4];
  float weights[4];
  float totalWeight = 0;
  
  // Punto 1: (77.0, 85.6)
  distances[0] = sqrt(pow(theta1 - 77.0, 2) + pow(theta2 - 85.6, 2));
  
  // Punto 2: (103.3, 69.6)
  distances[1] = sqrt(pow(theta1 - 103.3, 2) + pow(theta2 - 69.6, 2));
  
  // Punto 3: (103.3, 95.5)
  distances[2] = sqrt(pow(theta1 - 103.3, 2) + pow(theta2 - 95.5, 2));
  
  // Punto 4: (79.2, 116.2)
  distances[3] = sqrt(pow(theta1 - 79.2, 2) + pow(theta2 - 116.2, 2));
  
  // Calcular pesos (inversa de distancia)
  for (int i = 0; i < 4; i++) {
    weights[i] = 1.0 / (distances[i] + 0.1);  // +0.1 evita división por cero
    totalWeight += weights[i];
  }
  
  // Normalizar pesos
  for (int i = 0; i < 4; i++) {
    weights[i] = weights[i] / totalWeight;
  }
  
  // Coordenadas de los puntos
  float calib_x[4] = {-5.0, 5.0, 5.0, -5.0};
  float calib_y[4] = {-5.0, -5.0, 5.0, 5.0};
  
  // Interpolación ponderada
  x = 0;
  y = 0;
  for (int i = 0; i < 4; i++) {
    x += calib_x[i] * weights[i];
    y += calib_y[i] * weights[i];
  }
}

// Modo de operación
enum OperationMode {
  MENU_WIFI,
  DEBUG_MODE,
  WIFI_ACTIVE
};

OperationMode currentMode = MENU_WIFI;

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
  // Entrada: 0-300 (WiFi)
  // Salida: -5 a 5 (SCARA)
  // Centro: 150 → 0
  
  // Restar 150 para centrar (150→0, 0→-150, 300→150)
  int centered_x = wifi_x - 150;
  int centered_y = wifi_y - 150;
  
  // Mapear de ±150 a ±5
  scara_x = (float)centered_x / 30.0;  // 150/30 = 5
  scara_y = (float)centered_y / 30.0;
  
  // Constrain a límites de seguridad
  scara_x = constrain(scara_x, -5.0, 5.0);
  scara_y = constrain(scara_y, -5.0, 5.0);
}

// ============================================
// PARSEO DE DATOS WiFi
// ============================================
void parseWiFiData(String data) {
  data.trim();
  
  // Formato esperado: $X,Y*
  if (data.startsWith("$") && data.indexOf('*') > 0) {
    int asteriskIndex = data.indexOf('*');
    String content = data.substring(1, asteriskIndex);
    
    int commaIndex = content.indexOf(',');
    if (commaIndex > 0) {
      String xStr = content.substring(0, commaIndex);
      String yStr = content.substring(commaIndex + 1);
      
      int x = xStr.toInt();
      int y = yStr.toInt();
      
      // Validar rango (0-300)
      if (x >= 0 && x <= 300 && y >= 0 && y <= 300) {
        targetX_WiFi = x;
        targetY_WiFi = y;
        newDataAvailable = true;
        lastWiFiData = millis();
        
        Serial.print("\n📡 WiFi: X=");
        Serial.print(x); Serial.print(" Y=");
        Serial.println(y);
      } else {
        Serial.println("❌ Coordenadas fuera de rango (0-300)");
      }
    }
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
      Serial.println("\n✅ Cliente WiFi conectado");
      wifiConnected = true;
    }
  }
  
  // Procesar datos del cliente actual
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
  
  // Detectar timeout (sin datos por 5 segundos)
  if (wifiConnected && (millis() - lastWiFiData > WIFI_TIMEOUT)) {
    Serial.println("\n⚠  Timeout WiFi: Sin datos por 5s");
    wifiConnected = false;
  }
}
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
// MAPEO GUIADO DEL PLANO (PASO A PASO)
// ============================================
void guidedPlaneMapping(int gridSize) {
  if (gridSize < 2) {
    Serial.println("❌ Mínimo 2 puntos por lado");
    return;
  }
  
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║      MAPEO GUIADO DEL PLANO (PASO A PASO)           ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.print("Rango: X[-5 a 5] Y[-5 a 5]\n");
  Serial.print("Grid: "); Serial.print(gridSize); Serial.print("x");
  Serial.print(gridSize); Serial.println(" puntos");
  Serial.println("\n⏸️  Presiona 's' para siguiente, 'q' para salir");
  
  float step_x = 10.0 / (gridSize - 1);
  float step_y = 10.0 / (gridSize - 1);
  
  int pointCount = 0;
  int totalPoints = gridSize * gridSize;
  
  // Almacenar datos para exportar
  String dataLog = "MAPEO_GUIADO_SCARA\n";
  dataLog += "X,Y,M1_Calc,M2_Calc,M1_Real,M2_Real,X_Real,Y_Real,ErrorX,ErrorY\n";
  
  for (int row = 0; row < gridSize; row++) {
    float y = -5.0 + (step_y * row);
    
    for (int col = 0; col < gridSize; col++) {
      float x = -5.0 + (step_x * col);
      pointCount++;
      
      // Mostrar información del punto
      Serial.print("\n┌─────────────────────────────────────────────┐\n");
      Serial.print("│ Punto [");
      if (pointCount < 10) Serial.print("0");
      Serial.print(pointCount); Serial.print("/"); 
      if (totalPoints < 10) Serial.print("0");
      Serial.print(totalPoints); Serial.println("]");
      Serial.print("└─────────────────────────────────────────────┘\n");
      
      Serial.print("📍 Destino XY: (");
      Serial.print(x, 2); Serial.print(", "); Serial.print(y, 2);
      Serial.println(") cm");
      
      // Calcular ángulos
      float m1_angle, m2_angle;
      if (!inverseKinematicsCalibrated(x, y, m1_angle, m2_angle)) {
        Serial.println("❌ Posición inalcanzable");
        Serial.println("\n⏸️  Escribe 's' para siguiente, 'q' para salir: ");
        
        // Esperar input
        while (true) {
          if (Serial.available()) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            input.toLowerCase();
            if (input == "s") break;
            else if (input == "q") return;
          }
        }
        continue;
      }
      
      Serial.print("🎯 Ángulos calculados:\n");
      Serial.print("   M1 = "); Serial.print(m1_angle, 2); Serial.println("°");
      Serial.print("   M2 = "); Serial.print(m2_angle, 2); Serial.println("°");
      
      // Ejecutar movimiento
      if (goToAngles(m1_angle, m2_angle)) {
        // Verificar posición real
        noInterrupts();
        long m1_f = m1_encoderCount;
        long m2_f = m2_encoderCount;
        interrupts();
        
        float m1_real = m1_ticksToDegrees(m1_f);
        float m2_real = m2_ticksToDegrees(m2_f);
        float x_real, y_real;
        forwardKinematicsCalibrated(m1_real, m2_real, x_real, y_real);
        
        float error_x = abs(x_real - x);
        float error_y = abs(y_real - y);
        
        Serial.print("\n✅ Posición alcanzada:\n");
        Serial.print("   XY Real: ("); Serial.print(x_real, 2);
        Serial.print(", "); Serial.print(y_real, 2); Serial.println(") cm");
        Serial.print("   M1 Real: "); Serial.print(m1_real, 2); Serial.println("°");
        Serial.print("   M2 Real: "); Serial.print(m2_real, 2); Serial.println("°");
        
        Serial.print("\n📊 Error:\n");
        Serial.print("   ΔX = "); Serial.print(error_x, 3); Serial.println(" cm");
        Serial.print("   ΔY = "); Serial.print(error_y, 3); Serial.println(" cm");
        
        // Guardar en log
        dataLog += String(x, 2) + "," + String(y, 2) + ",";
        dataLog += String(m1_angle, 2) + "," + String(m2_angle, 2) + ",";
        dataLog += String(m1_real, 2) + "," + String(m2_real, 2) + ",";
        dataLog += String(x_real, 2) + "," + String(y_real, 2) + ",";
        dataLog += String(error_x, 3) + "," + String(error_y, 3) + "\n";
      }
      
      // Esperar comando
      Serial.println("\n⏸️  Escribe 's' para siguiente, 'q' para salir: ");
      while (true) {
        if (Serial.available()) {
          String input = Serial.readStringUntil('\n');
          input.trim();
          input.toLowerCase();
          if (input == "s") {
            break;
          }
          else if (input == "q") {
            Serial.println("\n🔴 Mapeo cancelado");
            // Mostrar datos recolectados
            Serial.println("\n📋 DATOS RECOLECTADOS:");
            Serial.println(dataLog);
            return;
          }
        }
      }
    }
  }
  
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║      MAPEO GUIADO COMPLETADO                        ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  
  // Mostrar log completo
  Serial.println("\n📋 DATOS COMPLETOS DEL MAPEO:");
  Serial.println("════════════════════════════════════════════════════════");
  Serial.println(dataLog);
  Serial.println("════════════════════════════════════════════════════════");
  
  // Volver a origen
  Serial.println("\n🏠 Regresando a origen (0, 0)...");
  goToXY(0.0, 0.0);
}
void fullPlaneMapping(int gridSize) {
  if (gridSize < 2) {
    Serial.println("❌ Mínimo 2 puntos por lado");
    return;
  }
  
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║         MAPEO COMPLETO DEL PLANO                    ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.print("Rango: X[-5 a 5] Y[-5 a 5]\n");
  Serial.print("Grid: "); Serial.print(gridSize); Serial.print("x");
  Serial.print(gridSize); Serial.println(" puntos");
  
  float step_x = 10.0 / (gridSize - 1);  // De -5 a 5
  float step_y = 10.0 / (gridSize - 1);
  
  int pointCount = 0;
  int totalPoints = gridSize * gridSize;
  
  // Recorrer el plano de izquierda a derecha, abajo a arriba
  for (int row = 0; row < gridSize; row++) {
    float y = -5.0 + (step_y * row);
    
    Serial.print("\n📍 Fila Y="); Serial.print(y, 1); Serial.println("°");
    Serial.println("─────────────────────────────────────────────────");
    
    for (int col = 0; col < gridSize; col++) {
      float x = -5.0 + (step_x * col);
      pointCount++;
      
      Serial.print("  [");
      if (pointCount < 10) Serial.print("0");
      Serial.print(pointCount); Serial.print("/"); 
      if (totalPoints < 10) Serial.print("0");
      Serial.print(totalPoints); Serial.print("] XY(");
      Serial.print(x, 1); Serial.print(","); Serial.print(y, 1);
      Serial.print(") → ");
      
      // Calcular ángulos
      float m1_angle, m2_angle;
      if (!inverseKinematicsCalibrated(x, y, m1_angle, m2_angle)) {
        Serial.println("❌ Inalcanzable");
        continue;
      }
      
      // Mostrar ángulos teóricos
      Serial.print("M1="); Serial.print(m1_angle, 1);
      Serial.print("° M2="); Serial.print(m2_angle, 1); Serial.print("° → ");
      
      // Ejecutar movimiento
      if (goToAngles(m1_angle, m2_angle)) {
        // Verificar posición final
        noInterrupts();
        long m1_f = m1_encoderCount;
        long m2_f = m2_encoderCount;
        interrupts();
        
        float m1_real = m1_ticksToDegrees(m1_f);
        float m2_real = m2_ticksToDegrees(m2_f);
        float x_real, y_real;
        forwardKinematicsCalibrated(m1_real, m2_real, x_real, y_real);
        
        Serial.print("✓ Real: XY(");
        Serial.print(x_real, 1); Serial.print(",");
        Serial.print(y_real, 1); Serial.print(") M1=");
        Serial.print(m1_real, 1); Serial.print("° M2=");
        Serial.print(m2_real, 1); Serial.println("°");
        
        // Mostrar error
        float error_x = abs(x_real - x);
        float error_y = abs(y_real - y);
        if (error_x > 0.2 || error_y > 0.2) {
          Serial.print("    ⚠  Error: ΔX="); Serial.print(error_x, 2);
          Serial.print(" ΔY="); Serial.print(error_y, 2); Serial.println(" cm");
        }
      }
      
      delay(300);
      
      // Permitir stop
      if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toLowerCase();
        if (input == "s" || input == "stop") {
          Serial.println("\n⚠  Mapeo detenido por usuario");
          return;
        }
      }
    }
  }
  
  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║         MAPEO COMPLETADO                            ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  
  // Volver a origen
  Serial.println("\n🏠 Regresando a origen (0, 0)...");
  goToXY(0.0, 0.0);
}
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
  Serial.println("  c1 → (-5.0, -5.0) cm   Esquina inferior izquierda");
  Serial.println("  c2 → (5.0, -5.0) cm    Esquina inferior derecha");
  Serial.println("  c3 → (5.0, 5.0) cm     Esquina superior derecha");
  Serial.println("  c4 → (-5.0, 5.0) cm    Esquina superior izquierda");
  
  Serial.println("\n💡 COMANDOS DE MOVIMIENTO:");
  Serial.println("  xy X Y          → Mover a coordenadas XY");
  Serial.println("                    Ejemplo: xy 2.5 3.0");
  Serial.println("  c1 / c2 / c3 / c4 → Ir a esquina predefinida");
  Serial.println("  home            → Centro (0.0, 0.0 cm)");
  Serial.println("  0               → Alias de home");
  Serial.println("  rect N          → Trayectoria rectangular");
  Serial.println("                    Ejemplo: rect 5");
  Serial.println("  map N           → Mapeo automático NxN grid");
  Serial.println("                    Ejemplo: map 3");
  Serial.println("  gmap N          → Mapeo guiado paso a paso");
  Serial.println("                    Ejemplo: gmap 3");
  Serial.println("                    (Presiona 's' siguiente, 'q' salir)");
  
  Serial.println("\n🛑 CONTROL:");
  Serial.println("  stop            → Detener movimiento actual");
  Serial.println("  s               → Alias de stop");
  
  Serial.println("\n📊 INFORMACIÓN:");
  Serial.println("  status          → Ver estado actual de motores y posición");
  Serial.println("  help            → Mostrar esta ayuda");
  Serial.println("  h               → Alias de help");
  
  Serial.println("\n⚙  CONFIGURACIÓN:");
  Serial.println("  reset           → Resetear contadores de encoders a 0");
  
  Serial.println("\n🔙 GLOBALES (funciona en cualquier modo):");
  Serial.println("  m               → Volver al menú principal");
  Serial.println("  s / stop        → Detener movimiento actual");
  
  Serial.println("\n📌 NOTAS:");
  Serial.println("  • Todos los comandos son case-insensitive");
  Serial.println("  • Durante movimiento, escribe 'stop' para detener");
  Serial.println("  • Las coordenadas XY están en centímetros");
  Serial.println("  • Rango válido: X[-5 a 5 cm] Y[-5 a 5 cm]");
  Serial.println("  • Origen (0,0) en el centro del workspace");
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
  
  Serial.print("\n➜ Comando recibido: '"); Serial.print(cmd); Serial.println("'");
  
  if (cmd == "help" || cmd == "h") {
    Serial.println("📖 Mostrando ayuda...");
    printHelp();
  }
  else if (cmd == "status") {
    Serial.println("📊 Obteniendo estado del sistema...");
    printStatus();
  }
  else if (cmd == "stop" || cmd == "s") {
    Serial.println("🛑 Deteniendo motores...");
    stopAllMotors();
    Serial.println("⚠  MOTORES DETENIDOS");
  }
  else if (cmd == "home" || cmd == "0") {
    Serial.println("🏠 Moviendo a posición HOME (0.0, 0.0 cm)...");
    // Usar ángulos calibrados para home (medidos como exactamente 0,0)
    goToAngles(90.0, 90.0);
  }
  else if (cmd == "c1") {
    Serial.println("🎯 Moviendo a esquina C1 (-5.0, -5.0 cm)...");
    goToXY(-5.0, -5.0);
  }
  else if (cmd == "c2") {
    Serial.println("🎯 Moviendo a esquina C2 (5.0, -5.0 cm)...");
    goToXY(5.0, -5.0);
  }
  else if (cmd == "c3") {
    Serial.println("🎯 Moviendo a esquina C3 (5.0, 5.0 cm)...");
    goToXY(5.0, 5.0);
  }
  else if (cmd == "c4") {
    Serial.println("🎯 Moviendo a esquina C4 (-5.0, 5.0 cm)...");
    goToXY(-5.0, 5.0);
  }
  else if (cmd == "reset") {
    Serial.println("🔄 Reseteando contadores de encoders...");
    noInterrupts();
    m1_encoderCount = 0;
    m2_encoderCount = 0;
    interrupts();
    Serial.println("✓ Encoders reiniciados a 0");
  }
  else if (cmd.startsWith("xy ")) {
    int space1 = cmd.indexOf(' ');
    int space2 = cmd.indexOf(' ', space1 + 1);
    
    if (space2 > 0) {
      float x = cmd.substring(space1 + 1, space2).toFloat();
      float y = cmd.substring(space2 + 1).toFloat();
      Serial.print("📍 Moviendo a coordenadas XY (");
      Serial.print(x, 2); Serial.print(", "); Serial.print(y, 2); Serial.println(" cm)...");
      goToXY(x, y);
    } else {
      Serial.println("❌ Formato: xy X Y (Ejemplo: xy 4.25 2.9)");
    }
  }
  else if (cmd.startsWith("rect ")) {
    int space = cmd.indexOf(' ');
    int points = cmd.substring(space + 1).toInt();
    if (points >= 2) {
      Serial.print("📦 Iniciando trayectoria rectangular con ");
      Serial.print(points); Serial.println(" puntos por lado...");
      rectangularPath(points);
    } else {
      Serial.println("❌ Error: mínimo 2 puntos por lado (Ejemplo: rect 5)");
    }
  }
  else if (cmd.startsWith("map ")) {
    int space = cmd.indexOf(' ');
    int gridSize = cmd.substring(space + 1).toInt();
    if (gridSize >= 2) {
      Serial.print("🗺️  Iniciando mapeo automático con ");
      Serial.print(gridSize); Serial.print("x");
      Serial.print(gridSize); Serial.println(" grid...");
      fullPlaneMapping(gridSize);
    } else {
      Serial.println("❌ Error: mínimo 2 puntos por lado (Ejemplo: map 5)");
    }
  }
  else if (cmd.startsWith("gmap ")) {
    int space = cmd.indexOf(' ');
    int gridSize = cmd.substring(space + 1).toInt();
    if (gridSize >= 2) {
      Serial.print("🎮 Iniciando mapeo guiado con ");
      Serial.print(gridSize); Serial.print("x");
      Serial.print(gridSize); Serial.println(" grid...");
      guidedPlaneMapping(gridSize);
    } else {
      Serial.println("❌ Error: mínimo 2 puntos (Ejemplo: gmap 3)");
    }
  }
  else {
    Serial.print("✗ Comando no reconocido: '");
    Serial.print(cmd);
    Serial.println("'");
    Serial.println("  Escribe 'help' para ver comandos disponibles");
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
  
  // Mostrar menú inicial
  printWiFiMenu();
  lastDebugPrint = millis();
}

// ============================================
// LOOP
// ============================================
void loop() {
  // Procesar entrada serial según modo
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    
    // Comando global: regresar al menú desde cualquier modo
    if (cmd == "m") {
      currentMode = MENU_WIFI;
      printWiFiMenu();
      return;
    }
    
    if (currentMode == MENU_WIFI) {
      processWiFiMenu(cmd);
    } 
    else if (currentMode == DEBUG_MODE) {
      processCommand(cmd);
    }
    else if (currentMode == WIFI_ACTIVE) {
      // En modo WiFi, acepta comandos SCARA normales
      if (cmd == "d") {
        currentMode = DEBUG_MODE;
        Serial.println("🔧 Debug mode");
        printHelp();
      } else {
        processCommand(cmd);  // Comandos SCARA normales
      }
    }
  }
  
  // Modo WiFi: Gestionar conexión y movimiento automático
  if (currentMode == WIFI_ACTIVE) {
    handleWiFi();
    
    // Estado cada 3s
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
      Serial.print(","); Serial.print(y, 1); Serial.print(")");
      
      if (wifiConnected) {
        Serial.print(" 🟢");
      } else {
        Serial.print(" 🔴");
      }
      Serial.println();
    }
    
    // Movimiento automático WiFi
    if (newDataAvailable) {
      newDataAvailable = false;
      
      float scara_x, scara_y;
      convertWiFiToSCARA(targetX_WiFi, targetY_WiFi, scara_x, scara_y);
      
      Serial.print("\n🚀 Moviendo a (");
      Serial.print(scara_x, 2); Serial.print(", ");
      Serial.print(scara_y, 2); Serial.println(")");
      
      goToXY(scara_x, scara_y);
    }
  }
  // Modo Debug: Mostrar estado
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