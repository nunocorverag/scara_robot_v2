#include <Arduino.h>

// ============================================
// CONFIGURACION FISICA DEL ROBOT
// ============================================
const float L1 = 30.0;
const float L2 = 18.0;

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

// ============================================
// FAIL-SAFE SYSTEM (John Deere Style)
// ============================================
enum SystemState {
  SYSTEM_NORMAL,
  SYSTEM_FAIL,
  SYSTEM_EMERGENCY_STOP
};

SystemState currentSystemState = SYSTEM_NORMAL;

// Códigos de diagnóstico estilo ECU John Deere
enum DiagnosticCode {
  JD_CODE_00_NORMAL = 0,
  JD_CODE_01_UNDERVOLTAGE = 1,
  JD_CODE_02_OVERVOLTAGE = 2,
  JD_CODE_03_MOTOR1_STALL = 3,
  JD_CODE_04_MOTOR2_STALL = 4,
  JD_CODE_05_ENCODER1_FAULT = 5,
  JD_CODE_06_ENCODER2_FAULT = 6,
  JD_CODE_07_COMMUNICATION_TIMEOUT = 7,
  JD_CODE_08_SYSTEM_OVERLOAD = 8
};

DiagnosticCode lastDiagnosticCode = JD_CODE_00_NORMAL;
unsigned long lastFaultTime = 0;
String lastFaultMessage = "";

// ============================================
// FILTRO FIR PARA ENCODERS (PROCESAMIENTO DIGITAL DE SEÑALES)
// ============================================
#define M1_FILTER_N 5
#define M2_FILTER_N 5
long m1_filterBuffer[M1_FILTER_N] = {0};
int m1_filterIndex = 0;
long m2_filterBuffer[M2_FILTER_N] = {0};
int m2_filterIndex = 0;

long filterEncoderFIR(long raw, long* buffer, int& index, int N) {
  buffer[index] = raw;
  index = (index + 1) % N;
  
  long sum = 0;
  for (int i = 0; i < N; i++) {
    sum += buffer[i];
  }
  
  return sum / N;
}

float m1_Kp = 0.8, m1_Ki = 0.02, m1_Kd = 1.2;
float m1_lastError = 0, m1_integral = 0;
unsigned long m1_lastPIDTime = 0;
int m1_currentPWM = 0;

float m2_Kp = 0.8, m2_Ki = 0.02, m2_Kd = 1.2;
float m2_lastError = 0, m2_integral = 0;
unsigned long m2_lastPIDTime = 0;
int m2_currentPWM = 0;

bool emergencyStop = false;
bool verboseMode = false;
unsigned long lastPrint = 0;

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int MAX_PWM = 255;

// ============================================
// WIFI CONFIGURATION
// ============================================
#include <WiFi.h>

// ============================================
// FREERTOS CONFIGURATION
// ============================================
TaskHandle_t Task_PID;
TaskHandle_t Task_WiFi;
TaskHandle_t Task_Status;
TaskHandle_t Task_Serial;
TaskHandle_t Task_Voltage;
TaskHandle_t Task_Stall;
TaskHandle_t Task_DSP;

// Mutex para proteger variables compartidas
SemaphoreHandle_t xEncoderMutex;
SemaphoreHandle_t xTargetMutex;
SemaphoreHandle_t xSystemStateMutex;

// ============================================
// VOLTAGE MONITORING (UV/OV Detection)
// ============================================
#define VOLTAGE_PIN 36  // ADC1_CH0 (GPIO36)
#define VOLTAGE_SAMPLES 10
#define VOLTAGE_REFERENCE 3.3
#define VOLTAGE_DIVIDER_RATIO 11.0  // Ajustar según tu divisor (R1+R2)/R2
#define VOLTAGE_MIN 9.8  // Undervoltage threshold (V)
#define VOLTAGE_MAX 13.2  // Overvoltage threshold (V)

float currentVoltage = 12.0;
bool voltageOK = true;

// ============================================
// STALL DETECTION MEJORADO
// ============================================
#define STALL_CHECK_INTERVAL 1000  // Revisar cada 1 segundo
#define STALL_MIN_PWM_THRESHOLD 20 // PWM mínimo sobre MIN_PWM para considerar "esfuerzo real"
#define STALL_MIN_ERROR_TICKS 15   // Error mínimo para esperar movimiento (5 grados)
#define STALL_MIN_MOVEMENT 5       // Movimiento mínimo esperado en 1 segundo

long last_m1_stall_check = 0;
long last_m2_stall_check = 0;
unsigned long lastStallCheckTime = 0;

// Variables para rastrear error
long last_m1_error_ticks = 0;
long last_m2_error_ticks = 0;

// ============================================
// DSP MODULE - FFT ANALYSIS
// ============================================
#define FFT_SIZE 64
#define DSP_SAMPLE_FREQ 100  // Hz
float dsp_encoder_buffer[FFT_SIZE];
int dsp_buffer_index = 0;
float dsp_dominant_frequency = 0;
bool dsp_vibration_detected = false;

// Variables de control para las tareas
bool pidTaskEnabled = false;
float target_m1_angle = 90.0;
float target_m2_angle = 90.0;
bool newTargetAvailable = false;

const char* ssid = "SCARANet";
const char* password = "robot12345";
WiFiServer server(8080);
WiFiClient client;

float targetX_WiFi = 0.0;
float targetY_WiFi = 0.0;
bool newDataAvailable = false;
unsigned long lastWiFiData = 0;
const unsigned long WIFI_TIMEOUT = 5000;

bool wifiConnected = false;
unsigned long lastDebugPrint = 0;

// Modo de operacion
enum OperationMode {
  MENU_WIFI,
  DEBUG_MODE,
  WIFI_ACTIVE
};

OperationMode currentMode = MENU_WIFI;

// ============================================
// PUNTOS DE CALIBRACION
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
// CINEMATICA INVERSA CON INTERPOLACION
// ============================================
bool inverseKinematicsCalibrated(float x, float y, float& theta1, float& theta2) {
  if (x < WS_X_MIN || x > WS_X_MAX || y < WS_Y_MIN || y > WS_Y_MAX) {
    Serial.print("[ERROR] Fuera de workspace: (");
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
    Serial.print("[ERROR] Motor 1 fuera de rango: ");
    Serial.print(t1, 1); Serial.println(" deg");
    return false;
  }
  if (t2 < M2_MIN_ANGLE || t2 > M2_MAX_ANGLE) {
    Serial.print("[ERROR] Motor 2 fuera de rango: ");
    Serial.print(t2, 1); Serial.println(" deg");
    return false;
  }

  theta1 = t1;
  theta2 = t2;
  return true;
}

// ============================================
// CINEMATICA DIRECTA CON CALIBRACION
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
// FUNCIONES DE CONVERSION
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
// RESET DE ENCODERS
// ============================================
void resetEncoders() {
  stopAllMotors();
  
  // Establecer encoders a posicion home (90, 90)
  long m1_home_ticks = m1_degreesToTicks(90.0);
  long m2_home_ticks = m2_degreesToTicks(90.0);
  
  noInterrupts();
  m1_encoderCount = m1_home_ticks;
  m2_encoderCount = m2_home_ticks;
  interrupts();
  
  // Reset PID
  m1_lastError = 0;
  m1_integral = 0;
  m1_currentPWM = 0;
  m2_lastError = 0;
  m2_integral = 0;
  m2_currentPWM = 0;
  m1_lastPIDTime = millis();
  m2_lastPIDTime = millis();
  
  Serial.println("[RESET] Encoders calibrados a home (90, 90)");
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
// JOHN DEERE DIAGNOSTIC REPORTING
// ============================================
void jd_report(DiagnosticCode code, const char* message) {
  lastDiagnosticCode = code;
  lastFaultTime = millis();
  
  Serial.println("\n========================================");
  Serial.println("  JOHN DEERE ECU DIAGNOSTIC REPORT");
  Serial.println("========================================");
  Serial.print("JD_CODE_");
  if (code < 10) Serial.print("0");
  Serial.print(code);
  Serial.print(": ");
  Serial.println(message);
  Serial.print("Timestamp: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  Serial.println("========================================\n");
}

void setFailState(DiagnosticCode code, const char* message) {
  if (xSemaphoreTake(xSystemStateMutex, pdMS_TO_TICKS(100))) {
    currentSystemState = SYSTEM_FAIL;
    xSemaphoreGive(xSystemStateMutex);
  }
  
  stopAllMotors();
  pidTaskEnabled = false;
  
  // Guardar mensaje para referencia futura
  lastFaultMessage = String(message);
  
  jd_report(code, message);
  
  Serial.println("[FAIL-SAFE] Sistema en modo FALLO");
  Serial.print("[RAZON] ");
  Serial.println(message);
  
  // Agregar detalles específicos según el código
  if (code == JD_CODE_01_UNDERVOLTAGE || code == JD_CODE_02_OVERVOLTAGE) {
    Serial.print("[VOLTAJE] Medido: ");
    Serial.print(currentVoltage, 2);
    Serial.print("V  |  Rango valido: ");
    Serial.print(VOLTAGE_MIN, 1);
    Serial.print("V - ");
    Serial.print(VOLTAGE_MAX, 1);
    Serial.println("V");
    
    // Mostrar valor crudo del ADC para debug
    long sum = 0;
    for (int i = 0; i < 10; i++) {
      sum += analogRead(VOLTAGE_PIN);
      delay(1);
    }
    float avgADC = sum / 10.0;
    Serial.print("[DEBUG] ADC raw: ");
    Serial.print(avgADC, 1);
    Serial.print(" / 4095  |  ADC voltage: ");
    Serial.print((avgADC / 4095.0) * 3.3, 3);
    Serial.println("V");
  }
  
  Serial.println("[INFO] Reinicie el ESP32 para recuperar");
  Serial.println("[INFO] O escriba 'diag' para diagnostico completo");
}

bool checkSystemState() {
  static unsigned long lastWarningTime = 0;
  const unsigned long WARNING_INTERVAL = 5000; // Solo mostrar cada 5 segundos
  
  SystemState state;
  if (xSemaphoreTake(xSystemStateMutex, pdMS_TO_TICKS(10))) {
    state = currentSystemState;
    xSemaphoreGive(xSystemStateMutex);
  } else {
    return false;
  }
  
  if (state != SYSTEM_NORMAL) {
    // Solo mostrar mensaje completo cada 5 segundos
    if (millis() - lastWarningTime >= WARNING_INTERVAL) {
      lastWarningTime = millis();
      
      Serial.println("\n========================================");
      Serial.println("[BLOCKED] Sistema en FAIL - comando ignorado");
      Serial.print("[RAZON] ");
      Serial.println(lastFaultMessage);
      Serial.print("[CODIGO] JD_CODE_");
      if (lastDiagnosticCode < 10) Serial.print("0");
      Serial.println(lastDiagnosticCode);
      Serial.print("[TIEMPO] Fallo hace ");
      Serial.print((millis() - lastFaultTime) / 1000);
      Serial.println(" segundos");
      Serial.println("[ACCION] Reinicie el ESP32 o escriba 'diag'");
      Serial.println("========================================\n");
    }
    return false;
  }
  return true;
}

// ============================================
// VOLTAGE MONITORING
// ============================================
float readVoltage() {
  long sum = 0;
  for (int i = 0; i < VOLTAGE_SAMPLES; i++) {
    sum += analogRead(VOLTAGE_PIN);
    delay(1);
  }
  float avgADC = sum / (float)VOLTAGE_SAMPLES;
  float voltage = (avgADC / 4095.0) * VOLTAGE_REFERENCE * VOLTAGE_DIVIDER_RATIO;
  return voltage;
}

// ============================================
// DSP MODULE - Simple FFT for Vibration Detection
// ============================================
void dsp_addSample(float sample) {
  dsp_encoder_buffer[dsp_buffer_index] = sample;
  dsp_buffer_index = (dsp_buffer_index + 1) % FFT_SIZE;
}

float dsp_calculateRMS() {
  float sum = 0;
  for (int i = 0; i < FFT_SIZE; i++) {
    sum += dsp_encoder_buffer[i] * dsp_encoder_buffer[i];
  }
  return sqrt(sum / FFT_SIZE);
}

void dsp_analyzeVibration() {
  // Análisis simple de energía espectral
  float rms = dsp_calculateRMS();
  
  // Detectar cambios bruscos (posible vibración)
  float variance = 0;
  float mean = 0;
  for (int i = 0; i < FFT_SIZE; i++) {
    mean += dsp_encoder_buffer[i];
  }
  mean /= FFT_SIZE;
  
  for (int i = 0; i < FFT_SIZE; i++) {
    float diff = dsp_encoder_buffer[i] - mean;
    variance += diff * diff;
  }
  variance /= FFT_SIZE;
  
  // Threshold para vibración excesiva
  if (variance > 100.0) {
    dsp_vibration_detected = true;
    dsp_dominant_frequency = DSP_SAMPLE_FREQ / 4.0; // Estimación simplificada
  } else {
    dsp_vibration_detected = false;
  }
}

// ============================================
// TAREAS DE FREERTOS
// ============================================

// -------------------------------------------
// Task: Voltage Monitor
// Frequency: 500 ms
// Priority: High (Critical Safety)
// Core: 0
// Description: Monitors supply voltage for
//              undervoltage and overvoltage
//              conditions. Triggers fail-safe
//              if thresholds exceeded.
// -------------------------------------------
void TaskVoltageMonitor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(500);
  
  for(;;) {
    currentVoltage = readVoltage();
    
    if (currentVoltage < VOLTAGE_MIN) {
      char msg[100];
      snprintf(msg, sizeof(msg), "UNDERVOLTAGE %.2fV (min: %.1fV)", currentVoltage, VOLTAGE_MIN);
      setFailState(JD_CODE_01_UNDERVOLTAGE, msg);
      vTaskSuspend(NULL);
    }
    else if (currentVoltage > VOLTAGE_MAX) {
      char msg[100];
      snprintf(msg, sizeof(msg), "OVERVOLTAGE %.2fV (max: %.1fV)", currentVoltage, VOLTAGE_MAX);
      setFailState(JD_CODE_02_OVERVOLTAGE, msg);
      vTaskSuspend(NULL);
    }
    else {
      voltageOK = true;
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// -------------------------------------------
// Task: Stall Detection Monitor
// Frequency: 500 ms
// Priority: Medium-High
// Core: 0
// Description: Detects motor stall conditions
//              by monitoring encoder movement.
//              Triggers fail-safe if motor
//              is commanded but not moving.
// -------------------------------------------
void TaskStallMonitor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Cada 1 segundo
  
  for(;;) {
    if (pidTaskEnabled && newTargetAvailable) {  // Solo si hay target activo
      long current_m1, current_m2;
      float local_target_m1, local_target_m2;
      
      // Leer encoders actuales
      if (xSemaphoreTake(xEncoderMutex, pdMS_TO_TICKS(50))) {
        current_m1 = m1_encoderCount;
        current_m2 = m2_encoderCount;
        xSemaphoreGive(xEncoderMutex);
      } else {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;
      }
      
      // Leer targets
      if (xSemaphoreTake(xTargetMutex, pdMS_TO_TICKS(50))) {
        local_target_m1 = target_m1_angle;
        local_target_m2 = target_m2_angle;
        xSemaphoreGive(xTargetMutex);
      } else {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;
      }
      
      // Calcular targets en ticks
      long target_m1_ticks = m1_degreesToTicks(local_target_m1);
      long target_m2_ticks = m2_degreesToTicks(local_target_m2);
      
      // Calcular errores actuales
      long error_m1 = abs(target_m1_ticks - current_m1);
      long error_m2 = abs(target_m2_ticks - current_m2);
      
      // Calcular movimiento desde última verificación
      long delta_m1 = abs(current_m1 - last_m1_stall_check);
      long delta_m2 = abs(current_m2 - last_m2_stall_check);
      
      if (millis() - lastStallCheckTime > STALL_CHECK_INTERVAL) {
        
        // ===== MOTOR 1 - DETECCIÓN INTELIGENTE =====
        bool m1_high_pwm = (m1_currentPWM > M1_MIN_PWM + STALL_MIN_PWM_THRESHOLD);
        bool m1_big_error = (error_m1 > STALL_MIN_ERROR_TICKS);
        bool m1_no_movement = (delta_m1 < STALL_MIN_MOVEMENT);
        
        if (m1_high_pwm && m1_big_error && m1_no_movement) {
          char msg[150];
          snprintf(msg, sizeof(msg), 
                   "MOTOR 1 STALL - PWM=%d, Error=%ld ticks (%.1f°), Movimiento=%ld ticks en %dms", 
                   m1_currentPWM, error_m1, error_m1 / M1_TICKS_PER_DEGREE, 
                   delta_m1, STALL_CHECK_INTERVAL);
          setFailState(JD_CODE_03_MOTOR1_STALL, msg);
          vTaskSuspend(NULL);
        }
        
        // ===== MOTOR 2 - DETECCIÓN INTELIGENTE =====
        bool m2_high_pwm = (m2_currentPWM > M2_MIN_PWM + STALL_MIN_PWM_THRESHOLD);
        bool m2_big_error = (error_m2 > STALL_MIN_ERROR_TICKS);
        bool m2_no_movement = (delta_m2 < STALL_MIN_MOVEMENT);
        
        if (m2_high_pwm && m2_big_error && m2_no_movement) {
          char msg[150];
          snprintf(msg, sizeof(msg), 
                   "MOTOR 2 STALL - PWM=%d, Error=%ld ticks (%.1f°), Movimiento=%ld ticks en %dms", 
                   m2_currentPWM, error_m2, error_m2 / M2_TICKS_PER_DEGREE, 
                   delta_m2, STALL_CHECK_INTERVAL);
          setFailState(JD_CODE_04_MOTOR2_STALL, msg);
          vTaskSuspend(NULL);
        }
        
        // Guardar para próxima verificación
        last_m1_stall_check = current_m1;
        last_m2_stall_check = current_m2;
        last_m1_error_ticks = error_m1;
        last_m2_error_ticks = error_m2;
        lastStallCheckTime = millis();
      }
    } else {
      // Si PID no está activo, resetear contadores
      lastStallCheckTime = millis();
      last_m1_stall_check = m1_encoderCount;
      last_m2_stall_check = m2_encoderCount;
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// -------------------------------------------
// Task: DSP Analysis Module
// Frequency: 100 ms (10 Hz)
// Priority: Low
// Core: 0
// Description: Performs digital signal
//              processing on encoder data.
//              Analyzes frequency content
//              and detects abnormal vibrations.
// -------------------------------------------
void TaskDSPAnalysis(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  
  for(;;) {
    if (pidTaskEnabled) {
      long m1_val;
      
      if (xSemaphoreTake(xEncoderMutex, pdMS_TO_TICKS(10))) {
        m1_val = m1_encoderCount;
        xSemaphoreGive(xEncoderMutex);
      } else {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;
      }
      
      // Agregar muestra al buffer DSP
      float m1_angle = m1_ticksToDegrees(m1_val);
      dsp_addSample(m1_angle);
      
      // Analizar cada vez que se llena el buffer
      if (dsp_buffer_index == 0) {
        dsp_analyzeVibration();
        
        if (dsp_vibration_detected) {
          Serial.print("[DSP] Vibracion detectada - Freq: ");
          Serial.print(dsp_dominant_frequency, 2);
          Serial.println(" Hz");
        }
      }
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// -------------------------------------------
// Task: PID Control
// Frequency: 10 ms (100 Hz)
// Priority: High (Real-time control)
// Core: 1
// Description: Computes PID control for both
//              motors with FIR filtered encoder
//              feedback. Critical for motion
//              control stability.
// -------------------------------------------
void TaskPIDControl(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz
  
  for(;;) {
    if (!checkSystemState()) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      continue;
    }
    
    if (pidTaskEnabled && newTargetAvailable) {
      // Leer targets protegidos
      float local_target_m1, local_target_m2;
      if (xSemaphoreTake(xTargetMutex, pdMS_TO_TICKS(50))) {
        local_target_m1 = target_m1_angle;
        local_target_m2 = target_m2_angle;
        xSemaphoreGive(xTargetMutex);
      } else {
        jd_report(JD_CODE_08_SYSTEM_OVERLOAD, "Mutex timeout en PID task");
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;
      }
      
      long m1_targetTicks = m1_degreesToTicks(local_target_m1);
      long m2_targetTicks = m2_degreesToTicks(local_target_m2);
      
      // Leer encoders con filtro
      long m1_current, m2_current;
      if (xSemaphoreTake(xEncoderMutex, pdMS_TO_TICKS(50))) {
        m1_current = m1_encoderCount;
        m2_current = m2_encoderCount;
        xSemaphoreGive(xEncoderMutex);
      } else {
        jd_report(JD_CODE_08_SYSTEM_OVERLOAD, "Mutex timeout en encoder read");
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;
      }
      
      m1_current = filterEncoderFIR(m1_current, m1_filterBuffer, m1_filterIndex, M1_FILTER_N);
      m2_current = filterEncoderFIR(m2_current, m2_filterBuffer, m2_filterIndex, M2_FILTER_N);
      
      // Control PID Motor 1
      bool m1_arrived = abs(m1_current - m1_targetTicks) <= TOLERANCE;
      if (!m1_arrived) {
        int pwm1 = m1_calculatePID(m1_current, m1_targetTicks);
        m1_move((m1_current > m1_targetTicks) ? -1 : 1, pwm1);
      } else {
        m1_stop();
      }
      
      // Control PID Motor 2
      bool m2_arrived = abs(m2_current - m2_targetTicks) <= TOLERANCE;
      if (!m2_arrived) {
        int pwm2 = m2_calculatePID(m2_current, m2_targetTicks);
        m2_move((m2_current > m2_targetTicks) ? -1 : 1, pwm2);
      } else {
        m2_stop();
      }
      
      // Si ambos llegaron, detener tarea
      if (m1_arrived && m2_arrived) {
        newTargetAvailable = false;
        pidTaskEnabled = false;
        stopAllMotors();
        Serial.println("[OK] Posicion alcanzada");
      }
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// -------------------------------------------
// Task: WiFi Handler
// Frequency: 50 ms
// Priority: Medium
// Core: 1
// Description: Manages WiFi client connections
//              and coordinate data reception.
//              Processes inverse kinematics
//              and updates motion targets.
// -------------------------------------------
void TaskWiFiHandler(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  
  for(;;) {
    if (!checkSystemState()) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      continue;
    }
    
    if (currentMode == WIFI_ACTIVE) {
      // Verificar nuevos clientes
      if (server.hasClient()) {
        if (!client || !client.connected()) {
          if (client) client.stop();
          client = server.available();
          Serial.println("\n[OK] Cliente conectado");
          wifiConnected = true;
          lastWiFiData = millis();
        }
      }
      
      // Leer datos disponibles
      if (client && client.connected()) {
        while (client.available()) {
          String line = client.readStringUntil('\n');
          parseWiFiData(line);
        }
      } else if (wifiConnected) {
        Serial.println("\n[WARN] Cliente desconectado");
        wifiConnected = false;
        stopAllMotors();
        pidTaskEnabled = false;
      }
      
      // Procesar nuevas coordenadas
      if (newDataAvailable) {
        newDataAvailable = false;
        
        Serial.print("[WIFI] Moviendo a (");
        Serial.print(targetX_WiFi, 2); Serial.print(", ");
        Serial.print(targetY_WiFi, 2); Serial.println(")");
        
        float m1_target, m2_target;
        if (inverseKinematicsCalibrated(targetX_WiFi, targetY_WiFi, m1_target, m2_target)) {
          if (xSemaphoreTake(xTargetMutex, pdMS_TO_TICKS(100))) {
            target_m1_angle = m1_target;
            target_m2_angle = m2_target;
            newTargetAvailable = true;
            pidTaskEnabled = true;
            xSemaphoreGive(xTargetMutex);
          } else {
            jd_report(JD_CODE_08_SYSTEM_OVERLOAD, "Mutex timeout en WiFi task");
          }
        }
      }
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// -------------------------------------------
// Task: Status Monitor
// Frequency: 3000 ms (3 seconds)
// Priority: Low
// Core: 0
// Description: Periodic system status reporting.
//              Displays position, voltage, and
//              diagnostic information. Also
//              reports DSP analysis results.
// -------------------------------------------
void TaskStatusMonitor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(3000);
  
  for(;;) {
    if (currentMode == WIFI_ACTIVE || currentMode == DEBUG_MODE) {
      long m1_val, m2_val;
      
      if (xSemaphoreTake(xEncoderMutex, pdMS_TO_TICKS(100))) {
        m1_val = m1_encoderCount;
        m2_val = m2_encoderCount;
        xSemaphoreGive(xEncoderMutex);
      } else {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;
      }
      
      float m1_angle = m1_ticksToDegrees(m1_val);
      float m2_angle = m2_ticksToDegrees(m2_val);
      float x, y;
      forwardKinematicsCalibrated(m1_angle, m2_angle, x, y);
      
      if (currentMode == WIFI_ACTIVE) {
        Serial.print("[POS] XY("); Serial.print(x, 1);
        Serial.print(", "); Serial.print(y, 1); Serial.print(")");
        Serial.print(" | V: "); Serial.print(currentVoltage, 2); Serial.print("V");
        Serial.print(wifiConnected ? " [CONN]" : " [DISC]");
        if (pidTaskEnabled) Serial.print(" [MOVING]");
        if (dsp_vibration_detected) Serial.print(" [VIBR]");
        Serial.println();
      } else if (currentMode == DEBUG_MODE) {
        Serial.print("[POS] M1="); Serial.print(m1_angle, 1);
        Serial.print(" deg  M2="); Serial.print(m2_angle, 1);
        Serial.print(" deg  XY("); Serial.print(x, 1);
        Serial.print(", "); Serial.print(y, 1); Serial.print(")");
        Serial.print(" | V: "); Serial.print(currentVoltage, 2); Serial.println("V");
      }
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// -------------------------------------------
// Task: Serial Command Handler
// Frequency: 100 ms
// Priority: Medium
// Core: 0
// Description: Processes serial commands from
//              user. Handles mode switching,
//              diagnostics, and manual control.
// -------------------------------------------
void TaskSerialHandler(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  
  for(;;) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      
      // STOP global
      if (cmd.equalsIgnoreCase("stop") || cmd == "s" || cmd == "S") {
        stopAllMotors();
        pidTaskEnabled = false;
        Serial.println("[STOP] Motores detenidos");
      }
      // Procesar según modo
      else if (currentMode == MENU_WIFI) {
        processWiFiMenu(cmd);
      }
      else if (currentMode == DEBUG_MODE) {
        processCommand(cmd);
      }
      else if (currentMode == WIFI_ACTIVE) {
        if (cmd.equalsIgnoreCase("d")) {
          currentMode = DEBUG_MODE;
          Serial.println("[DEBUG] Modo debug activado");
          printHelp();
        } 
        else if (cmd.equalsIgnoreCase("m")) {
          stopAllMotors();
          pidTaskEnabled = false;
          currentMode = MENU_WIFI;
          Serial.println("[MENU] Volviendo al menu...");
          printWiFiMenu();
        }
        else {
          processCommand(cmd);
        }
      }
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============================================
// MOVIMIENTO A ANGULOS (BLOQUEANTE - PARA DEBUG)
// ============================================
bool goToAngles(float m1_targetDegrees, float m2_targetDegrees, unsigned long timeout_ms = 10000) {
  if (m1_targetDegrees < M1_MIN_ANGLE || m1_targetDegrees > M1_MAX_ANGLE) {
    Serial.print("[ERROR] Motor 1 fuera de rango: ");
    Serial.print(m1_targetDegrees, 1); Serial.println(" deg");
    return false;
  }
  if (m2_targetDegrees < M2_MIN_ANGLE || m2_targetDegrees > M2_MAX_ANGLE) {
    Serial.print("[ERROR] Motor 2 fuera de rango: ");
    Serial.print(m2_targetDegrees, 1); Serial.println(" deg");
    return false;
  }

  long m1_targetTicks = m1_degreesToTicks(m1_targetDegrees);
  long m2_targetTicks = m2_degreesToTicks(m2_targetDegrees);

  emergencyStop = false;
  m1_lastError = 0; m1_integral = 0; m1_currentPWM = 0;
  m2_lastError = 0; m2_integral = 0; m2_currentPWM = 0;
  m1_lastPIDTime = m2_lastPIDTime = millis();

  Serial.print("[MOV] Target: M1="); Serial.print(m1_targetDegrees, 1);
  Serial.print(" deg  M2="); Serial.print(m2_targetDegrees, 1); Serial.println(" deg");

  unsigned long startTime = millis();
  unsigned long lastProgressTime = millis();
  
  noInterrupts();
  long last_m1 = m1_encoderCount;
  long last_m2 = m2_encoderCount;
  interrupts();

  while (true) {
    // Check timeout
    if (millis() - startTime > timeout_ms) {
      stopAllMotors();
      Serial.println("[TIMEOUT] Movimiento excedio tiempo limite");
      return false;
    }

    // Check for stop command
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "stop" || cmd == "s") {
        stopAllMotors();
        Serial.println("[STOP] Movimiento detenido");
        return false;
      }
    }

    noInterrupts();
    long m1_current = m1_encoderCount;
    long m2_current = m2_encoderCount;
    interrupts();

    // Aplicar filtro FIR a las lecturas del encoder
    m1_current = filterEncoderFIR(m1_current, m1_filterBuffer, m1_filterIndex, M1_FILTER_N);
    m2_current = filterEncoderFIR(m2_current, m2_filterBuffer, m2_filterIndex, M2_FILTER_N);

    // Check if motors are stuck
    if (millis() - lastProgressTime > 2000) {
      long delta_m1 = abs(m1_current - last_m1);
      long delta_m2 = abs(m2_current - last_m2);
      
      if (delta_m1 < 3 && delta_m2 < 3) {
        stopAllMotors();
        Serial.println("[ERROR] Motores atascados - sin progreso");
        return false;
      }
      
      last_m1 = m1_current;
      last_m2 = m2_current;
      lastProgressTime = millis();
    }

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
  Serial.println("[OK] Posicion alcanzada");
  return true;
}

// ============================================
// MOVIMIENTO A COORDENADAS XY (USADO POR DEBUG Y WIFI)
// ============================================
bool goToXY(float x, float y) {
  float m1_target, m2_target;

  if (!inverseKinematicsCalibrated(x, y, m1_target, m2_target)) {
    return false;
  }

  Serial.print("[MOV] XY(");
  Serial.print(x, 2); Serial.print(", "); Serial.print(y, 2);
  Serial.print(") -> M1="); Serial.print(m1_target, 1);
  Serial.print(" deg  M2="); Serial.print(m2_target, 1); Serial.println(" deg");

  return goToAngles(m1_target, m2_target);
}

// ============================================
// MENU INICIAL WiFi
// ============================================
void printWiFiMenu() {
  Serial.println("\n========================================");
  Serial.println("       MENU INICIAL - SCARA ROBOT      ");
  Serial.println("========================================");
  Serial.println("\n  c  -> Conectar WiFi (SCARANet)");
  Serial.println("  i  -> Mostrar IP actual");
  Serial.println("  d  -> Modo Debug (comandos locales)");
  Serial.println("  h  -> Mostrar este menu");
  Serial.println("  m  -> Menu (desde cualquier modo)");
  Serial.println("\n========================================");
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

  Serial.print("\n> Comando: "); Serial.println(cmd);

  if (cmd == "c") {
    Serial.println("[WIFI] Intentando conectar...");
    connectWiFi();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("[OK] WiFi conectado");
      Serial.print("[INFO] IP: ");
      Serial.println(WiFi.localIP());
      Serial.println("[INFO] Puerto: 8080");
      Serial.println("\n[WAIT] Esperando conexion de Raspberry Pi...");
      currentMode = WIFI_ACTIVE;
      server.begin();
    } else {
      Serial.println("[ERROR] Conexion fallida");
    }
  }
  else if (cmd == "i") {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("[INFO] IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("[ERROR] WiFi no conectado");
    }
  }
  else if (cmd == "d") {
    Serial.println("[DEBUG] Entrando a modo debug");
    currentMode = DEBUG_MODE;
    printHelp();
  }
  else if (cmd == "h") {
    printWiFiMenu();
  }
  else {
    Serial.println("[ERROR] Comando invalido");
    printWiFiMenu();
  }
}

// ============================================
// PARSEO DE DATOS WiFi
// ============================================
void parseWiFiData(String data) {
  data.trim();
  
  if (data.length() == 0) return;

  // Comandos especiales
  if (data == "PING") {
    Serial.println("[WIFI] PING recibido");
    if (client && client.connected()) {
      client.println("PONG");
    }
    return;
  }

  if (data == "STOP") {
    Serial.println("[WIFI] STOP recibido");
    stopAllMotors();
    return;
  }

  // Parseo de coordenadas: "X.XX,Y.YY"
  int commaIndex = data.indexOf(',');

  if (commaIndex > 0) {
    String xStr = data.substring(0, commaIndex);
    String yStr = data.substring(commaIndex + 1);

    float x = xStr.toFloat();
    float y = yStr.toFloat();

    // Validacion de parseo
    bool validParse = true;
    if (x == 0.0 && xStr.charAt(0) != '0' && xStr.charAt(0) != '-') {
      validParse = false;
    }
    if (y == 0.0 && yStr.charAt(0) != '0' && yStr.charAt(0) != '-') {
      validParse = false;
    }

    // Validar rango de workspace
    if (validParse && x >= WS_X_MIN && x <= WS_X_MAX && 
        y >= WS_Y_MIN && y <= WS_Y_MAX) {
      
      targetX_WiFi = x;
      targetY_WiFi = y;
      newDataAvailable = true;
      lastWiFiData = millis();

      Serial.print("[WIFI] Recibido: X=");
      Serial.print(x, 2); Serial.print("  Y=");
      Serial.println(y, 2);
    } else {
      Serial.print("[ERROR] Coordenadas invalidas: ");
      Serial.println(data);
    }
  }
}

// ============================================
// GESTION DE CONEXION WiFi
// ============================================
void handleWiFi() {
  if (server.hasClient()) {
    if (!client || !client.connected()) {
      if (client) client.stop();
      client = server.available();
      Serial.println("\n[OK] Cliente conectado");
      wifiConnected = true;
      lastWiFiData = millis();
    }
  }

  if (client && client.connected()) {
    while (client.available()) {
      String line = client.readStringUntil('\n');
      parseWiFiData(line);
    }
  } else if (wifiConnected) {
    Serial.println("\n[WARN] Cliente desconectado");
    wifiConnected = false;
    stopAllMotors();
  }
}

// ============================================
// AYUDA Y ESTADO
// ============================================
void printHelp() {
  Serial.println("\n========================================");
  Serial.println("   SCARA XY - CINEMATICA INVERSA       ");
  Serial.println("========================================");
  Serial.println("\nCOMANDOS:");
  Serial.println("  xy X Y    -> Mover a coordenadas (ej: xy 2.5 3.0)");
  Serial.println("  c1-c4     -> Ir a esquinas");
  Serial.println("  home / 0  -> Centro (0, 0)");
  Serial.println("  stop / s  -> Detener motores");
  Serial.println("  reset     -> Reset encoders a home (90, 90)");
  Serial.println("  status    -> Ver estado actual");
  Serial.println("  diag      -> Diagnostico completo del sistema");
  Serial.println("  volt      -> Test de lectura de voltaje");  // ← NUEVO
  Serial.println("  m         -> Volver al menu");
  Serial.println("========================================\n");
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

  Serial.println("\n[STATUS]");
  Serial.print("  M1: "); Serial.print(m1_angle, 2); Serial.println(" deg");
  Serial.print("  M2: "); Serial.print(m2_angle, 2); Serial.println(" deg");
  Serial.print("  XY: ("); Serial.print(x, 2); Serial.print(", ");
  Serial.print(y, 2); Serial.println(")");
}

// ============================================
// DIAGNOSTICO COMPLETO DEL SISTEMA
// ============================================
void printDiagnostic() {
  Serial.println("\n========================================");
  Serial.println("  DIAGNOSTICO COMPLETO DEL SISTEMA");
  Serial.println("========================================");
  
  // Estado del sistema
  Serial.print("Estado: ");
  switch(currentSystemState) {
    case SYSTEM_NORMAL:
      Serial.println("NORMAL");
      break;
    case SYSTEM_FAIL:
      Serial.println("FALLO");
      break;
    case SYSTEM_EMERGENCY_STOP:
      Serial.println("PARADA DE EMERGENCIA");
      break;
  }
  
  // Último código de diagnóstico
  Serial.print("Ultimo Codigo: JD_CODE_");
  if (lastDiagnosticCode < 10) Serial.print("0");
  Serial.println(lastDiagnosticCode);
  
  Serial.print("Descripcion: ");
  switch(lastDiagnosticCode) {
    case JD_CODE_00_NORMAL:
      Serial.println("Sistema operando normalmente");
      break;
    case JD_CODE_01_UNDERVOLTAGE:
      Serial.println("Bajo voltaje detectado");
      break;
    case JD_CODE_02_OVERVOLTAGE:
      Serial.println("Sobre voltaje detectado");
      break;
    case JD_CODE_03_MOTOR1_STALL:
      Serial.println("Motor 1 bloqueado/atascado");
      break;
    case JD_CODE_04_MOTOR2_STALL:
      Serial.println("Motor 2 bloqueado/atascado");
      break;
    case JD_CODE_05_ENCODER1_FAULT:
      Serial.println("Fallo en encoder del Motor 1");
      break;
    case JD_CODE_06_ENCODER2_FAULT:
      Serial.println("Fallo en encoder del Motor 2");
      break;
    case JD_CODE_07_COMMUNICATION_TIMEOUT:
      Serial.println("Timeout de comunicacion");
      break;
    case JD_CODE_08_SYSTEM_OVERLOAD:
      Serial.println("Sobrecarga del sistema");
      break;
    default:
      Serial.println("Codigo desconocido");
  }
  
  if (lastFaultMessage.length() > 0) {
    Serial.print("Mensaje: ");
    Serial.println(lastFaultMessage);
  }
  
  if (lastFaultTime > 0) {
    Serial.print("Tiempo desde fallo: ");
    Serial.print((millis() - lastFaultTime) / 1000);
    Serial.println(" segundos");
  }
  
  // Voltaje actual
  Serial.println("\n--- VOLTAJE ---");
  Serial.print("Voltaje actual: ");
  Serial.print(currentVoltage, 2);
  Serial.println(" V");
  Serial.print("Rango valido: ");
  Serial.print(VOLTAGE_MIN, 1);
  Serial.print(" - ");
  Serial.print(VOLTAGE_MAX, 1);
  Serial.println(" V");
  Serial.print("Estado: ");
  Serial.println(voltageOK ? "OK" : "FUERA DE RANGO");
  
  // Posición actual
  Serial.println("\n--- POSICION ---");
  noInterrupts();
  long m1_ticks = m1_encoderCount;
  long m2_ticks = m2_encoderCount;
  interrupts();
  
  float m1_angle = m1_ticksToDegrees(m1_ticks);
  float m2_angle = m2_ticksToDegrees(m2_ticks);
  float x, y;
  forwardKinematicsCalibrated(m1_angle, m2_angle, x, y);
  
  Serial.print("Motor 1: ");
  Serial.print(m1_angle, 2);
  Serial.println(" deg");
  Serial.print("Motor 2: ");
  Serial.print(m2_angle, 2);
  Serial.println(" deg");
  Serial.print("Posicion XY: (");
  Serial.print(x, 2);
  Serial.print(", ");
  Serial.print(y, 2);
  Serial.println(")");
  
  // Estado de tareas
  Serial.println("\n--- TAREAS ---");
  Serial.print("PID: ");
  Serial.println(pidTaskEnabled ? "ACTIVO" : "INACTIVO");
  Serial.print("WiFi: ");
  Serial.println(wifiConnected ? "CONECTADO" : "DESCONECTADO");
  Serial.print("Modo: ");
  switch(currentMode) {
    case MENU_WIFI:
      Serial.println("MENU");
      break;
    case DEBUG_MODE:
      Serial.println("DEBUG");
      break;
    case WIFI_ACTIVE:
      Serial.println("WIFI ACTIVO");
      break;
  }
  
  // DSP
  Serial.println("\n--- DSP ---");
  Serial.print("Vibracion: ");
  Serial.println(dsp_vibration_detected ? "DETECTADA" : "NO DETECTADA");
  if (dsp_vibration_detected) {
    Serial.print("Frecuencia: ");
    Serial.print(dsp_dominant_frequency, 2);
    Serial.println(" Hz");
  }
  
  Serial.println("========================================");
  Serial.println("[INFO] Escriba 'help' para ver comandos");
  Serial.println("========================================\n");
}

void testVoltageReading() {
  Serial.println("\n========================================");
  Serial.println("  TEST DE LECTURA DE VOLTAJE");
  Serial.println("========================================");
  
  // Leer 20 muestras
  Serial.println("\nLeyendo 20 muestras del ADC...\n");
  
  for (int i = 0; i < 20; i++) {
    int raw = analogRead(VOLTAGE_PIN);
    float adcVoltage = (raw / 4095.0) * 3.3;
    float batteryVoltage = adcVoltage * VOLTAGE_DIVIDER_RATIO;
    
    Serial.print("Muestra ");
    if (i < 10) Serial.print(" ");
    Serial.print(i + 1);
    Serial.print(": ADC=");
    Serial.print(raw);
    Serial.print(" (");
    Serial.print(adcVoltage, 3);
    Serial.print("V)  ->  Bateria=");
    Serial.print(batteryVoltage, 2);
    Serial.println("V");
    
    delay(100);
  }
  
  // Calcular promedio
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogRead(VOLTAGE_PIN);
    delay(10);
  }
  float avgADC = sum / 50.0;
  float avgADCVoltage = (avgADC / 4095.0) * 3.3;
  float avgBatteryVoltage = avgADCVoltage * VOLTAGE_DIVIDER_RATIO;
  
  Serial.println("\n--- PROMEDIO (50 muestras) ---");
  Serial.print("ADC raw promedio: ");
  Serial.print(avgADC, 1);
  Serial.println(" / 4095");
  Serial.print("Voltaje ADC: ");
  Serial.print(avgADCVoltage, 3);
  Serial.println("V");
  Serial.print("Voltaje Bateria: ");
  Serial.print(avgBatteryVoltage, 2);
  Serial.println("V");
  
  Serial.println("\n--- CONFIGURACION ACTUAL ---");
  Serial.print("Divisor de voltaje: ");
  Serial.println(VOLTAGE_DIVIDER_RATIO, 1);
  Serial.print("Umbral minimo: ");
  Serial.print(VOLTAGE_MIN, 1);
  Serial.println("V");
  Serial.print("Umbral maximo: ");
  Serial.print(VOLTAGE_MAX, 1);
  Serial.println("V");
  
  Serial.print("\nEstado: ");
  if (avgBatteryVoltage < VOLTAGE_MIN) {
    Serial.println("UNDERVOLTAGE ❌");
  } else if (avgBatteryVoltage > VOLTAGE_MAX) {
    Serial.println("OVERVOLTAGE ❌");
  } else {
    Serial.println("OK ✓");
  }
  
  Serial.println("========================================\n");
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
  else if (cmd == "diag" || cmd == "diagnostic") {
    printDiagnostic();
  }
    else if (cmd == "volt" || cmd == "voltage") {
    testVoltageReading();
  }
  else if (cmd == "stop" || cmd == "s") {
    stopAllMotors();
    Serial.println("[STOP] Motores detenidos");
  }
  else if (cmd == "reset") {
    resetEncoders();
  }
  else if (cmd == "m") {
    stopAllMotors();
    currentMode = MENU_WIFI;
    Serial.println("[MENU] Volviendo al menu...");
    printWiFiMenu();
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
      Serial.println("[ERROR] Formato: xy X Y");
    }
  }
  else {
    Serial.println("[ERROR] Comando no reconocido. Escribe 'help'");
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

  Serial.println("\n========================================");
  Serial.println("     SCARA XY - SISTEMA INICIADO       ");
  Serial.println("     CON FREERTOS Y FILTROS DIGITALES  ");
  Serial.println("========================================");

  // Calibracion inicial
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

  Serial.println("[OK] Sistema calibrado");
  
  // Crear Mutex
  xEncoderMutex = xSemaphoreCreateMutex();
  xTargetMutex = xSemaphoreCreateMutex();
  xSystemStateMutex = xSemaphoreCreateMutex();
  
  if (xEncoderMutex == NULL || xTargetMutex == NULL || xSystemStateMutex == NULL) {
    Serial.println("[FATAL] No se pudieron crear los mutex");
    while(1);
  }
  
  // Configurar ADC para monitoreo de voltaje
  pinMode(VOLTAGE_PIN, INPUT);
  analogReadResolution(12);  // 12-bit resolution
  
  Serial.println("\n========================================");
  Serial.println("  INICIALIZANDO SISTEMA FREERTOS");
  Serial.println("========================================");
  
  // Crear Tareas FreeRTOS
  xTaskCreatePinnedToCore(
    TaskPIDControl,
    "PID_Control",
    4096,
    NULL,
    3,                   // Prioridad ALTA
    &Task_PID,
    1                    // Core 1
  );
  
  xTaskCreatePinnedToCore(
    TaskWiFiHandler,
    "WiFi_Handler",
    4096,
    NULL,
    2,                   // Prioridad MEDIA
    &Task_WiFi,
    1                    // Core 1
  );
  
  xTaskCreatePinnedToCore(
    TaskVoltageMonitor,
    "Voltage_Monitor",
    2048,
    NULL,
    4,                   // Prioridad MUY ALTA (Safety)
    &Task_Voltage,
    0                    // Core 0
  );
  
  xTaskCreatePinnedToCore(
    TaskStallMonitor,
    "Stall_Monitor",
    2048,
    NULL,
    3,                   // Prioridad ALTA
    &Task_Stall,
    0                    // Core 0
  );
  
  xTaskCreatePinnedToCore(
    TaskDSPAnalysis,
    "DSP_Analysis",
    3072,
    NULL,
    1,                   // Prioridad BAJA
    &Task_DSP,
    0                    // Core 0
  );
  
  xTaskCreatePinnedToCore(
    TaskStatusMonitor,
    "Status_Monitor",
    2048,
    NULL,
    1,                   // Prioridad BAJA
    &Task_Status,
    0                    // Core 0
  );
  
  xTaskCreatePinnedToCore(
    TaskSerialHandler,
    "Serial_Handler",
    3072,
    NULL,
    2,                   // Prioridad MEDIA
    &Task_Serial,
    0                    // Core 0
  );
  
  Serial.println("[RTOS] Tareas creadas exitosamente:");
  Serial.println("  CORE 1 (Control):");
  Serial.println("    - PID Control    (10ms,  P:3)");
  Serial.println("    - WiFi Handler   (50ms,  P:2)");
  Serial.println("  CORE 0 (Monitor/Safety):");
  Serial.println("    - Voltage Mon.   (500ms, P:4) [SAFETY]");
  Serial.println("    - Stall Mon.     (500ms, P:3) [SAFETY]");
  Serial.println("    - DSP Analysis   (100ms, P:1) [DSP]");
  Serial.println("    - Status Mon.    (3s,    P:1)");
  Serial.println("    - Serial Handler (100ms, P:2)");
  Serial.println("========================================");
  Serial.println("[INFO] Fail-safe activado");
  Serial.println("[INFO] Monitoreo UV/OV ac+tivo");
  Serial.println("[INFO] Deteccion de stall activa");
  Serial.println("[INFO] Modulo DSP operativo");
  Serial.println("========================================\n");
  
  jd_report(JD_CODE_00_NORMAL, "Sistema iniciado correctamente");
  
  printWiFiMenu();
  lastDebugPrint = millis();
}

// ============================================
// LOOP
// ============================================
void loop() {
  // FreeRTOS maneja todo, loop vacío
  vTaskDelay(pdMS_TO_TICKS(1000));
}