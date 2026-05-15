
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// --- WiFi Credentials ---
const char* ssid = "LUMBELL";
const char* password = "mtabayira";

// --- Firebase Configuration (UPDATED to history.json) ---
const String firebaseURL = "https://solar-power-monitor-4f1a0-default-rtdb.firebaseio.com/tracker/history.json";

// --- NTP Setup (For Real Time) ---
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// --- Solar Tracker Setup ---
const int left_sensor_pin = 36;
const int right_sensor_pin = 39;
const int motor_left_pwm = 23;
const int motor_right_pwm = 22;
const int voltage_sensor_pin = 35;
const int current_sensor_pin = 34;

const int lightThreshold = 50;
const int maxPWM = 200;
const int minPWM = 125;
const int adcMax = 4095;
const int pwmChannelLeft = 2;
const int pwmChannelRight = 3;
const int samples = 10;
const int deadband = 100;
const int tolerance = 80;

const float voltageCalibrationFactor = 11.0; 
const float currentSensorOffset = 2.5;       
const float currentSensorSensitivity = 0.185; 

int prevLeft = 0;
int prevRight = 0;
unsigned long lastFirebaseTime = 0;
float lastVoltage = 0.0;
float lastCurrent = 0.0;
const float voltageChangeThreshold = 0.5; 
const float currentChangeThreshold = 0.1;
const unsigned long heartbeatIntervalMs = 30000; 

bool wifiReady = false;
bool allowMotorControl = false;
bool initialized = false;
unsigned long lastWifiCheck = 0;
const unsigned long wifiReconnectIntervalMs = 10000;
bool timeSynced = false;

void stopMotors() {
  ledcWrite(pwmChannelLeft, 0);
  ledcWrite(pwmChannelRight, 0);
}

int readFiltered(int pin) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return sum / samples;
}

int computePwm(int errorValue) {
  int absError = abs(errorValue);
  if (absError <= deadband) return 0;
  int pwm = map(absError, deadband, adcMax, minPWM, maxPWM);
  return constrain(pwm, minPWM, maxPWM);
}

float readVoltage() {
  int raw = 0;
  for (int i = 0; i < 10; i++) { raw += analogRead(voltage_sensor_pin); delay(1); }
  raw /= 10;
  return (raw / 4095.0) * 3.3 * voltageCalibrationFactor;
}

float readCurrent() {
  int raw = 0;
  for (int i = 0; i < 10; i++) { raw += analogRead(current_sensor_pin); delay(1); }
  raw /= 10;
  float adcVoltage = (raw / 4095.0) * 3.3;
  return abs((adcVoltage - currentSensorOffset) / currentSensorSensitivity);
}

bool connectWiFi(unsigned long timeoutMs = 10000) {
  Serial.printf("📶 Connecting to WiFi '%s'...", ssid);
  WiFi.begin(ssid, password);
  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start >= timeoutMs) {
      Serial.println("\n⚠️ WiFi connect timeout");
      return false;
    }
    delay(500);
    Serial.print('.');
  }

  Serial.println("\n✅ Connected!");
  return true;
}

bool updateTime() {
  if (!wifiReady) {
    return false;
  }
  if (timeClient.update()) {
    timeSynced = true;
    return true;
  }
  return false;
}

String buildTrackerJSON(int leftSensor, int rightSensor, float voltage, float current, String action, int pwm) {
  unsigned long epochTime = 0;
  if (timeSynced) {
    epochTime = timeClient.getEpochTime();
  }
  if (epochTime == 0) {
    epochTime = millis() / 1000;
  }

  unsigned long long epochTimeMs = (unsigned long long)epochTime * 1000ULL;
  String json = "{";
  json += "\"timestamp\":" + String((double)epochTimeMs, 0) + ",";
  json += "\"left_sensor\":" + String(leftSensor) + ",";
  json += "\"right_sensor\":" + String(rightSensor) + ",";
  json += "\"voltage\":" + String(voltage, 2) + ",";
  json += "\"current\":" + String(current, 2) + ",";
  json += "\"action\":\"" + action + "\",";
  json += "\"pwm\":" + String(pwm);
  json += "}";
  return json;
}

void sendToFirebase(int leftSensor, int rightSensor, float voltage, float current, String action, int pwm) {
  if (WiFi.status() != WL_CONNECTED) {
    wifiReady = false;
    return;
  }

  if (!updateTime() && !timeSynced) {
    Serial.println("⚠️ NTP update failed, using fallback timestamp");
  }

  HTTPClient http;
  http.begin(firebaseURL);
  http.addHeader("Content-Type", "application/json");

  String jsonData = buildTrackerJSON(leftSensor, rightSensor, voltage, current, action, pwm);
  int httpResponseCode = http.POST(jsonData);

  if (httpResponseCode > 0) {
    Serial.printf("📡 History Logged (%d)\n", httpResponseCode);
  } else {
    Serial.printf("❌ Error: %s\n", http.errorToString(httpResponseCode).c_str());
  }
  http.end();
}

bool hasVoltageChanged(float currentVoltage) {
  if (abs(currentVoltage - lastVoltage) >= voltageChangeThreshold) {
    lastVoltage = currentVoltage;
    return true;
  }
  return false;
}

bool hasCurrentChanged(float currentVal) {
  if (abs(currentVal - lastCurrent) >= currentChangeThreshold) {
    lastCurrent = currentVal;
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  // Initialize all analog sensor pins with pulldown to prevent floating during reset/upload.
  pinMode(left_sensor_pin, INPUT_PULLDOWN);
  pinMode(right_sensor_pin, INPUT_PULLDOWN);
  pinMode(voltage_sensor_pin, INPUT_PULLDOWN);
  pinMode(current_sensor_pin, INPUT_PULLDOWN);
  // Add settling delay to ensure stable analog readings before WiFi connection.
  delay(100);
  // Connect to WiFi first, then allow motor logic only after WiFi is ready.
  wifiReady = connectWiFi(10000);
  timeClient.begin();
  if (wifiReady) {
    allowMotorControl = true;
    if (updateTime()) {
      Serial.println("✅ NTP time synchronized");
    } else {
      Serial.println("⚠️ NTP sync failed, using fallback time");
    }
  } else {
    Serial.println("⚠️ Running offline until WiFi becomes available");
  }

  // Required to attach PWM channels to motor pins before using ledcWrite().
  ledcSetup(pwmChannelLeft, 1000, 8);
  ledcSetup(pwmChannelRight, 1000, 8);
  ledcAttachPin(motor_left_pwm, pwmChannelLeft);
  ledcAttachPin(motor_right_pwm, pwmChannelRight);
  stopMotors();
}

void loop() {
  if (!wifiReady && millis() - lastWifiCheck >= wifiReconnectIntervalMs) {
    lastWifiCheck = millis();
    wifiReady = connectWiFi(10000);
    if (wifiReady) {
      allowMotorControl = true;
      if (updateTime()) {
        Serial.println("✅ NTP time synchronized after reconnect");
      } else {
        Serial.println("⚠️ NTP sync failed after reconnect, using fallback time");
      }
    }
  }

  if (!allowMotorControl) {
    // Hold motors stopped until WiFi connection is established.
    stopMotors();
    delay(300);
    return;
  }

  int left_sensor = readFiltered(left_sensor_pin);
  int right_sensor = readFiltered(right_sensor_pin);

  if (!initialized) {
    // Capture the first valid sensor baseline before making any movement decisions.
    prevLeft = left_sensor;
    prevRight = right_sensor;
    initialized = true;
    Serial.println("⚙️ Initial sensor baseline captured. Holding motors until next decision.");
    stopMotors();
    delay(300);
    return;
  }

  float voltage = readVoltage();
  float current = readCurrent();

  int error = left_sensor - right_sensor;
  int absError = abs(error);
  String action = "Hold";
  int activePwm = 0;

  if (absError > deadband) {
    int pwm = computePwm(error);
    activePwm = pwm;
    if (error > 0 && left_sensor >= prevLeft - tolerance) {
      action = "Move LEFT";
      Serial.print("L: "); Serial.print(left_sensor);
      Serial.print(" R: "); Serial.print(right_sensor);
      Serial.print(" | V: "); Serial.print(voltage, 1);
      Serial.print("V I: "); Serial.print(current, 2);
      Serial.print("A --> ");
      Serial.print(action);
      Serial.print(" PWM="); Serial.println(pwm);
      ledcWrite(pwmChannelLeft, pwm); ledcWrite(pwmChannelRight, 0);
    } else if (error < 0 && right_sensor >= prevRight - tolerance) {
      action = "Move RIGHT";
      Serial.print("L: "); Serial.print(left_sensor);
      Serial.print(" R: "); Serial.print(right_sensor);
      Serial.print(" | V: "); Serial.print(voltage, 1);
      Serial.print("V I: "); Serial.print(current, 2);
      Serial.print("A --> ");
      Serial.print(action);
      Serial.print(" PWM="); Serial.println(pwm);
      ledcWrite(pwmChannelLeft, 0); ledcWrite(pwmChannelRight, pwm);
    } else {
      action = (error > 0) ? "Hold (Left peak)" : "Hold (Right peak)";
      Serial.print("L: "); Serial.print(left_sensor);
      Serial.print(" R: "); Serial.print(right_sensor);
      Serial.print(" | V: "); Serial.print(voltage, 1);
      Serial.print("V I: "); Serial.print(current, 2);
      Serial.print("A --> ");
      Serial.println(action);
      stopMotors();
    }
  } else {
    Serial.print("L: "); Serial.print(left_sensor);
    Serial.print(" R: "); Serial.print(right_sensor);
    Serial.print(" | V: "); Serial.print(voltage, 1);
    Serial.print("V I: "); Serial.print(current, 2);
    Serial.print("A --> Hold (Balanced)");
    Serial.println();
    stopMotors();
  }

  if (hasVoltageChanged(voltage) || hasCurrentChanged(current)) {
    sendToFirebase(left_sensor, right_sensor, voltage, current, action, activePwm);
    lastFirebaseTime = millis();
  }

  prevLeft = left_sensor;
  prevRight = right_sensor;
  delay(300);
}
