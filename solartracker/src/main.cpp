#include <Arduino.h>

// --- Solar Tracker Setup ---
int left_sensor_pin = 36;
int right_sensor_pin = 39;
int motor_left_pwm = 5;
int motor_right_pwm = 4;

int lightThreshold = 500;
int maxPWM = 200;
int minPWM = 60;

// Previous sensor values
int prevLeft = 0;
int prevRight = 0;

// Filtering settings
const int samples = 10;

// Tolerance (important!)
int tolerance = 15;

// --- Stop motors ---
void stopMotors() {
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

// --- Read filtered sensor ---
int readFiltered(int pin) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2); // small delay for stability
  }
  return sum / samples;
}

void setup() {
  Serial.begin(115200);

  ledcSetup(2, 1000, 8);
  ledcSetup(3, 1000, 8);
  ledcAttachPin(motor_left_pwm, 2);
  ledcAttachPin(motor_right_pwm, 3);

  stopMotors();
}

void loop() {

  // --- Read filtered sensors ---
  int left_sensor = readFiltered(left_sensor_pin);
  int right_sensor = readFiltered(right_sensor_pin);
  int totalLight = (left_sensor + right_sensor) / 2;

  Serial.print("L: "); Serial.print(left_sensor);
  Serial.print(" R: "); Serial.print(right_sensor);

  // --- Night check ---
  if (totalLight < lightThreshold) {
    Serial.println(" --> Night mode");
    stopMotors();
    prevLeft = left_sensor;
    prevRight = right_sensor;
    delay(1000);
    return;
  }

  int error = left_sensor - right_sensor;

  // --- Decide direction ---
  if (error > 0) {
    // Move LEFT
    if (left_sensor >= prevLeft - tolerance) {
      int pwm = map(abs(error), 0, 2000, minPWM, maxPWM);
      pwm = constrain(pwm, minPWM, maxPWM);

      Serial.print(" --> Move LEFT PWM="); Serial.println(pwm);
      ledcWrite(2, pwm);
      ledcWrite(3, 0);
    } else {
      Serial.println(" --> Hold (Left peak)");
      stopMotors();
    }

  } else if (error < 0) {
    // Move RIGHT
    if (right_sensor >= prevRight - tolerance) {
      int pwm = map(abs(error), 0, 2000, minPWM, maxPWM);
      pwm = constrain(pwm, minPWM, maxPWM);

      Serial.print(" --> Move RIGHT PWM="); Serial.println(pwm);
      ledcWrite(2, 0);
      ledcWrite(3, pwm);
    } else {
      Serial.println(" --> Hold (Right peak)");
      stopMotors();
    }

  } else {
    Serial.println(" --> Hold (Balanced)");
    stopMotors();
  }

  // --- Update previous values ---
  prevLeft = left_sensor;
  prevRight = right_sensor;

  delay(300);
}