/* Dual BLDC (Gokart) - ESP32
   - Single throttle (POT) on GPIO33
   - Single direction button on GPIO25 (toggle when throttle approx 0)
   - Two motors (A and B) each with 3 hall sensors (external pull-ups to 3.3V)
   - IR2110 gate drivers assumed: high-side = PWM (ledc), low-side = GPIO
   - 20 kHz PWM, 8-bit resolution
   - Soft-start, regen braking, hall debounce, fault detection
   - Motor A task pinned to core 0, Motor B to core 1
*/
#include <Arduino.h>
/* ------------------ PINOUT (change if you rewired) ------------------ */
/* Motor A (unchanged from your working mapping) */
#define A_AH 22
#define A_AL 21
#define A_BH 19
#define A_BL 18
#define A_CH 5
#define A_CL 23

#define A_HALL_A 32
#define A_HALL_B 35
#define A_HALL_C 34

/* Motor B (chosen previously) */
#define B_AH 14
#define B_AL 27
#define B_BH 26
#define B_BL 17
#define B_CH 16
#define B_CL 4

#define B_HALL_A 36
#define B_HALL_B 39
#define B_HALL_C 37

/* Controls */
#define POT_PIN 33      // single shared throttle (ADC 0..4095)
#define DIR_BUTTON 25   // one direction toggle for both motors

/* PWM / LEDC config */
#define PWM_FREQ 20000
#define PWM_RES 8        // 0..255
// channels
#define A_AH_CH 0
#define A_BH_CH 1
#define A_CH_CH 2
#define B_AH_CH 3
#define B_BH_CH 4
#define B_CH_CH 5

/* ------------------ TUNABLE PARAMETERS ------------------ */
const uint32_t HALL_DEBOUNCE_US = 200;   // ignore edges within this window (microseconds)
const uint32_t HALL_FAULT_TIMEOUT_MS = 250; // if no valid hall events within this and throttle>0 => fault
const uint16_t SOFT_START_MIN = 40;
const uint16_t SOFT_START_MAX = 180;
const uint16_t SOFT_STEP = 4;
const uint16_t SOFT_DELAY_MS = 25;
const uint16_t BRAKE_THRESHOLD = 120;    // pot < this (0..4095) => regen active
const uint8_t MAX_PWM = 255;
const uint8_t MIN_RUN_PWM = 30;          // below this assume stopped
const uint16_t DIRECTION_SAFE_POT = 200; // require pot < this to allow direction toggle
const uint16_t DEADTIME_US = 6;          // simple deadtime (allOff() then wait) - must match hardware

/* ------------------ Volatile state shared by ISR / tasks ------------------ */
volatile uint32_t lastHallTimeA = 0;     // micros of last hall ISR for motor A
volatile uint8_t lastHallStateA = 0;     // 3-bit state packed
volatile bool hallUpdatedA = false;

volatile uint32_t lastHallTimeB = 0;
volatile uint8_t lastHallStateB = 0;
volatile bool hallUpdatedB = false;

/* runtime flags */
volatile bool DIR = false; // false = forward, true = reverse (applies to both motors)

/* runtime pwm duty (0..255) */
volatile uint8_t pwmDuty = 0;

/* fault flags */
volatile bool faultA = false;
volatile bool faultB = false;

/* ------------------ Small helpers ------------------ */
inline void delayDeadtime() {
  // minimal delay to ensure MOSFETs are off
  // keep this short but non-zero
  delayMicroseconds(DEADTIME_US);
}

inline void allOffA() {
  ledcWrite(A_AH_CH, 0);
  ledcWrite(A_BH_CH, 0);
  ledcWrite(A_CH_CH, 0);
  digitalWrite(A_AL, LOW);
  digitalWrite(A_BL, LOW);
  digitalWrite(A_CL, LOW);
}
inline void allOffB() {
  ledcWrite(B_AH_CH, 0);
  ledcWrite(B_BH_CH, 0);
  ledcWrite(B_CH_CH, 0);
  digitalWrite(B_AL, LOW);
  digitalWrite(B_BL, LOW);
  digitalWrite(B_CL, LOW);
}

/* ledc-safe commutation: turns off all, small deadtime, then apply new step */
void commutateA(int step) {
  allOffA();
  delayDeadtime();
  switch(step) {
    case 0: ledcWrite(A_AH_CH, pwmDuty); digitalWrite(A_BL, HIGH); break;
    case 1: ledcWrite(A_AH_CH, pwmDuty); digitalWrite(A_CL, HIGH); break;
    case 2: ledcWrite(A_BH_CH, pwmDuty); digitalWrite(A_CL, HIGH); break;
    case 3: ledcWrite(A_BH_CH, pwmDuty); digitalWrite(A_AL, HIGH); break;
    case 4: ledcWrite(A_CH_CH, pwmDuty); digitalWrite(A_AL, HIGH); break;
    case 5: ledcWrite(A_CH_CH, pwmDuty); digitalWrite(A_BL, HIGH); break;
    default: allOffA(); break;
  }
}

void commutateB(int step) {
  allOffB();
  delayDeadtime();
  switch(step) {
    case 0: ledcWrite(B_AH_CH, pwmDuty); digitalWrite(B_BL, HIGH); break;
    case 1: ledcWrite(B_AH_CH, pwmDuty); digitalWrite(B_CL, HIGH); break;
    case 2: ledcWrite(B_BH_CH, pwmDuty); digitalWrite(B_CL, HIGH); break;
    case 3: ledcWrite(B_BH_CH, pwmDuty); digitalWrite(B_AL, HIGH); break;
    case 4: ledcWrite(B_CH_CH, pwmDuty); digitalWrite(B_AL, HIGH); break;
    case 5: ledcWrite(B_CH_CH, pwmDuty); digitalWrite(B_BL, HIGH); break;
    default: allOffB(); break;
  }
}

/* regen: turn on a low-side appropriate to absorb energy.
   NOTE: hardware must be able to handle current back to battery. */
void regenA(int step) {
  allOffA();
  delayDeadtime();
  switch(step) {
    case 0: digitalWrite(A_CL, HIGH); break;
    case 1: digitalWrite(A_BL, HIGH); break;
    case 2: digitalWrite(A_AL, HIGH); break;
    case 3: digitalWrite(A_CL, HIGH); break;
    case 4: digitalWrite(A_BL, HIGH); break;
    case 5: digitalWrite(A_AL, HIGH); break;
    default: allOffA(); break;
  }
}
void regenB(int step) {
  allOffB();
  delayDeadtime();
  switch(step) {
    case 0: digitalWrite(B_CL, HIGH); break;
    case 1: digitalWrite(B_BL, HIGH); break;
    case 2: digitalWrite(B_AL, HIGH); break;
    case 3: digitalWrite(B_CL, HIGH); break;
    case 4: digitalWrite(B_BL, HIGH); break;
    case 5: digitalWrite(B_AL, HIGH); break;
    default: allOffB(); break;
  }
}

/* hall decode (active HIGH, external pull-ups to 3.3V) */
int hallToStep(uint8_t ha, uint8_t hb, uint8_t hc) {
  uint8_t hall = (ha << 2) | (hb << 1) | hc;
  switch(hall) {
    case 0b101: return 0;
    case 0b100: return 1;
    case 0b110: return 2;
    case 0b010: return 3;
    case 0b011: return 4;
    case 0b001: return 5;
    default: return -1;
  }
}

/* ------------------ Hall ISR (very small, per motor) ------------------
   Approach: attach same ISR to all three hall pins of a motor. ISR reads
   the three pins atomically and sets hallUpdated flag + timestamp. */
void IRAM_ATTR isr_hallA() {
  uint32_t now = micros();
  // debounce in ISR: ignore if too close to previous event
  if ((now - lastHallTimeA) < HALL_DEBOUNCE_US) return;
  lastHallTimeA = now;
  uint8_t s = (digitalRead(A_HALL_A) << 2) | (digitalRead(A_HALL_B) << 1) | digitalRead(A_HALL_C);
  lastHallStateA = s;
  hallUpdatedA = true;
}

void IRAM_ATTR isr_hallB() {
  uint32_t now = micros();
  if ((now - lastHallTimeB) < HALL_DEBOUNCE_US) return;
  lastHallTimeB = now;
  uint8_t s = (digitalRead(B_HALL_A) << 2) | (digitalRead(B_HALL_B) << 1) | digitalRead(B_HALL_C);
  lastHallStateB = s;
  hallUpdatedB = true;
}

/* ------------------ Tasks ------------------ */

/* Motor A task: pinned to core 0 */
void taskMotorA(void *pvParameters) {
  (void) pvParameters;
  uint32_t lastCommutationMicros = 0;
  for (;;) {
    if (hallUpdatedA) {
      // atomically copy state
      noInterrupts();
      uint8_t state = lastHallStateA;
      hallUpdatedA = false;
      interrupts();

      // decode halls to step
      int step = hallToStep((state>>2)&1, (state>>1)&1, state&1);
      if (step < 0) {
        // unknown state => shut off
        allOffA();
        continue;
      }

      // reset fault timer
      lastCommutationMicros = millis();

      // if throttle low -> regen
      uint16_t pot = analogRead(POT_PIN);
      bool braking = (pot < BRAKE_THRESHOLD);

      if (braking) {
        regenA(step);
      } else {
        int cmd = DIR ? (5 - step) : step;
        commutateA(cmd);
      }
    } else {
      // If throttle > min and no hall updates for long -> fault
      uint16_t pot = analogRead(POT_PIN);
      if (pot > (BRAKE_THRESHOLD + 50)) {
        // check time since last hall change
        static uint32_t lastGood = 0;
        uint32_t nowms = millis();
        if (nowms - lastGood > HALL_FAULT_TIMEOUT_MS) {
          // double-check: if no hallUpdated recently and pot > threshold => fault
          faultA = true;
          allOffA();
        }
      } else {
        // clear condition
        faultA = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // light sleep 1 ms
  }
}

/* Motor B task: pinned to core 1 */
void taskMotorB(void *pvParameters) {
  (void) pvParameters;
  uint32_t lastCommutationMicros = 0;
  for (;;) {
    if (hallUpdatedB) {
      noInterrupts();
      uint8_t state = lastHallStateB;
      hallUpdatedB = false;
      interrupts();

      int step = hallToStep((state>>2)&1, (state>>1)&1, state&1);
      if (step < 0) {
        allOffB();
        continue;
      }
      lastCommutationMicros = millis();

      uint16_t pot = analogRead(POT_PIN);
      bool braking = (pot < BRAKE_THRESHOLD);

      if (braking) {
        regenB(step);
      } else {
        int cmd = DIR ? (5 - step) : step;
        commutateB(cmd);
      }
    } else {
      uint16_t pot = analogRead(POT_PIN);
      if (pot > (BRAKE_THRESHOLD + 50)) {
        static uint32_t lastGood = 0;
        uint32_t nowms = millis();
        if (nowms - lastGood > HALL_FAULT_TIMEOUT_MS) {
          faultB = true;
          allOffB();
        }
      } else {
        faultB = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* ------------------ Setup ------------------ */
void setup() {
  Serial.begin(115200);

  // configure low-side outputs
  pinMode(A_AL, OUTPUT); pinMode(A_BL, OUTPUT); pinMode(A_CL, OUTPUT);
  pinMode(B_AL, OUTPUT); pinMode(B_BL, OUTPUT); pinMode(B_CL, OUTPUT);

  // configure hall pins (external pull-ups to 3.3V)
  pinMode(A_HALL_A, INPUT); pinMode(A_HALL_B, INPUT); pinMode(A_HALL_C, INPUT);
  pinMode(B_HALL_A, INPUT); pinMode(B_HALL_B, INPUT); pinMode(B_HALL_C, INPUT);

  // attach hall ISRs (all three pins call same motor ISR)
  attachInterrupt(digitalPinToInterrupt(A_HALL_A), isr_hallA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A_HALL_B), isr_hallA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A_HALL_C), isr_hallA, CHANGE);

  attachInterrupt(digitalPinToInterrupt(B_HALL_A), isr_hallB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(B_HALL_B), isr_hallB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(B_HALL_C), isr_hallB, CHANGE);

  // direction button
  pinMode(DIR_BUTTON, INPUT_PULLUP);

  // throttle pot
  pinMode(POT_PIN, INPUT);

  // PWM channels
  ledcSetup(A_AH_CH, PWM_FREQ, PWM_RES);
  ledcSetup(A_BH_CH, PWM_FREQ, PWM_RES);
  ledcSetup(A_CH_CH, PWM_FREQ, PWM_RES);
  ledcSetup(B_AH_CH, PWM_FREQ, PWM_RES);
  ledcSetup(B_BH_CH, PWM_FREQ, PWM_RES);
  ledcSetup(B_CH_CH, PWM_FREQ, PWM_RES);

  ledcAttachPin(A_AH, A_AH_CH);
  ledcAttachPin(A_BH, A_BH_CH);
  ledcAttachPin(A_CH, A_CH_CH);
  ledcAttachPin(B_AH, B_AH_CH);
  ledcAttachPin(B_BH, B_BH_CH);
  ledcAttachPin(B_CH, B_CH_CH);

  allOffA();
  allOffB();

  // soft-start ramp
  for (uint16_t p = SOFT_START_MIN; p <= SOFT_START_MAX; p += SOFT_STEP) {
    pwmDuty = p;
    delay(SOFT_DELAY_MS);
  }

  // Create tasks pinned to cores
  xTaskCreatePinnedToCore(taskMotorA, "MotorA", 4096, NULL, 2, NULL, 0); // core 0
  xTaskCreatePinnedToCore(taskMotorB, "MotorB", 4096, NULL, 2, NULL, 1); // core 1

  Serial.println("Dual BLDC controller started");
}

/* ------------------ Main loop (supervisory) ------------------ */
void loop() {
  // read pot and set pwmDuty (smoothed)
  static uint16_t potFiltered = 0;
  int pot = analogRead(POT_PIN); // 0..4095
  // simple exponential smoothing
  potFiltered = (potFiltered * 7 + pot) >> 3;
  // map to PWM range
  uint8_t targetPwm = map(constrain((int)potFiltered, 0, 4095), 0, 4095, 0, MAX_PWM);

  // soft ramp toward target
  if (pwmDuty < targetPwm) pwmDuty++;
  else if (pwmDuty > targetPwm) pwmDuty--;

  // direction toggle only when pot near zero (safety)
  static uint32_t lastDirToggle = 0;
  static bool lastBtn = HIGH;
  bool btn = digitalRead(DIR_BUTTON);
  if (btn == LOW && lastBtn == HIGH) {
    // pressed
    if (potFiltered < DIRECTION_SAFE_POT) {
      // require long-press to avoid accidental flip
      uint32_t tstart = millis();
      while (digitalRead(DIR_BUTTON) == LOW) {
        if ((millis() - tstart) > 800) {
          DIR = !DIR;
          Serial.printf("Direction toggled -> %s\n", DIR ? "REVERSE" : "FORWARD");
          // ramp down & soft-start up to avoid shock
          pwmDuty = SOFT_START_MIN;
          for (uint16_t p = SOFT_START_MIN; p <= SOFT_START_MAX; p += SOFT_STEP) {
            pwmDuty = p;
            delay(SOFT_DELAY_MS);
          }
          break;
        }
        delay(10);
      }
    } else {
      Serial.println("Direction change blocked: reduce throttle to near zero first");
    }
  }
  lastBtn = btn;

  // fault handling: if either motor fault => shut both down for safety
  if (faultA || faultB) {
    allOffA();
    allOffB();
    Serial.println("FAULT: motor disabled");
    delay(200);
    // require manual reset via power-cycle or implement button reset (left as future)
    while (faultA || faultB) { delay(100); }
  }

  // tiny yield
  delay(5);
}
