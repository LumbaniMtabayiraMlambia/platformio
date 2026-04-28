#include <Arduino.h>
#include <algorithm>

// LED pins (use LEDs instead of MOSFETs)
#define HS_A 10
#define LS_A 9
#define HS_B 8
#define LS_B 7
#define HS_C 3
#define LS_C 5

// Hall sensor pins
#define HALL_A A0
#define HALL_B A1
#define HALL_C A2



// Controls & sensing
const uint8_t VREF_PIN = A3;           // pot setpoint (if available)
const uint8_t CURRENT_SENSE_PIN = A2;  // current-sense ADC
const uint8_t KILL_PIN = 8;            // kill switch active LOW

/* ---------- PARAMETERS ---------- */
const uint8_t MAX_PWM_VAL = 240;   // max PWM (0-255)
const uint8_t RAMP_STEP = 2;
const uint16_t RAMP_MS = 10;
const uint16_t DEADTIME_US = 6;    // small software deadtime
const int CURRENT_TRIP_ADC = 650;  // tune for your shunt amplifier
const uint32_t FAULT_RETRY_MS = 1500;

/* ---------- STATE ---------- */
volatile bool fault = false;
uint32_t fault_time = 0;
uint8_t pwm_target = 0;
uint8_t pwm_out = 0;
uint32_t last_ramp_ms = 0;

/* ---------- HELPERS ---------- */
void allOutputsOff() {
  // turn off gate inputs (assume LOW = MOSFET off)
  digitalWrite(AH_PIN, LOW);
  digitalWrite(AL_PIN, LOW);
  digitalWrite(BH_PIN, LOW);
  digitalWrite(BL_PIN, LOW);
  digitalWrite(CH_PIN, LOW);
  digitalWrite(CL_PIN, LOW);
  // ensure PWM off
  analogWrite(AL_PIN, 0);
  analogWrite(BL_PIN, 0);
  analogWrite(CL_PIN, 0);
}

int readMajority(uint8_t pin) {
  int a = digitalRead(pin);
  delayMicroseconds(40);
  int b = digitalRead(pin);
  delayMicroseconds(40);
  int c = digitalRead(pin);
  int s = a + b + c;
  return (s >= 2) ? 1 : 0;
}

// convert active-LOW hall raw to logic (active HIGH)
int hallLogicToStep(int h1raw, int h2raw, int h3raw) {
  int A = !h1raw;
  int B = !h2raw;
  int C = !h3raw;
  int pattern = (A<<2) | (B<<1) | C;
  switch(pattern) {
    case 0b101: return 0;
    case 0b100: return 1;
    case 0b110: return 2;
    case 0b010: return 3;
    case 0b011: return 4;
    case 0b001: return 5;
    default:    return -1;
  }
}

/* commutation: apply HS digital, LS PWM (pwmVal) */
void applyStep(int step, uint8_t pwmVal) {
  // turn everything off and wait deadtime
  allOutputsOff();
  delayMicroseconds(DEADTIME_US);

  switch(step) {
    case 0: // AH ON, BL sinks
      digitalWrite(AH_PIN, HIGH);
      analogWrite(BL_PIN, pwmVal);
      break;
    case 1: // AH ON, CL sinks
      digitalWrite(AH_PIN, HIGH);
      analogWrite(CL_PIN, pwmVal);
      break;
    case 2: // BH ON, CL sinks
      digitalWrite(BH_PIN, HIGH);
      analogWrite(CL_PIN, pwmVal);
      break;
    case 3: // BH ON, AL sinks
      digitalWrite(BH_PIN, HIGH);
      analogWrite(AL_PIN, pwmVal);
      break;
    case 4: // CH ON, AL sinks
      digitalWrite(CH_PIN, HIGH);
      analogWrite(AL_PIN, pwmVal);
      break;
    case 5: // CH ON, BL sinks
      digitalWrite(CH_PIN, HIGH);
      analogWrite(BL_PIN, pwmVal);
      break;
    default:
      allOutputsOff();
  }
}

void faultEnter(const char *msg) {
  fault = true;
  fault_time = millis();
  allOutputsOff();
  Serial.print("FAULT: ");
  Serial.println(msg);
}

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);

  pinMode(AH_PIN, OUTPUT);
  pinMode(BH_PIN, OUTPUT);
  pinMode(CH_PIN, OUTPUT);
  pinMode(AL_PIN, OUTPUT);
  pinMode(BL_PIN, OUTPUT);
  pinMode(CL_PIN, OUTPUT);

  pinMode(H1_PIN, INPUT_PULLUP);
  pinMode(H2_PIN, INPUT_PULLUP);
  pinMode(H3_PIN, INPUT_PULLUP);

  pinMode(KILL_PIN, INPUT_PULLUP);

  allOutputsOff();
  Serial.println("UNO_ESC: ready");
}

/* ---------- RAMP ---------- */
void rampStep() {
  uint32_t now = millis();
  if (now - last_ramp_ms < RAMP_MS) return;
  last_ramp_ms = now;
  if (pwm_out < pwm_target) {
    pwm_out = min<uint8_t>(pwm_out + RAMP_STEP, pwm_target);
  } else if (pwm_out > pwm_target) {
    pwm_out = max<int>(pwm_out - RAMP_STEP, pwm_target);
  }
}

bool checkOvercurrent() {
  int v = analogRead(CURRENT_SENSE_PIN);
  return (v >= CURRENT_TRIP_ADC);
}

/* ---------- MAIN LOOP ---------- */
void loop() {
  // kill prioritized
  if (digitalRead(KILL_PIN) == LOW) {
    faultEnter("KILL");
  }

  // read vref pot -> pwm target
  int vref = analogRead(VREF_PIN); // 0-1023
  uint8_t desired = vref >> 2; // 0-255
  if (desired > MAX_PWM_VAL) desired = MAX_PWM_VAL;
  pwm_target = desired;

  if (checkOvercurrent()) {
    faultEnter("OVERCURRENT");
  }

  if (fault) {
    if (millis() - fault_time > FAULT_RETRY_MS) {
      fault = false;
      Serial.println("Fault cleared - retry allowed");
    } else {
      delay(50);
      return;
    }
  }

  rampStep();

  // read halls (debounced majority)
  int r1 = readMajority(H1_PIN);
  int r2 = readMajority(H2_PIN);
  int r3 = readMajority(H3_PIN);
  int step = hallLogicToStep(r1, r2, r3);
  if (step < 0) {
    allOutputsOff();
    Serial.println("INVALID_HALL");
    delay(5);
    return;
  }

  // apply commutation with pwm_out
  applyStep(step, pwm_out);

  // debug
  Serial.print("HRAW: "); Serial.print(r1); Serial.print(r2); Serial.print(r3);
  Serial.print(" STEP: "); Serial.print(step);
  Serial.print(" PWM: "); Serial.print(pwm_out);
  Serial.print(" I: "); Serial.println(analogRead(CURRENT_SENSE_PIN));

  delay(5);
}
