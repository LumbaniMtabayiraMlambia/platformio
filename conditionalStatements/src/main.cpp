#include <Arduino.h>
   
#include <HardwareSerial.h>
#include <Preferences.h>

HardwareSerial A9G(1);
Preferences preferences;

#define LED_PIN 18

String line = "";
bool ledActive = false;
unsigned long ledOffTime = 0;
int remainingMinutes = 0;
unsigned long lastTick = 0;
String lastSender = "";

void setup() {
  Serial.begin(115200);
  A9G.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(LED_PIN, OUTPUT);

  // Preferences
  preferences.begin("dashcam", false);
  remainingMinutes = preferences.getInt("timeLeft", 0);

  if (remainingMinutes > 0) {
    Serial.print("⚡ RECOVERY: Minutes left: ");
    Serial.println(remainingMinutes);
    startSystem(remainingMinutes);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  delay(3000);

  sendAT("AT");
sendAT("ATE0");                 // disable echo (important)
sendAT("AT+CMGF=1");            // TEXT mode
sendAT("AT+CSCS=\"GSM\"");      // GSM charset (CRITICAL)
sendAT("AT+CNMI=2,2,0,0,0");    // push SMS directly as TEXT
sendAT("AT+CPMS=\"ME\",\"ME\",\"ME\"");
sendAT("AT+CMGD=1,4");

Serial.println("📩 Ready for timed SMS commands");

}

void loop() {

  // ---- READ FROM A9G ----
  while (A9G.available()) {
    char c = A9G.read();
    Serial.write(c);   // ✅ restore modem output

    if (c == '\n') {
      // ---- detect sender ONLY after full line ----
      if (line.startsWith("+CMT:")) {
        int q1 = line.indexOf('"');
        int q2 = line.indexOf('"', q1 + 1);
        if (q1 != -1 && q2 != -1) {
          lastSender = line.substring(q1 + 1, q2);
          Serial.print("📱 SMS from: ");
          Serial.println(lastSender);
        }
      }

      processLine(line);
      line = "";
    } else {
      line += c;
      if (line.length() > 300) line = ""; // safety
    }
  }

  // ---- FORWARD SERIAL AT COMMANDS ----
  while (Serial.available()) {
    A9G.write(Serial.read());
  }

  // ---- TIMER LOGIC ----
  if (ledActive) {
    if (millis() - lastTick >= 60000) {
      lastTick = millis();
      remainingMinutes--;

      preferences.putInt("timeLeft", remainingMinutes);

      Serial.print("⏰ Minutes remaining: ");
      Serial.println(remainingMinutes);

      if (remainingMinutes <= 0) {
        stopSystem("Timer expired");
      }
    }
  }
}

void processLine(String msg) {

  msg.trim();
  msg.toUpperCase();

  // Ignore SMS header
  if (msg.startsWith("+CMT:")) return;

  // 🔴 RESET COMMAND
  if (msg.indexOf("MTABAYIRA") != -1) {
    stopSystem("Reset by MTABAYIRA");
    return;
  }

  // 🔴 OFF COMMAND
  if (msg.indexOf("OFF") != -1) {
    stopSystem("Remote OFF");
    return;
  }

  // 🟢 ADD MINUTES COMMAND
  if (msg.indexOf("ON") != -1 && msg.indexOf("MINUTES") != -1) {

    int mins = extractMinutesFromText(msg);
    if (mins <= 0) return;

    // If already running → ADD time
    if (ledActive) {
      remainingMinutes += mins;
      preferences.putInt("timeLeft", remainingMinutes);

      Serial.print("➕ Added minutes. New total: ");
      Serial.println(remainingMinutes);
    }
    // If OFF → start fresh
    else {
      startSystem(mins);
      Serial.print("🟢 Started for ");
      Serial.print(mins);
      Serial.println(" minutes");
    }

    if (lastSender != "") {
      sendSMS(lastSender, "Time updated: " + String(remainingMinutes) + " minutes remaining");
      lastSender = "";
    }
  }
}


void startSystem(int mins) {
  remainingMinutes = mins;
  ledActive = true;
  lastTick = millis();
  digitalWrite(LED_PIN, HIGH);
  preferences.putInt("timeLeft", remainingMinutes);
}

void stopSystem(String reason) {
  ledActive = false;
  remainingMinutes = 0;
  digitalWrite(LED_PIN, LOW);
  preferences.putInt("timeLeft", 0);

  Serial.print("🔴 LED OFF: ");
  Serial.println(reason);

  if (lastSender != "") {
    sendSMS(lastSender, "System OFF: " + reason);
    lastSender = "";
  }
}

int extractMinutesFromJSON(String txt) {
  int k = txt.indexOf("\"MINUTES\"");
  if (k == -1) return 0;
  int c = txt.indexOf(':', k);
  if (c == -1) return 0;

  String num = "";
  for (int i = c + 1; i < txt.length(); i++) {
    if (isDigit(txt[i])) num += txt[i];
    else if (num.length()) break;
  }
  return num.toInt();
}

int extractMinutesFromText(String txt) {
  int i = txt.indexOf("MINUTES");
  if (i == -1) return 0;

  String num = "";
  for (int j = i - 1; j >= 0; j--) {
    if (isDigit(txt[j])) num = txt[j] + num;
    else if (num.length()) break;
  }
  return num.toInt();
}

void sendAT(const char* cmd) {
  A9G.println(cmd);
  delay(400);
}

void sendSMS(String number, String text) {
  A9G.println("AT+CMGF=1");
  delay(200);
  A9G.print("AT+CMGS=\"");
  A9G.print(number);
  A9G.println("\"");
  delay(200);
  A9G.print(text);
  A9G.write(26);
  delay(500);
}
