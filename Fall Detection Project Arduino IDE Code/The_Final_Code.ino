#define BLYNK_TEMPLATE_ID "TMPL3XZ5qjI5u"
#define BLYNK_TEMPLATE_NAME "Fall Detector and Heart Rate monitor 2"
#define BLYNK_AUTH_TOKEN "4wi7bOid5ebC80e6ozPwusonIHxhRKl6"


#include <Wire.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <U8g2lib.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// WiFi credentials
char ssid[] = "MANIT";
char pass[] = "";

// OLED
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// MAX30100
PulseOximeter pox;
#define REPORTING_PERIOD_MS 1000
uint32_t tsLastReport = 0;

// ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
const float FALL_THRESHOLD = 40.0;
const float SUDDEN_IMPACT_THRESHOLD = 10.0;
float prevAccel = 0;
unsigned long lastFallTime = 0;
const unsigned long FALL_ALERT_INTERVAL = 5000; // 5 sec

// Buzzer & Button
#define BUZZER_PIN 25
#define BUTTON_PIN 26
bool buzzerOn = false;
unsigned long buzzerStartTime = 0;
const unsigned long BUZZER_TIMEOUT = 6000; // 6 seconds

// GPS
TinyGPSPlus gps;
HardwareSerial GPSserial(1); // UART1 (GPIO 16 = RX, 17 = TX)

void onBeatDetected() {
  Serial.println("Beat!");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(21, 22);  // SDA, SCL

  // OLED
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.clearBuffer();
  u8g2.drawStr(20, 15, "Initialising...");

// Smiley face
u8g2.drawCircle(64, 40, 18);    // Head
u8g2.drawDisc(56, 34, 2);       // Left eye
u8g2.drawDisc(72, 34, 2);       // Right eye

// Smile (using lines for better arc)
u8g2.drawLine(56, 46, 59, 48);
u8g2.drawLine(59, 48, 64, 49);
u8g2.drawLine(64, 49, 69, 48);
u8g2.drawLine(69, 48, 72, 46);

u8g2.sendBuffer();


  // Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // MAX30100
  if (!pox.begin()) {
    Serial.println("MAX30100 FAILED");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 30, "MAX30100 FAILED");
    u8g2.sendBuffer();
    while (1);
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_27_1MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 30, "MAX30100 SUCCESS");
  u8g2.sendBuffer();

  // ADXL345
  if (!accel.begin()) {
    Serial.println("ADXL345 not detected");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  Serial.println("ADXL345 Ready");

  // Buzzer & Button
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUZZER_PIN, LOW);

  // GPS
  GPSserial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("GPS Initialized");
}

void loop() {
  Blynk.run();
  pox.update();
  delay(10); // small required delay

  // Read GPS data
  while (GPSserial.available()) {
    gps.encode(GPSserial.read());
  }

  // Accelerometer
  sensors_event_t event;
  accel.getEvent(&event);
  float ax = event.acceleration.x;
  float ay = event.acceleration.y;
  float az = event.acceleration.z;
  float totalAccel = sqrt(ax * ax + ay * ay + az * az);
  float accelChange = abs(totalAccel - prevAccel);
  prevAccel = totalAccel;

  // Fall detection
  if (totalAccel > FALL_THRESHOLD && accelChange > SUDDEN_IMPACT_THRESHOLD && millis() - lastFallTime > FALL_ALERT_INTERVAL) {
    Serial.println("🚨 Fall Detected!");
    Blynk.logEvent("fall_detected", "⚠️ Fall Detected!⚠️");

    // Send GPS location with alert
    if (gps.location.isValid()) {
      Blynk.virtualWrite(V2, gps.location.lat());
      Blynk.virtualWrite(V3, gps.location.lng());

      String link = "https://maps.google.com/?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
      Blynk.logEvent("gps_location", link);
    }

    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOn = true;
    buzzerStartTime = millis();
    lastFallTime = millis();
  }

  // Buzzer cancel or timeout
  if (buzzerOn) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("✅ Button Pressed - User is OK");
      digitalWrite(BUZZER_PIN, LOW);
      buzzerOn = false;
      Blynk.logEvent("im_ok", "✅ I'm OK 😊😊");
      delay(500);
    } else if (millis() - buzzerStartTime >= BUZZER_TIMEOUT) {
      Serial.println("⏱️ Timeout - Turning off buzzer");
      digitalWrite(BUZZER_PIN, LOW);
      buzzerOn = false;
      Blynk.logEvent("alert", "🚨 Possible Emergency Detected!");
    }
  }

  // Pulse Oximeter + GPS Reporting
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    float hr = pox.getHeartRate();
    float spo2 = pox.getSpO2();
    bool fingerDetected = (hr > 40 && hr < 180 && spo2 > 85 && spo2 <= 100);

    Serial.print("HR: "); Serial.print(hr);
    Serial.print(" | SpO2: "); Serial.print(spo2);
    Serial.print(" | Finger: "); Serial.println(fingerDetected ? "Yes" : "No");

    u8g2.clearBuffer();

    if (fingerDetected) {
      char hrStr[20], spo2Str[20];
      sprintf(hrStr, "HR: %d bpm", (int)hr);
      sprintf(spo2Str, "SpO2: %d %%", (int)spo2);
      u8g2.drawStr(0, 15, "Pulse Oximeter");
      u8g2.drawStr(0, 35, hrStr);
      u8g2.drawStr(0, 55, spo2Str);

      Blynk.virtualWrite(V0, (int)hr);
      Blynk.virtualWrite(V1, (int)spo2);
    } else {
      u8g2.drawStr(0, 30, "Place Finger");
      Blynk.virtualWrite(V0, 0);
      Blynk.virtualWrite(V1, 0);
    }

    // GPS to OLED and Blynk Map (V2)
    if (gps.location.isValid()) {
      char gpsStr[40];
      sprintf(gpsStr, "Lat:%.2f Lng:%.2f", gps.location.lat(), gps.location.lng());
      u8g2.drawStr(0, 64, gpsStr); // Last row on OLED

      // Send to Blynk Map widget (V2)
      Blynk.virtualWrite(V2, gps.location.lat());
      Blynk.virtualWrite(V3, gps.location.lng());

      Serial.print("GPS → Lat: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" | Lng: ");
      Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("Waiting for GPS signal...");
    }
    u8g2.sendBuffer();
    tsLastReport = millis();
  }
}
