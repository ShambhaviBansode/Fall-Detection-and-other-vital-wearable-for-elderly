#define BLYNK_TEMPLATE_ID "TMPL3ehUpEt5k"
#define BLYNK_TEMPLATE_NAME "max30100 pulse oxi"
#define BLYNK_AUTH_TOKEN "I8XPrkddXBguC1w6rZqNy2-OotH4dt9o"

#include <Wire.h>
#include <U8g2lib.h>
#include "MAX30100_PulseOximeter.h"
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// WiFi credentials
char ssid[] = "Galaxy F55";
char pass[] = "mummykaphone";

// OLED
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// MAX30100
PulseOximeter pox;
#define REPORTING_PERIOD_MS 1000
uint32_t tsLastReport = 0;

void onBeatDetected() {
  Serial.println("Beat!");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(21, 22); // ESP32 I2C: SDA=21, SCL=22

  // OLED setup
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 20, "Initializing...");
  u8g2.sendBuffer();

  // Connect to WiFi and Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // MAX30100 init
  if (!pox.begin()) {
    Serial.println("MAX30100 FAILED");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 30, "MAX30100 FAILED");
    u8g2.sendBuffer();
    while (true);
  }

  Serial.println("MAX30100 SUCCESS");
  u8g2.clearBuffer();
  u8g2.drawStr(0, 30, "MAX30100 SUCCESS");
  u8g2.sendBuffer();

  pox.setIRLedCurrent(MAX30100_LED_CURR_27_1MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop() {
  Blynk.run();
  pox.update();
  delay(10);

  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    float hr = pox.getHeartRate();
    float spo2 = pox.getSpO2();

    bool fingerDetected = (hr > 40 && hr < 180 && spo2 > 85 && spo2 <= 100);

    Serial.print("HR: ");
    Serial.print(hr);
    Serial.print(" | SpO2: ");
    Serial.print(spo2);
    Serial.print(" | Finger: ");
    Serial.println(fingerDetected ? "Yes" : "No");

    // OLED display
    u8g2.clearBuffer();
    if (fingerDetected) {
      char hrStr[20], spo2Str[20];
      sprintf(hrStr, "HR: %d bpm", (int)hr);
      sprintf(spo2Str, "SpO2: %d %%", (int)spo2);

      u8g2.drawStr(0, 15, "Pulse Oximeter");
      u8g2.drawStr(0, 35, hrStr);
      u8g2.drawStr(0, 55, spo2Str);

      // Send to Blynk
      Blynk.virtualWrite(V0, (int)hr);
      Blynk.virtualWrite(V1, (int)spo2);
    } else {
      u8g2.drawStr(0, 30, "Place Finger");

      // Send zero or -1 to indicate no reading
      Blynk.virtualWrite(V0, 0);
      Blynk.virtualWrite(V1, 0);
    }
    u8g2.sendBuffer();
    tsLastReport = millis();
  }
}
