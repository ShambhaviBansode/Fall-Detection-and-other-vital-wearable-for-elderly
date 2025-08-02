# Fall-Detection-and-other-vital-wearable-for-elderly

A smart wearable system using **ESP32** that detects **falls** and monitors **heart rate** and **SpO₂ levels** in real-time. Designed for **elderly care, delivery riders, patients**, and **industrial workers**, the device sends instant alerts via **Blynk IoT** and triggers a local **buzzer alarm** during emergencies.

---

## 🛠️ Tech Stack

- **Microcontroller:** ESP32
- **Sensors:** MAX30100 (HR & SpO₂), ADXL345 (Fall detection)
- **Display:** OLED (U8g2 library)
- **Connectivity:** Blynk IoT platform
- **Others:** Push button, Buzzer, (Optional: GPS Module)

---

## ⚙️ Features

-  Real-time heart rate & SpO₂ monitoring  
-  Accurate fall detection using 3-axis accelerometer  
-  Local buzzer alarm and remote Blynk alert  
-  Mobile app dashboard for vitals  
-  "I’m OK" button to avoid false alarms  
-  Optional GPS tracking for emergency location

---


## 🧪 Setup Instructions

1. Connect MAX30100, ADXL345, OLED, and push button to ESP32.
2. Install libraries: `Wire`, `Adafruit_Sensor`, `MAX30100`, `U8g2`, `Blynk`.
3. Flash the code from `code/fall_detection_esp32.ino`.
4. Configure Blynk with your WiFi credentials and auth token.
5. View vitals and alerts on the Blynk mobile app.

---

## 📁 Folder Structure
fall-detection-wearable/
├── code/
│ └── fall_detection_Project_ArduinoIDE_code
├── images/
│ └── prototype_images
│ └── Device_assembly_images
│ └──Blynk_Notifcation_images
│ └──Individual_divice_images
├── README.md

### 👤 Author

**Shambhavi Bansode**  
[LinkedIn](https://www.linkedin.com/in/shambhavi-bansode-35bb77297?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=android_app)
