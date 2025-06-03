#include <TinyGPS++.h>

TinyGPSPlus gps;

// กำหนด Serial2: RX2 = 7, TX2 = 8 สำหรับ Teensy 4.1
#define GPS_SERIAL Serial2

// ผมไม่ได้ใช้ SoftwareSerial เพราะ Teensy 4.1 มี Serial2 ที่สามารถใช้ได้เลย

void setup() {
  Serial.begin(115200);      // Serial Monitor
  GPS_SERIAL.begin(9600);    // GPS module baudrate (ปกติ 9600)
  Serial.println("GPS Ready");
}

void loop() {
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
  }

  if (gps.location.isUpdated()) {
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print("  Lon: ");
    Serial.print(gps.location.lng(), 6);
    Serial.print("  Alt: ");
    Serial.print(gps.altitude.meters());
    Serial.print(" m  Sat: ");
    Serial.println(gps.satellites.value());
  }

  delay(200);
}