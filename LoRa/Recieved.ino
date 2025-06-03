// รับค่าจาก LoRa 2 ตัวที่ส่งมาคนละความถี่ (917 MHz และ 915 MHz)

#include <SPI.h>
#include <LoRa.h>

// กำหนดขา LoRa สำหรับ TTGO LoRa32 (SX1276)
#define LORA_CS 18
#define LORA_RST 14
#define LORA_DIO0 26

void setup() {
  Serial.begin(115200);
  delay(1000);

  // เริ่มต้น LoRa
  Serial.println("Initializing LoRa...");
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0); 

  if (!LoRa.begin(915000000)) { // ตั้งค่าความถี่ 915 MHz โดยปกติจะเขียน 915E6 = 915 * 1000000 = 915000000
    Serial.println("LoRa initialization failed!");
    while (1);
  }

  // ตั้งค่า Spreading Factor, Bandwidth และ Coding Rate ตามรายการ CANSAT-ROCKET กำหนด
  LoRa.setSpreadingFactor(9);       // SF9
  LoRa.setSignalBandwidth(125E3);  // BW125 kHz
  LoRa.setCodingRate4(5);          // CR 4/5
  Serial.println("LoRa configured: SF9, BW125 kHz, CR 4/5,915 MHz");
}

void loop() {
  String receivedMessage;

  // fuction to receive LoRa packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    receivedMessage = "";
    while (LoRa.available()) {
      receivedMessage += (char)LoRa.read();
    }
    Serial.print("[Canard] : ");
    Serial.println(receivedMessage);
  }

  // delay ปรับตาม delay ของ transmission
  delay(10);
}