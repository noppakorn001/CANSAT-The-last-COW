#include <SPI.h>
#include <LoRa.h>

// Teensy4.1 SPI1: MISO=1, MOSI=26, SCK=27
SPIClass &spiLoRa = SPI1;

// LoRa ไม่่สามารถเขียน slave address ได้โดยตรง
// ต้องใช้ pinMode และ digitalWrite เพื่อควบคุม Chip Select (CS) ของแต่ละ LoRa moduleฃ

// LORA1
#define LORA1_CS 3
#define LORA1_DIO0 4

// LORA2
#define LORA2_CS 9
#define LORA2_DIO0 0

void setup() {
  Serial.begin(115200);
  delay(500);

  // ตั้งค่า pinMode สำหรับ CS
  pinMode(LORA1_CS, OUTPUT);
  pinMode(LORA2_CS, OUTPUT);


  // ตั้งค่าเริ่มต้นให้ปิด CS ทั้งสองตัว
  digitalWrite(LORA1_CS, HIGH);
  digitalWrite(LORA2_CS, HIGH);

  // เริ่ม SPI1
  spiLoRa.begin();

  // เริ่มต้น LoRa1
  Serial.println("Starting LoRa1...");
  digitalWrite(LORA1_CS, LOW);  // เปิด CS สำหรับ LoRa1
  LoRa.setSPI(SPI1);
  LoRa.setPins(LORA1_CS, static_cast<IRQ_NUMBER_t>(digitalPinToInterrupt(LORA1_DIO0)));
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa1 failed!");
  } else {
    Serial.println("LoRa1 ready");
  }


  // ตั้งค่า spreading factor, bandwidth และ coding rate สำหรับ LoRa1
  LoRa.setSpreadingFactor(9);      // SF9
  LoRa.setSignalBandwidth(125E3);  // BW125
  LoRa.setCodingRate4(5);          // CR 4/5
  Serial.println("LoRa1 configured with SF9 BW125 CR 4/5");
  // ค่าพวกนี้สัมพันธ์กับความสามารถในการส่งข้อมูลและระยะทาง แต่รายนการ CANSAT-ROCKET กำหนดค่านี้
  // conding rate 4/5 มาจากคำแนะนำของ Data Sheet ของ LoRa module ที่ผมใช้



  digitalWrite(LORA1_CS, HIGH);  // ปิด CS สำหรับ LoRa1

  // เริ่มต้น LoRa2
  Serial.println("Starting LoRa2...");
  digitalWrite(LORA2_CS, LOW);  // เปิด CS สำหรับ LoRa2
  LoRa.setSPI(SPI1);
  LoRa.setPins(LORA2_CS, static_cast<IRQ_NUMBER_t>(digitalPinToInterrupt(LORA2_DIO0)));
  if (!LoRa.begin(917E6)) {
    Serial.println("LoRa2 failed!");
  } else {
    Serial.println("LoRa2 ready");
  }

  // ตั้งค่า spreading factor, bandwidth และ coding rate สำหรับ LoRa2
  LoRa.setSpreadingFactor(9);      // SF9
  LoRa.setSignalBandwidth(125E3);  // BW125
  LoRa.setCodingRate4(5);          // CR 4/5
  Serial.println("LoRa2 configured with SF9 BW125 CR 4/5");

  digitalWrite(LORA2_CS, HIGH);  // ปิด CS สำหรับ LoRa2
}

void loop() {
  static int counter = 0;

  // === ส่งจาก LoRa1 ที่ความถี่ 915 MHz ===
  digitalWrite(LORA2_CS, HIGH);  // ปิด CS ของ LoRa2
  digitalWrite(LORA1_CS, LOW);   // เปิด CS ของ LoRa1

  LoRa.setSPI(SPI1);
  LoRa.setPins(LORA1_CS, static_cast<IRQ_NUMBER_t>(digitalPinToInterrupt(LORA1_DIO0)));
  LoRa.begin(915E6);  // ตั้งค่าความถี่ 915 MHz

  String message1 = "LoRa1 Message #" + String(counter);
  Serial.println("[LoRa1] Sending: " + message1);
  LoRa.beginPacket();
  LoRa.print(message1);
  LoRa.endPacket(true);

  digitalWrite(LORA1_CS, HIGH);                      // ปิด CS ของ LoRa1

  delay(200);  // ลด delay ระหว่างการส่งจาก LoRa1 และ LoRa2

  // === ส่งจาก LoRa2 ที่ความถี่ 917 MHz ===
  digitalWrite(LORA1_CS, HIGH);  // ปิด CS ของ LoRa1
  digitalWrite(LORA2_CS, LOW);   // เปิด CS ของ LoRa2

  LoRa.setSPI(SPI1);
  LoRa.setPins(LORA2_CS, static_cast<IRQ_NUMBER_t>(digitalPinToInterrupt(LORA2_DIO0)));
  LoRa.begin(917E6);  // ตั้งค่าความถี่ 917 MHz

  String message2 = "LoRa2 Message #" + String(counter);
  Serial.println("[LoRa2] Sending: " + message2);
  LoRa.beginPacket();
  LoRa.print(message2);
  LoRa.endPacket(true);

  digitalWrite(LORA2_CS, HIGH);                      // ปิด CS ของ LoRa2

  counter++;
  delay(200);  
}