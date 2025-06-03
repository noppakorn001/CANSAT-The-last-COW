#include <SD.h>
#include <SPI.h>
#include <LoRa.h> // เพิ่มไลบรารี LoRa

#define RXD2 16
#define TXD2 17

#define SD_CS 13    // Chip Select
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK  18

// กำหนดพิน LoRa (ตัวอย่างสำหรับ Heltec ESP32 LoRa)
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

#define LORA_BAND 919925000   // 919.925 MHz

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // UART1 → GPIO16(RX), 17(TX)
  Serial.println("Initializing SD card...");

 
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!");
    while (true);  
  }
  Serial.println("SD Card initialized.");
  Serial.println("Waiting for data from Teensy...");

  // เริ่มต้น LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(9);      // SF9
  LoRa.setSignalBandwidth(125E3);  // 125 kHz
  Serial.println("LoRa Initial OK!");
}

void loop() {
  if (Serial2.available()) {
    String received = Serial2.readStringUntil('\n');
    received.trim();  // ลบช่องว่างหรือ '\r'

    Serial.print("Received: ");
    Serial.println(received);

    // ส่งข้อมูลผ่าน LoRa
    LoRa.beginPacket();
    LoRa.print(received);
    LoRa.endPacket();

    File dataFile = SD.open("/data.csv", FILE_APPEND);
    if (dataFile) {
      dataFile.println(received);
      dataFile.close();
      Serial.println("Saved to SD card.");
    } else {
      Serial.println("Failed to open data.csv");
    }
  }
}