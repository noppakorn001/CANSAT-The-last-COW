#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SDA_PIN 17
#define SCL_PIN 16

Adafruit_BME280 bme;

// โค้ดนี้ใช้ EMA (Exponential Moving Average) เพื่อกรองค่า altitude ซึ่งเป็น algorithm ที่เป็น IIR (Infinite Impulse Response) filter
// Infinite Impulse Response filter เป็น filter ที่มีการตอบสนองต่อสัญญาณที่ไม่สิ้นสุด ซึ่งเหมาะสำหรับการกรองสัญญาณที่มี noise
// ถ้าอ่าน datasheetของ BME280 จะพบว่า sensor นี้มี noise ที่ค่อนข้างสูง ดังนั้นการใช้ filter จะช่วยให้ได้ค่าที่แม่นยำมากขึ้น

const int NUM_SAMPLES = 100;  // ตัวอย่างจำนวน 100 ครั้งในการคำนวณค่าเฉลี่ยของ pressure
float referencePressure = 0;

float filteredAltitude = 0;
const float alpha = 0.3;  // ค่า alpha สำหรับ Exponential Moving Average (EMA)

void setup() {
  Serial.begin(115200);
  Wire1.begin();            // ใช้ Wire1 สำหรับ Teensy 4.1 (SDA1=17, SCL1=16)
  bme.begin(0x76, &Wire1);  // ไม่ต้องตรวจสอบผลลัพธ์ ผมเช็คก่อนแล้วว่าใช้งานได้

  // อ่านค่า pressure หลายครั้งเพื่อหาค่าเฉลี่ย ลด noise
  float sumPressure = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sumPressure += bme.readPressure() / 100.0F;  // hPa
    delay(50);                                   // delay ต้องดูความเร็วของ sensor ด้วย
  }
  referencePressure = sumPressure / NUM_SAMPLES;

  Serial.print("Reference Pressure (average of ");
  Serial.print(NUM_SAMPLES);
  Serial.print(" samples) = ");
  Serial.print(referencePressure);
  Serial.println(" hPa");
}

void loop() {
  float temperature = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float humidity = bme.readHumidity();

  Serial.print("T=");
  Serial.print(temperature);

  Serial.print("P=");
  Serial.print(pressure);


  Serial.print("H=");
  Serial.print(humidity);

  // คำนวณความสูงสัมพัทธ์ (relative altitude) เพราะปกติจะเทียบ sea level pressure
  // ใช้สูตร barometric formula เพื่อคำนวณความสูงจาก pressure
  float altitude = 44330.0 * (1.0 - pow(pressure / referencePressure, 0.1903));

  // Exponential Moving Average filter
  static bool firstRun = true;
  if (firstRun) {
    filteredAltitude = altitude;
    firstRun = false;
  } else {
    filteredAltitude = alpha * altitude + (1 - alpha) * filteredAltitude;
  }

  Serial.print("EMA=");
  Serial.print(filteredAltitude);

  Serial.println();
  delay(100);
}