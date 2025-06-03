// ...existing code...
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>



const float alpha = 0.2;



#define SDA_PIN 17
#define SCL_PIN 16

Adafruit_BME280 bme;

const int NUM_SAMPLES = 100;
float referencePressure = 0;

float filteredAltitude = 0;
const float alpha_alt = 0.3;  // สำหรับกรองความสูง

// --- เพิ่มตัวแปรสำหรับ launch detection ---
unsigned long tnow = 0, tread = 0;
float r0 = 0, r1 = 0, trend = 0;
int checklunch = 0;

void setup() {
  Serial.begin(115200);
  delay(500);


  // --- BME280 ---
  Wire1.begin();  // ใช้ Wire1 สำหรับ Teensy 4.1 (SDA1=17, SCL1=16)
  bme.begin(0x76, &Wire1);

  // อ่านค่า pressure หลายครั้งเพื่อหาค่าเฉลี่ย ลด noise
  float sumPressure = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sumPressure += bme.readPressure() / 100.0F;  // hPa
    delay(100);
  }
  referencePressure = sumPressure / NUM_SAMPLES;

  Serial.print("Reference Pressure (average of ");
  Serial.print(NUM_SAMPLES);
  Serial.print(" samples) = ");
  Serial.print(referencePressure);
  Serial.println(" hPa");
}

void loop() {

  // --- BME280 ---
  float temperature = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float humidity = bme.readHumidity();

  // คำนวณความสูงสัมพัทธ์ (relative altitude)
  float altitude = 44330.0 * (1.0 - pow(pressure / referencePressure, 0.1903));

  // Exponential Moving Average filter สำหรับความสูง
  static bool firstRun = true;
  if (firstRun) {
    filteredAltitude = altitude;
    firstRun = false;
  } else {
    filteredAltitude = alpha_alt * altitude + (1 - alpha_alt) * filteredAltitude;
  }

  Serial.print(filteredAltitude);

  Serial.println();

  // ===Detect Launch===ห
  tnow = millis();
  if (tnow > tread) {
    r1 = filteredAltitude;
    trend = abs(r1 - r0);
    tread = tnow + 200;
    r0 = r1;
    if (trend > 0) {
      checklunch += 1;
    } else {
      checklunch = 0;
    }
    if (checklunch >= 5 && filteredAltitude >= 10) {
      Serial.print("Canard");
    }
    Serial.print(checklunch);
    Serial.println("");
  }
}
// ...existing code...