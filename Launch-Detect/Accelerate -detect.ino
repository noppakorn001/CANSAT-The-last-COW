// Acceleration detection using MPU9250 อันนี้ผมเขียนแบบง่ายๆนะครับ
// โค้ดใช้จริงๆเขียนใหม่หนน้างานแล้วไม่ได้่บันทึกไว้ แต่หลักการประมาณนี้แหละ
// ควรใช้ filter low-pass or kalman อะไรพวกนี้แหละอันนี้ผมขี้เกียจเลยเอา bme version มาใช้

#include <Wire.h>
#include <MPU9250_asukiaaa.h>

// === MPU9255 Setup ===
MPU9250_asukiaaa mpu;

const float alpha = 0.2; 

// === Filter Variables ===
float filteredPitchRate = 0;
float filteredRollRate = 0;
float filteredYawRate = 0; // เพิ่มตัวแปรสำหรับ gyro Z
float filteredAccelX = 0;
float filteredAccelY = 0;
float filteredAccelZ = 0;

#define SDA_PIN 17
#define SCL_PIN 16



float filteredAccelZ = 0;
const float alpha_alt = 0.3; // สำหรับกรองความสูง

// --- เพิ่มตัวแปรสำหรับ launch detection ---
unsigned long tnow = 0, tread = 0;
float r0 = 0, r1 = 0, trend = 0;
int checklunch = 0;

void setup()
{
    Serial.begin(115200);
    delay(500);

    // --- MPU9250 ---
    Wire.begin();
    mpu.setWire(&Wire);
    mpu.beginAccel();
    mpu.beginGyro();
}

void loop()
{
    // --- MPU9250 ---
    mpu.gyroUpdate();
    mpu.accelUpdate();
    delay(10);

    float pitchRate = mpu.gyroY();
    float rollRate = mpu.gyroX();
    float yawRate = mpu.gyroZ();

    float accelX = mpu.accelX();
    float accelY = mpu.accelY();
    float accelZ = mpu.accelZ();

    filteredPitchRate = alpha * pitchRate + (1 - alpha) * filteredPitchRate;
    filteredRollRate = alpha * rollRate + (1 - alpha) * filteredRollRate;
    filteredYawRate = alpha * yawRate + (1 - alpha) * filteredYawRate;
    filteredAccelX = alpha * accelX + (1 - alpha) * filteredAccelX;
    filteredAccelY = alpha * accelY + (1 - alpha) * filteredAccelY;
    filteredAccelZ = alpha * accelZ + (1 - alpha) * filteredAccelZ;


    /*
      // --- Output ---
      Serial.print("[Gyro] X: ");
      Serial.print(filteredRollRate, 2);
      Serial.print(" Y: ");
      Serial.print(filteredPitchRate, 2);
      Serial.print(" Z: ");
      Serial.print(filteredYawRate, 2);
      Serial.print(" deg/s | ");

      Serial.print("[Accel] X: ");
      Serial.print(filteredAccelX, 3);
      Serial.print(" Y: ");
      Serial.print(filteredAccelY, 3);
      Serial.print(" Z: ");
      Serial.print(filteredAccelZ, 3);

      Serial.print(" | [BME] T=");
      Serial.print(temperature);
      Serial.print(" P=");
      Serial.print(pressure);
      Serial.print(" H=");
      Serial.print(humidity);
      Serial.print(" Alt(EMA)=");
      */
    Serial.print(filteredAccelZ);

    Serial.println();

    // ===Detect Launch===
    tnow = millis();
    if (tnow > tread)
    {
        r1 = filteredAccelZ;
        trend = abs(r1 - r0);
        tread = tnow + 200;
        r0 = r1;
        if (trend > 0)
        {
            checklunch += 1;
        }
        else
        {
            checklunch = 0;
        }
        if (checklunch >= 5 && filteredAccelZ >= 3)
        {
            Serial.print("Canard");
        }
        Serial.print(checklunch);
        Serial.println("");
    }
}
// ...existing code...