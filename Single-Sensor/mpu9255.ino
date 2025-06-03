/*
Raw data example
This example reads raw readings from the magnetometer gyroscope and the accelerometer and then
displays them in serial monitor.
*/

//อันนี้ผมใช้ตัวอย่างที่มีทั่วไปของ mpu6050 แต่เปลี่ยน library เป็นของ asukiaaa mpu9250

#include <Wire.h>
#include "MPU9250_asukiaaa.h"
#include "Kalman.h"

MPU9250_asukiaaa mpu;

// Kalman filter สำหรับ Roll และ Pitch
Kalman kalmanX;
Kalman kalmanY;

// ตัวแปรสำหรับเก็บค่า Offset
float accelOffsetX = 0.0;
float accelOffsetY = 0.0;
float accelOffsetZ = 0.0;

float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;
float gyroOffsetZ = 0.0;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float kalAngleX, kalAngleY;
float dt, currentTime, previousTime;
float AccErrorX = 0, AccErrorY = 0, AccErrorZ = 0, GyroErrorX = 0, GyroErrorY = 0;
int c = 0;

void calculate_IMU_error() {
    // ในการ set mpu ต้องวางให้ตรงแนวที่ต้องการและนิ่งนิ่งประมาณ 10 วินาทีนะครับ
    while (c < 200) {
        mpu.accelUpdate();
        mpu.gyroUpdate();
        AccX = mpu.accelX();
        AccY = mpu.accelY();
        AccZ = mpu.accelZ();
        AccErrorX += AccX;
        AccErrorY += AccY;
        AccErrorZ += AccZ;
        c++;
        delay(5);
    }
    AccErrorX /= 200;
    AccErrorY /= 200;
    AccErrorZ /= 200;
    c = 0;
    while (c < 2000) {
        mpu.gyroUpdate();
        GyroX = mpu.gyroX();
        GyroY = mpu.gyroY();
        GyroErrorX += GyroX;
        GyroErrorY += GyroY;
        c++;
        delay(2);
    }
    GyroErrorX /= 2000;
    GyroErrorY /= 2000;
    Serial.print("AccErrorX: "); Serial.println(AccErrorX);
    Serial.print("AccErrorY: "); Serial.println(AccErrorY);
    Serial.print("AccErrorZ: "); Serial.println(AccErrorZ);
    Serial.print("GyroErrorX: "); Serial.println(GyroErrorX);
    Serial.print("GyroErrorY: "); Serial.println(GyroErrorY);
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(100000);

    uint8_t id;
    if (mpu.readId(&id) != 0) {
        Serial.println("Initialization failed. Check connections or I2C address.");
        while (1);
    } else {
        Serial.print("Initialization successful! Device ID: ");
        Serial.println(id, HEX);
    }

    mpu.beginAccel(ACC_FULL_SCALE_2_G);
    mpu.beginGyro(GYRO_FULL_SCALE_250_DPS);
    mpu.beginMag(MAG_MODE_CONTINUOUS_8HZ);

    delay(100);
    calculate_IMU_error();
    delay(1000);

    // อ่านค่าเริ่มต้นเพื่อเซ็ต Kalman filter
    mpu.accelUpdate();
    AccX = mpu.accelX() - AccErrorX;
    AccY = mpu.accelY() - AccErrorY;
    AccZ = mpu.accelZ() - AccErrorZ;

    float roll  = atan2(AccY, AccZ) * 180.0 / PI;
    float pitch = atan2(AccX, AccZ) * 180.0 / PI;

    kalmanX.setAngle(roll);
    kalmanY.setAngle(pitch);

    previousTime = millis();
}

void loop()
{
    float meanKroll = 0, meanKpitch = 0;
    for (int i = 0; i < 10; i++) {
        mpu.accelUpdate();
        mpu.gyroUpdate();

        previousTime = currentTime;
        currentTime = millis();
        dt = (currentTime - previousTime) / 1000.0;

        AccX = mpu.accelX() - AccErrorX;
        AccY = mpu.accelY() - AccErrorY;
        AccZ = mpu.accelZ() - AccErrorZ;
        GyroX = mpu.gyroX() - GyroErrorX;
        GyroY = mpu.gyroY() - GyroErrorY;

        float roll  = atan2(AccY, AccZ) * 180.0 / PI;
        float pitch = atan2(AccX, AccZ) * 180.0 / PI;

        kalAngleX = kalmanX.getAngle(roll, GyroX, dt);
        kalAngleY = kalmanY.getAngle(pitch, GyroY, dt);

        meanKroll += kalAngleX;
        meanKpitch += kalAngleY;
        delay(2);
    }
    meanKroll /= 10.0;
    meanKpitch /= 10.0;

    Serial.print("AccelX_raw:"); Serial.print(AccX, 3); Serial.print(",");
    Serial.print("AccelY_raw:"); Serial.print(AccY, 3); Serial.print(",");
    Serial.print("GyroX_raw:"); Serial.print(GyroX, 3); Serial.print(",");
    Serial.print("GyroY_raw:"); Serial.print(GyroY, 3); Serial.print(",");

    Serial.print("AccelX_Kalman:"); Serial.print(meanKpitch, 2); Serial.print(","); // Pitch (angle)
    Serial.print("AccelY_Kalman:"); Serial.print(meanKroll, 2); Serial.print(",");  // Roll (angle)
    Serial.print("GyroX_Kalman:"); Serial.print(meanKroll, 2); Serial.print(",");   // Roll (angle)
    Serial.print("GyroY_Kalman:"); Serial.print(meanKpitch, 2);                     // Pitch (angle)

    Serial.println();
    delay(50);
}