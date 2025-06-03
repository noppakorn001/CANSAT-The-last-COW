/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2020-2024 Timo Sandmann
 * ผมใช้ FreeRTOS ของคนนี้เป็น reference แล้วค่อยๆเขียนเลาะไปมันจะมั่วๆหน่อย
 */





#include "arduino_freertos.h"
#include "avr/pgmspace.h"

// Fix for Adafruit_BME280/MSBFIRST/LSBFIRST macro issue
#ifndef MSBFIRST
#define MSBFIRST arduino::MSBFIRST
#endif
#ifndef LSBFIRST
#define LSBFIRST arduino::LSBFIRST
#endif

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "MPU9250_asukiaaa.h"
#include "Kalman.h"


static void taskSensor(void*) {
    // BME280 config
    #define SDA_PIN 17
    #define SCL_PIN 16
    Adafruit_BME280 bme;
    const int NUM_SAMPLES = 100;
    float referencePressure = 0;
    float filteredAltitude = 0;
    const float alpha = 0.3;
    static bool firstRun = true;

    // MPU9250 config
    MPU9250_asukiaaa mpu;
    Kalman kalmanX;
    Kalman kalmanY;
    float AccX, AccY, AccZ;
    float GyroX, GyroY; // removed GyroZ (unused)
    float kalAngleX, kalAngleY;
    float dt, currentTime, previousTime;
    float AccErrorX = 0, AccErrorY = 0, AccErrorZ = 0, GyroErrorX = 0, GyroErrorY = 0;
    int c = 0;

    // Init BME280
    Wire1.begin();
    bme.begin(0x76, &Wire1);

    // Init MPU9250
    Wire.begin();
    Wire.setClock(100000);

    uint8_t id;
    if (mpu.readId(&id) != 0) {
        Serial.println("MPU Initialization failed. Check connections or I2C address.");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
        Serial.print("MPU Initialization successful! Device ID: ");
        Serial.println(id, arduino::HEX); // use arduino::HEX
    }

    mpu.beginAccel(ACC_FULL_SCALE_2_G);
    mpu.beginGyro(GYRO_FULL_SCALE_250_DPS);
    mpu.beginMag(MAG_MODE_CONTINUOUS_8HZ);

    // Calibrate referencePressure (BME280)
    float sumPressure = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sumPressure += bme.readPressure() / 100.0F;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    referencePressure = sumPressure / NUM_SAMPLES;

    Serial.print("Reference Pressure (average of ");
    Serial.print(NUM_SAMPLES);
    Serial.print(" samples) = ");
    Serial.print(referencePressure);
    Serial.println(" hPa");

    // Calculate IMU error (MPU9250)
    c = 0;
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
        vTaskDelay(pdMS_TO_TICKS(5));
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
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    GyroErrorX /= 2000;
    GyroErrorY /= 2000;
    Serial.print("AccErrorX: "); Serial.println(AccErrorX);
    Serial.print("AccErrorY: "); Serial.println(AccErrorY);
    Serial.print("AccErrorZ: "); Serial.println(AccErrorZ);
    Serial.print("GyroErrorX: "); Serial.println(GyroErrorX);
    Serial.print("GyroErrorY: "); Serial.println(GyroErrorY);

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

    while (true) {
        // --- BME280 ---
        float temperature = bme.readTemperature();
        float pressure = bme.readPressure() / 100.0F;
        float humidity = bme.readHumidity();

        float altitude = 44330.0 * (1.0 - pow(pressure / referencePressure, 0.1903));
        if (firstRun) {
            filteredAltitude = altitude;
            firstRun = false;
        } else {
            filteredAltitude = alpha * altitude + (1 - alpha) * filteredAltitude;
        }

        // --- MPU9250 ---
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
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        meanKroll /= 10.0;
        meanKpitch /= 10.0;

        // Print ข้อมูล BME280 และ MPU9250
        Serial.print("T="); Serial.print(temperature);
        Serial.print(",P="); Serial.print(pressure);
        Serial.print(",H="); Serial.print(humidity);
        Serial.print(",EMA="); Serial.print(filteredAltitude);
        Serial.print(",AccelX:"); Serial.print(AccX, 3);
        Serial.print(",AccelY:"); Serial.print(AccY, 3);
        Serial.print(",AccelZ:"); Serial.print(AccZ, 3);
        Serial.print(",Kroll:"); Serial.print(meanKroll, 2);
        Serial.print(",Kpitch:"); Serial.print(meanKpitch, 2);
        Serial.println();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// static void task2(void*) { ... }  // ลบ task2 ออกทั้งหมด

FLASHMEM __attribute__((noinline)) void setup() {
    Serial.begin(0);
    delay(2'000);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

    xTaskCreate(taskSensor, "taskSensor", 2048, nullptr, 2, nullptr);

    Serial.println("setup(): starting scheduler...");
    Serial.flush();

    vTaskStartScheduler();
}

void loop() {}