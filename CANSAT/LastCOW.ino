/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2020-2024 Timo Sandmann
 * ผมใช้ FreeRTOS ของคนนี้เป็น reference แล้วค่อยๆเขียนเลาะไปมันจะมั่วๆหน่อย
 * ในโค้ดนี้ยังมีการ tune เวลา delay และ priority ของ task ต่างๆ เพื่อป้องกันปัญหา timing และ resource contention
 * แต่ผมยังทำได้ๆไม่ดีเท่าไหร่ช่วงบางครั้งอาจมีค่า mpu หายไปบ้าง
 * การบันทึก SD card เป็นการบันทึกสิ่งที่กำลังจะส่งผ่าน LoRa
 * ผมหาโค้ดเต็มๆที่ใช้ไม่เจอเอาโค้ดนี้ไปใช้ก่อนนะครับ555555
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
#include <TinyGPS++.h>
#include <SPI.h>
#include <LoRa.h>
#include <SD.h>
#include <FS.h>

TinyGPSPlus gps;

// กำหนด Serial2: RX2 = 7, TX2 = 8 สำหรับ Teensy 4.1
#define GPS_SERIAL Serial2

// SPI1: MISO=1, MOSI=26, SCK=27
SPIClass &spiLoRa = SPI1;

// LORA1
#define LORA1_CS 3
#define LORA1_DIO0 4
#define LORA1_LED 28

// LORA2
#define LORA2_CS 9
#define LORA2_DIO0 0
#define LORA2_LED 29

// --- ปรับจูนเพื่อป้องกันปัญหา timing และ resource contention ---

// 1. กำหนด priority ให้ LoRa ต่ำกว่า sensor
// 2. ลด stack size ของ LoRa ให้เหมาะสม
// 3. เพิ่ม vTaskDelay(1) ใน loop ของ sensor เพื่อ yield CPU
// 4. ใช้ mutex ป้องกันการชนกันของ Serial (optional, ถ้า Serial มีปัญหา)
// 5. ปรับ vTaskDelay ของ LoRa ให้มากขึ้นถ้า sensor sampling สำคัญกว่า

#include <FreeRTOS.h>
#include <semphr.h>

SemaphoreHandle_t serialMutex;

// เพิ่ม global buffer สำหรับส่งข้อมูลไปยัง LoRa
char loraPayload[128] = {0};
SemaphoreHandle_t loraPayloadMutex;

// เพิ่ม global counter สำหรับ LoRa packet
volatile uint32_t loraPacketCounter = 0;

// --- เพิ่ม struct สำหรับเก็บข้อมูล GPS ---
struct GPSData
{
    double lat;
    double lon;
    double alt;
    int sat;
    bool valid;
};

GPSData latestGPS = {0, 0, 0, 0, false};
SemaphoreHandle_t gpsMutex;

SemaphoreHandle_t sdMutex;
QueueHandle_t sdLogQueue;

#define SD_LOG_QUEUE_LENGTH 16
#define SD_LOG_LINE_MAXLEN 256

// --- เพิ่ม task สำหรับ SD logging ---
static void taskSDLogger(void *)
{
    if (!SD.begin(BUILTIN_SDCARD))
    {
        // Serial.println("SD card mount failed!");
        vTaskDelete(nullptr);
        return;
    }
    // Serial.println("SD card mounted.");

    // สร้างไฟล์ใหม่ (log_XXX.csv)
    char filename[32];
    int idx = 0;
    File logFile;
    do
    {
        snprintf(filename, sizeof(filename), "/log_%03d.csv", idx++);
    } while (SD.exists(filename) && idx < 1000);
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile)
    {
        // Serial.println("Failed to create log file!");
        vTaskDelete(nullptr);
        return;
    }
    // Serial.print("Logging to: "); Serial.println(filename);

    // เขียน header
    logFile.println("time,packet,temperature,pressure,humidity,altitude,accX,accY,accZ,roll,pitch,lat,lon,gps_alt,sat");
    logFile.flush();

    char logLine[SD_LOG_LINE_MAXLEN];

    while (true)
    {
        // รอรับข้อมูลจาก queue
        if (xQueueReceive(sdLogQueue, &logLine, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            if (sdMutex)
                xSemaphoreTake(sdMutex, portMAX_DELAY);
            logFile.println(logLine);
            logFile.flush();
            if (sdMutex)
                xSemaphoreGive(sdMutex);
        }
        taskYIELD();
    }
}

// --- เพิ่ม helper function สำหรับ push log line ---
void queueSDLogLine(const char *line)
{
    if (sdLogQueue)
    {
        char buf[SD_LOG_LINE_MAXLEN];
        strncpy(buf, line, SD_LOG_LINE_MAXLEN - 1);
        buf[SD_LOG_LINE_MAXLEN - 1] = '\0';
        xQueueSend(sdLogQueue, buf, 0);
    }
}

// --- Detect Launch Variables (global) ---
float filteredAltitude = 0;
const float alpha_alt = 0.3;
unsigned long tnow = 0, tread = 0;
float r0 = 0, r1 = 0, trend = 0;
int checklunch = 0;
static bool firstRunDetect = true;

// --- Relay Pin ---
const int relayPin = 22;

// --- เพิ่ม task สำหรับตรวจสอบการชนกันของ Serial (optional) ---
static void taskSerialWatcher(void *)
{
    while (true)
    {
        // ตรวจสอบการชนกันของ Serial
        if (serialMutex)
        {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            if (Serial.available())
            {
                // อ่านข้อมูลจาก Serial และส่งต่อไปยัง Serial อื่นๆ ถ้ามี
                int c = Serial.read();
                Serial.write(c);
            }
            xSemaphoreGive(serialMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void taskBME280(void *)
{
    // BME280 config
#define SDA_PIN 17
#define SCL_PIN 16
    Adafruit_BME280 bme;
    const int NUM_SAMPLES = 100; // กลับมาใช้ 100 sample
    float referencePressure = 0;
    // float filteredAltitude = 0; // <-- ย้ายไป global
    // const float alpha = 0.2; // <-- ใช้ alpha_alt สำหรับ detect launch
    static bool firstRun = true; // <-- ใช้ firstRunDetect สำหรับ detect launch

    // เพิ่ม mutex สำหรับ Wire1 (I2C ของ BME280) เพื่อป้องกันการชนกับ LoRa/SPI
    static SemaphoreHandle_t i2cMutex = nullptr;
    if (!i2cMutex)
    {
        i2cMutex = xSemaphoreCreateMutex();
    }

    // Init BME280
    Wire1.begin();
    // เพิ่ม retry + timeout ในการ init BME280
    int bmeInitTries = 0;
    while (!bme.begin(0x76, &Wire1))
    {
        // Serial.println("BME280 init failed! Retrying...");
        bmeInitTries++;
        if (bmeInitTries > 10)
        {
            // Serial.println("BME280 not found after 10 tries, using placeholder values.");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Calibrate referencePressure (BME280)
    float sumPressure = 0;
    int validSamples = 0;
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        float p = bme.readPressure() / 100.0F;
        if (!isnan(p) && p > 0.1)
        {
            sumPressure += p;
            validSamples++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (validSamples > 0)
    {
        referencePressure = sumPressure / validSamples;
    }
    else
    {
        referencePressure = 1013.25; // fallback to standard pressure
    }

    // Serial.print("Reference Pressure (average of ");
    // Serial.print(NUM_SAMPLES);
    // Serial.print(" samples) = ");
    // Serial.print(referencePressure);
    // Serial.println(" hPa");

    while (true)
    {
        // Lock I2C bus ก่อนอ่าน sensor
        if (i2cMutex)
            xSemaphoreTake(i2cMutex, portMAX_DELAY);

        float temperature = bme.readTemperature();
        float pressure = bme.readPressure() / 100.0F;
        float humidity = bme.readHumidity();

        if (i2cMutex)
            xSemaphoreGive(i2cMutex);

        // ตัด median filter ออก ใช้ค่า altitude ตรงๆ
        float altitude = 44330.0 * (1.0 - pow(pressure / referencePressure, 0.1903));
        if (firstRun && !isnan(altitude))
        {
            filteredAltitude = altitude;
            firstRun = false;
        }
        else if (!isnan(altitude))
        {
            filteredAltitude = alpha_alt * altitude + (1 - alpha_alt) * filteredAltitude;
        }

        // --- Detect Launch Logic ---
        tnow = millis();
        if (tnow > tread)
        {
            r1 = filteredAltitude;
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
            if (checklunch >= 5 && filteredAltitude >= 15)
            {
                Serial.print("Airbag");
                digitalWrite(relayPin, arduino::HIGH); // เปิดรีเลย์
                // สามารถเพิ่ม flag หรือ field ลงใน loraPayload ได้ที่นี่ถ้าต้องการ
            }
            Serial.print(checklunch);
            Serial.println("");
        }

        // เตรียมข้อมูลสำหรับส่ง LoRa (เพิ่ม counter เป็น field แรก)
        if (loraPayloadMutex)
        {
            xSemaphoreTake(loraPayloadMutex, portMAX_DELAY);
            snprintf(loraPayload, sizeof(loraPayload),
                     "%lu,%.2f,%.2f,%.2f,%.2f",
                     loraPacketCounter,
                     isnan(temperature) ? -1.0 : temperature, // เปลี่ยนใน taskLoRa เท่านั้น
                     isnan(pressure) ? -1.0 : pressure,
                     isnan(humidity) ? -1.0 : humidity,
                     isnan(filteredAltitude) ? -1.0 : filteredAltitude);
            xSemaphoreGive(loraPayloadMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
        taskYIELD();
    }
}

static void taskMPU9250(void *)
{
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

    // เพิ่ม mutex สำหรับ Wire (I2C ของ MPU9250) เพื่อป้องกันการชนกับ LoRa/SPI
    static SemaphoreHandle_t i2cMutex = nullptr;
    if (!i2cMutex)
    {
        i2cMutex = xSemaphoreCreateMutex();
    }

    // Init MPU9250
    Wire.begin();
    Wire.setClock(100000);

    // เพิ่ม retry + timeout ในการ init MPU9250
    uint8_t id;
    int mpuInitTries = 0;
    while (mpu.readId(&id) != 0)
    {
        // Serial.println("MPU Initialization failed. Retrying...");
        mpuInitTries++;
        if (mpuInitTries > 10)
        {
            // Serial.println("MPU not found after 10 tries, using placeholder values.");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (mpuInitTries <= 10)
    {
        // Serial.print("MPU Initialization successful! Device ID: ");
        // Serial.println(id, arduino::HEX); // use arduino::HEX
    }

    mpu.beginAccel(ACC_FULL_SCALE_2_G);
    mpu.beginGyro(GYRO_FULL_SCALE_250_DPS);
    mpu.beginMag(MAG_MODE_CONTINUOUS_8HZ);

    // Calculate IMU error (MPU9250)
    c = 0;
    int validAcc = 0;
    while (c < 200)
    {
        mpu.accelUpdate();
        mpu.gyroUpdate();
        AccX = mpu.accelX();
        AccY = mpu.accelY();
        AccZ = mpu.accelZ();
        if (!isnan(AccX) && !isnan(AccY) && !isnan(AccZ))
        {
            AccErrorX += AccX;
            AccErrorY += AccY;
            AccErrorZ += AccZ;
            validAcc++;
        }
        c++;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (validAcc > 0)
    {
        AccErrorX /= validAcc;
        AccErrorY /= validAcc;
        AccErrorZ /= validAcc;
    }
    else
    {
        AccErrorX = AccErrorY = AccErrorZ = 0;
    }
    c = 0;
    int validGyro = 0;
    while (c < 2000)
    {
        mpu.gyroUpdate();
        GyroX = mpu.gyroX();
        GyroY = mpu.gyroY();
        if (!isnan(GyroX) && !isnan(GyroY))
        {
            GyroErrorX += GyroX;
            GyroErrorY += GyroY;
            validGyro++;
        }
        c++;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (validGyro > 0)
    {
        GyroErrorX /= validGyro;
        GyroErrorY /= validGyro;
    }
    else
    {
        GyroErrorX = GyroErrorY = 0;
    }
    // Serial.print("AccErrorX: "); Serial.println(AccErrorX);
    // Serial.print("AccErrorY: "); Serial.println(AccErrorY);
    // Serial.print("AccErrorZ: "); Serial.println(AccErrorZ);
    // Serial.print("GyroErrorX: "); Serial.println(GyroErrorX);
    // Serial.print("GyroErrorY: "); Serial.println(GyroErrorY);

    // อ่านค่าเริ่มต้นเพื่อเซ็ต Kalman filter
    mpu.accelUpdate();
    AccX = mpu.accelX() - AccErrorX;
    AccY = mpu.accelY() - AccErrorY;
    AccZ = mpu.accelZ() - AccErrorZ;

    float roll = atan2(AccY, AccZ) * 180.0 / PI;
    float pitch = atan2(AccX, AccZ) * 180.0 / PI;

    kalmanX.setAngle(roll);
    kalmanY.setAngle(pitch);

    previousTime = millis();

    while (true)
    {
        // Lock I2C bus ก่อนอ่าน sensor
        if (i2cMutex)
            xSemaphoreTake(i2cMutex, portMAX_DELAY);

        currentTime = millis(); // ป้องกัน warning currentTime uninitialized

        float meanKroll = 0, meanKpitch = 0;
        float lastAccX = 0, lastAccY = 0, lastAccZ = 0, lastKroll = 0, lastKpitch = 0;
        const int AVG_LOOP = 2; // ลดจำนวนรอบเฉลี่ยเหลือ 2 (จาก 3)
        for (int i = 0; i < AVG_LOOP; i++)
        {
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

            float roll = (!isnan(AccY) && !isnan(AccZ)) ? atan2(AccY, AccZ) * 180.0 / PI : 0;
            float pitch = (!isnan(AccX) && !isnan(AccZ)) ? atan2(AccX, AccZ) * 180.0 / PI : 0;

            kalAngleX = kalmanX.getAngle(roll, GyroX, dt);
            kalAngleY = kalmanY.getAngle(pitch, GyroY, dt);

            meanKroll += kalAngleX;
            meanKpitch += kalAngleY;

            // เก็บค่ารอบสุดท้ายเท่านั้น
            lastAccX = AccX;
            lastAccY = AccY;
            lastAccZ = AccZ;
            lastKroll = kalAngleX;
            lastKpitch = kalAngleY;

            vTaskDelay(pdMS_TO_TICKS(1)); // ลด delay ใน loop เฉลี่ย
        }
        meanKroll /= (float)AVG_LOOP;
        meanKpitch /= (float)AVG_LOOP;

        if (i2cMutex)
            xSemaphoreGive(i2cMutex);

        // ต่อท้ายข้อมูลตัวเลขสำหรับ LoRa (ข้าม field counter)
        if (loraPayloadMutex)
        {
            xSemaphoreTake(loraPayloadMutex, portMAX_DELAY);
            size_t len = strlen(loraPayload);
            snprintf(loraPayload + len, sizeof(loraPayload) - len,
                     ",%.3f,%.3f,%.3f,%.2f,%.2f",
                     isnan(lastAccX) ? -1.0 : lastAccX,
                     isnan(lastAccY) ? -1.0 : lastAccY,
                     isnan(lastAccZ) ? -1.0 : lastAccZ,
                     isnan(lastKroll) ? -1.0 : lastKroll,
                     isnan(lastKpitch) ? -1.0 : lastKpitch);
            xSemaphoreGive(loraPayloadMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // ให้เท่ากับ taskBME280
        taskYIELD();
    }
}

static void taskGPS(void *)
{
    GPS_SERIAL.begin(9600); // GPS module baudrate (ปกติ 9600)
    // Serial.println("GPS Ready");

    while (true)
    {
        while (GPS_SERIAL.available() > 0)
        {
            gps.encode(GPS_SERIAL.read());
        }

        // อัปเดตค่า GPS ลง struct
        if (gpsMutex)
        {
            xSemaphoreTake(gpsMutex, portMAX_DELAY);
            if (gps.location.isValid() && gps.location.isUpdated())
            {
                latestGPS.lat = gps.location.lat();
                latestGPS.lon = gps.location.lng();
                latestGPS.alt = gps.altitude.meters();
                latestGPS.sat = gps.satellites.value();
                latestGPS.valid = true;
            }
            else
            {
                latestGPS.valid = false;
            }
            xSemaphoreGive(gpsMutex);
        }

        // Print ข้อมูล GPS
        /*
            if (serialMutex) {
                xSemaphoreTake(serialMutex, portMAX_DELAY);
                if (gps.location.isUpdated()) {
                    Serial.print("Lat: ");
                    Serial.print(gps.location.lat(), 6);
                    Serial.print("  Lon: ");
                    Serial.print(gps.location.lng(), 6);
                    Serial.print("  Alt: ");
                    Serial.print(gps.altitude.meters());
                    Serial.print(" m  Sat: ");
                    Serial.println(gps.satellites.value());
                } else {
                    Serial.print("Lat: -");
                    Serial.print("  Lon: -");
                    Serial.print("  Alt: - m  Sat: -");
                    Serial.println();
                }
                xSemaphoreGive(serialMutex);
            }
            */
        vTaskDelay(pdMS_TO_TICKS(200));
        taskYIELD();
    }
}

// --- LoRa custom config ---
uint8_t sync_word = 0x12;
int8_t power = 10;
int problemle_length = 8;

static void taskLoRa(void *)
{
    // ใช้ arduino::OUTPUT, arduino::HIGH, arduino::LOW
    pinMode(LORA1_CS, arduino::OUTPUT);
    pinMode(LORA2_CS, arduino::OUTPUT);
    // ลบ pinMode LED
    // pinMode(LORA1_LED, arduino::OUTPUT);
    // pinMode(LORA2_LED, arduino::OUTPUT);

    digitalWrite(LORA1_CS, arduino::HIGH);
    digitalWrite(LORA2_CS, arduino::HIGH);

    spiLoRa.begin();
    // spiLoRa.setClock(1000000); // <-- ลบหรือคอมเมนต์บรรทัดนี้ออก เพราะไม่มีใน SPIClass

    // Serial.println("Starting LoRa1...");
    digitalWrite(LORA1_CS, arduino::LOW); // เปิด CS สำหรับ LoRa1
    LoRa.setSPI(SPI1);
    LoRa.setSPIFrequency(1000000); // ลดความถี่ SPI1 เหลือ 1 MHz
    LoRa.setPins(LORA1_CS, static_cast<IRQ_NUMBER_t>(digitalPinToInterrupt(LORA1_DIO0)));
    if (!LoRa.begin(921775000))
    {
        // Serial.println("LoRa1 failed!");
    }
    else
    {
        // Serial.println("LoRa1 ready");
        LoRa.setTxPower(power); // ใช้ค่าจากตัวแปร
        LoRa.setSyncWord(sync_word); // ตั้งค่า sync word
    }
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    // Serial.println("LoRa1 configured with SF9 BW125 CR 4/5");
    digitalWrite(LORA1_CS, arduino::HIGH);

    // Serial.println("Starting LoRa2...");
    digitalWrite(LORA2_CS, arduino::LOW);
    LoRa.setSPI(SPI1);
    LoRa.setSPIFrequency(1000000);
    LoRa.setPins(LORA2_CS, static_cast<IRQ_NUMBER_t>(digitalPinToInterrupt(LORA2_DIO0)));
    if (!LoRa.begin(917E6))
    {
        // Serial.println("LoRa2 failed!");
    }
    else
    {
        // Serial.println("LoRa2 ready");
        LoRa.setTxPower(power); // ใช้ค่าจากตัวแปร
        LoRa.setSyncWord(sync_word); // ตั้งค่า sync word
    }
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    // Serial.println("LoRa2 configured with SF9 BW125 CR 4/5");
    digitalWrite(LORA2_CS, arduino::HIGH);

    int counter = 0;
    // เพิ่ม mutex สำหรับ SPI1 (LoRa) เพื่อป้องกันชนกับ I2C
    static SemaphoreHandle_t spiMutex = nullptr;
    if (!spiMutex)
    {
        spiMutex = xSemaphoreCreateMutex();
    }

    while (true)
    {
        if (spiMutex)
            xSemaphoreTake(spiMutex, portMAX_DELAY);

        // เตรียม payload สำหรับส่ง
        char payloadToSend[192];
        if (loraPayloadMutex)
        {
            xSemaphoreTake(loraPayloadMutex, portMAX_DELAY);
            strncpy(payloadToSend, loraPayload, sizeof(payloadToSend));
            payloadToSend[sizeof(payloadToSend) - 1] = '\0';
            xSemaphoreGive(loraPayloadMutex);
        }
        else
        {
            strcpy(payloadToSend, "0,0,0,0,0,0,0,0,0");
        }

        // --- ตรวจสอบและเติม field ให้ครบ 10 ค่า (counter, temp, pressure, humidity, altitude, accX, accY, accZ, roll, pitch) ---
        int commaCount = 0;
        for (size_t i = 0; payloadToSend[i] != '\0'; ++i)
        {
            if (payloadToSend[i] == ',')
                ++commaCount;
        }
        // ต้องมี 9 คอมม่า (10 field) ก่อนต่อ GPS
        while (commaCount < 9)
        {
            size_t len = strlen(payloadToSend);
            if (len < sizeof(payloadToSend) - 3)
            { // reserve space for ",-"
                strcat(payloadToSend, ",-");
                ++commaCount;
            }
            else
            {
                break;
            }
        }

        // --- ต่อข้อมูล GPS ---
        char gpsPart[64];
        if (gpsMutex)
        {
            xSemaphoreTake(gpsMutex, portMAX_DELAY);
            if (latestGPS.valid)
            {
                snprintf(gpsPart, sizeof(gpsPart), ",%.6f,%.6f,%.2f,%d",
                         latestGPS.lat, latestGPS.lon, latestGPS.alt, latestGPS.sat);
            }
            else
            {
                snprintf(gpsPart, sizeof(gpsPart), ",-,,-,-");
            }
            xSemaphoreGive(gpsMutex);
        }
        else
        {
            snprintf(gpsPart, sizeof(gpsPart), ",-,,-,-");
        }
        strncat(payloadToSend, gpsPart, sizeof(payloadToSend) - strlen(payloadToSend) - 1);

        // ส่ง LoRa1
        digitalWrite(LORA2_CS, arduino::HIGH);
        digitalWrite(LORA1_CS, arduino::LOW);

        LoRa.setSPI(SPI1);
        LoRa.setPins(LORA1_CS, static_cast<IRQ_NUMBER_t>(digitalPinToInterrupt(LORA1_DIO0)));
        LoRa.begin(921775000); // เปลี่ยนจาก 915E6 เป็น 921775000
        LoRa.setTxPower(2);
        LoRa.setSpreadingFactor(9);
        LoRa.setSignalBandwidth(125E3);
        LoRa.setCodingRate4(5);

        LoRa.beginPacket();
        LoRa.print(payloadToSend);
        LoRa.endPacket(true);

        digitalWrite(LORA1_CS, arduino::HIGH);

        // ส่ง LoRa2
        vTaskDelay(pdMS_TO_TICKS(200));

        digitalWrite(LORA1_CS, arduino::HIGH);
        digitalWrite(LORA2_CS, arduino::LOW);

        LoRa.setSPI(SPI1);
        LoRa.setPins(LORA2_CS, static_cast<IRQ_NUMBER_t>(digitalPinToInterrupt(LORA2_DIO0)));
        LoRa.begin(917E6);
        LoRa.setTxPower(2);
        LoRa.setSpreadingFactor(9);
        LoRa.setSignalBandwidth(125E3);
        LoRa.setCodingRate4(5);

        LoRa.beginPacket();
        LoRa.print(payloadToSend);
        LoRa.endPacket(true);

        digitalWrite(LORA2_CS, arduino::HIGH);

        if (spiMutex)
            xSemaphoreGive(spiMutex);

        // Serial print หลังส่งสำเร็จ
        /*
            if (serialMutex) {
                xSemaphoreTake(serialMutex, portMAX_DELAY);
                Serial.print("[LoRa] Sent: ");
                Serial.println(payloadToSend);
                xSemaphoreGive(serialMutex);
            }
            */

        // เพิ่มการนับ packet หลังส่งเสร็จ
        loraPacketCounter++;

        counter++;
        // --- เพิ่มการ log ลง SD card ---
        // เตรียม log line (timestamp, payload, gps)
        char logLine[SD_LOG_LINE_MAXLEN];
        unsigned long now = millis();
        snprintf(logLine, sizeof(logLine), "%lu,%s", now, payloadToSend);
        queueSDLogLine(logLine);

        vTaskDelay(pdMS_TO_TICKS(300));
        taskYIELD();
    }
}

FLASHMEM __attribute__((noinline)) void setup()
{
    Serial.begin(0);
    delay(2'000);

    // ตั้งค่าขา pin เป็น output สำหรับรีเลย์
    pinMode(relayPin, arduino::OUTPUT);

    serialMutex = xSemaphoreCreateMutex();
    loraPayloadMutex = xSemaphoreCreateMutex();
    gpsMutex = xSemaphoreCreateMutex();
    sdMutex = xSemaphoreCreateMutex();
    sdLogQueue = xQueueCreate(SD_LOG_QUEUE_LENGTH, SD_LOG_LINE_MAXLEN);

    if (CrashReport)
    {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

    // ให้ BME280 priority สูงสุด (4), MPU9250 (3), GPS (2), LoRa (1), SDLogger (2)
    xTaskCreate(taskBME280, "taskBME280", 1024, nullptr, 4, nullptr);
    xTaskCreate(taskMPU9250, "taskMPU9250", 1024, nullptr, 3, nullptr);
    xTaskCreate(taskGPS, "taskGPS", 1024, nullptr, 2, nullptr);
    xTaskCreate(taskLoRa, "taskLoRa", 1024, nullptr, 1, nullptr);
    xTaskCreate(taskSDLogger, "taskSDLogger", 2048, nullptr, 2, nullptr);

    Serial.println("setup(): starting scheduler...");
    Serial.flush();

    vTaskStartScheduler();
}

void loop() {}
