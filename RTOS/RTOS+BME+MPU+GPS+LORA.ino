/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2020-2024 Timo Sandmann
 * ผมใช้ FreeRTOS ของคนนี้เป็น reference แล้วค่อยๆเขียนเลาะไปมันจะยมั่วๆหน่อย
 */


/**
 * @file    main.cpp
 * @brief   FreeRTOS example for Teensy boards
 * @author  Timo Sandmann
 * @date    17.05.2020
 */

#include "arduino_freertos.h"
#include "avr/pgmspace.h"
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
#include <FreeRTOS.h>
#include <semphr.h>

#define GPS_SERIAL Serial2
SPIClass &spiLoRa = SPI1;
#define LORA1_CS 3
#define LORA1_DIO0 4
#define LORA2_CS 9
#define LORA2_DIO0 0

TinyGPSPlus gps;
SemaphoreHandle_t serialMutex, loraPayloadMutex, gpsMutex, sdMutex;
QueueHandle_t sdLogQueue;
char loraPayload[128] = {0};
volatile uint32_t loraPacketCounter = 0;

struct GPSData { double lat, lon, alt; int sat; bool valid; } latestGPS = {0,0,0,0,false};

#define SD_LOG_QUEUE_LENGTH 16
#define SD_LOG_LINE_MAXLEN 256

static void taskSDLogger(void*) {
    if (!SD.begin(BUILTIN_SDCARD)) vTaskDelete(nullptr);
    char filename[32]; int idx = 0;
    do snprintf(filename, sizeof(filename), "/log_%03d.csv", idx++);
    while (SD.exists(filename) && idx < 1000);
    File logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) vTaskDelete(nullptr);
    logFile.println("time,packet,temperature,pressure,humidity,altitude,accX,accY,accZ,roll,pitch,lat,lon,gps_alt,sat");
    logFile.flush();
    char logLine[SD_LOG_LINE_MAXLEN];
    while (true) {
        if (xQueueReceive(sdLogQueue, &logLine, pdMS_TO_TICKS(1000)) == pdPASS) {
            if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
            logFile.println(logLine); logFile.flush();
            if (sdMutex) xSemaphoreGive(sdMutex);
        }
        taskYIELD();
    }
}

void queueSDLogLine(const char* line) {
    if (sdLogQueue) {
        char buf[SD_LOG_LINE_MAXLEN];
        strncpy(buf, line, SD_LOG_LINE_MAXLEN-1);
        buf[SD_LOG_LINE_MAXLEN-1] = '\0';
        xQueueSend(sdLogQueue, buf, 0);
    }
}

static void taskBME280(void*) {
    #define SDA_PIN 17
    #define SCL_PIN 16
    Adafruit_BME280 bme;
    const int NUM_SAMPLES = 100;
    float referencePressure = 0, filteredAltitude = 0, alpha = 0.2;
    static bool firstRun = true;
    static SemaphoreHandle_t i2cMutex = nullptr;
    if (!i2cMutex) i2cMutex = xSemaphoreCreateMutex();
    Wire1.begin();
    int bmeInitTries = 0;
    while (!bme.begin(0x76, &Wire1)) { if (++bmeInitTries > 10) break; vTaskDelay(pdMS_TO_TICKS(500)); }
    float sumPressure = 0; int validSamples = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        float p = bme.readPressure() / 100.0F;
        if (!isnan(p) && p > 0.1) { sumPressure += p; validSamples++; }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    referencePressure = validSamples > 0 ? sumPressure / validSamples : 1013.25;
    while (true) {
        if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
        float temperature = bme.readTemperature(), pressure = bme.readPressure() / 100.0F, humidity = bme.readHumidity();
        if (i2cMutex) xSemaphoreGive(i2cMutex);
        float altitude = 44330.0 * (1.0 - pow(pressure / referencePressure, 0.1903));
        if (firstRun && !isnan(altitude)) { filteredAltitude = altitude; firstRun = false; }
        else if (!isnan(altitude)) filteredAltitude = alpha * altitude + (1 - alpha) * filteredAltitude;
        if (loraPayloadMutex) {
            xSemaphoreTake(loraPayloadMutex, portMAX_DELAY);
            snprintf(loraPayload, sizeof(loraPayload), "%lu,%.2f,%.2f,%.2f,%.2f", loraPacketCounter,
                isnan(temperature) ? -999.0 : temperature,
                isnan(pressure) ? -999.0 : pressure,
                isnan(humidity) ? -999.0 : humidity,
                isnan(filteredAltitude) ? -999.0 : filteredAltitude);
            xSemaphoreGive(loraPayloadMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        taskYIELD();
    }
}

static void taskMPU9250(void*) {
    MPU9250_asukiaaa mpu;
    Kalman kalmanX, kalmanY;
    float AccX, AccY, AccZ, GyroX, GyroY, kalAngleX, kalAngleY, dt, currentTime, previousTime;
    float AccErrorX = 0, AccErrorY = 0, AccErrorZ = 0, GyroErrorX = 0, GyroErrorY = 0;
    int c = 0;
    static SemaphoreHandle_t i2cMutex = nullptr;
    if (!i2cMutex) i2cMutex = xSemaphoreCreateMutex();
    Wire.begin(); Wire.setClock(100000);
    uint8_t id; int mpuInitTries = 0;
    while (mpu.readId(&id) != 0) { if (++mpuInitTries > 10) break; vTaskDelay(pdMS_TO_TICKS(500)); }
    mpu.beginAccel(ACC_FULL_SCALE_2_G); mpu.beginGyro(GYRO_FULL_SCALE_250_DPS); mpu.beginMag(MAG_MODE_CONTINUOUS_8HZ);
    c = 0; int validAcc = 0;
    while (c < 200) { mpu.accelUpdate(); mpu.gyroUpdate(); AccX = mpu.accelX(); AccY = mpu.accelY(); AccZ = mpu.accelZ();
        if (!isnan(AccX) && !isnan(AccY) && !isnan(AccZ)) { AccErrorX += AccX; AccErrorY += AccY; AccErrorZ += AccZ; validAcc++; }
        c++; vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (validAcc > 0) { AccErrorX /= validAcc; AccErrorY /= validAcc; AccErrorZ /= validAcc; }
    c = 0; int validGyro = 0;
    while (c < 2000) { mpu.gyroUpdate(); GyroX = mpu.gyroX(); GyroY = mpu.gyroY();
        if (!isnan(GyroX) && !isnan(GyroY)) { GyroErrorX += GyroX; GyroErrorY += GyroY; validGyro++; }
        c++; vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (validGyro > 0) { GyroErrorX /= validGyro; GyroErrorY /= validGyro; }
    mpu.accelUpdate();
    AccX = mpu.accelX() - AccErrorX; AccY = mpu.accelY() - AccErrorY; AccZ = mpu.accelZ() - AccErrorZ;
    float roll = atan2(AccY, AccZ) * 180.0 / PI, pitch = atan2(AccX, AccZ) * 180.0 / PI;
    kalmanX.setAngle(roll); kalmanY.setAngle(pitch);
    previousTime = millis();
    while (true) {
        if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
        currentTime = millis();
        float meanKroll = 0, meanKpitch = 0;
        for (int i = 0; i < 10; i++) {
            mpu.accelUpdate(); mpu.gyroUpdate();
            previousTime = currentTime; currentTime = millis();
            dt = (currentTime - previousTime) / 1000.0;
            AccX = mpu.accelX() - AccErrorX; AccY = mpu.accelY() - AccErrorY; AccZ = mpu.accelZ() - AccErrorZ;
            GyroX = mpu.gyroX() - GyroErrorX; GyroY = mpu.gyroY() - GyroErrorY;
            float roll = (!isnan(AccY) && !isnan(AccZ)) ? atan2(AccY, AccZ) * 180.0 / PI : 0;
            float pitch = (!isnan(AccX) && !isnan(AccZ)) ? atan2(AccX, AccZ) * 180.0 / PI : 0;
            kalAngleX = kalmanX.getAngle(roll, GyroX, dt);
            kalAngleY = kalmanY.getAngle(pitch, GyroY, dt);
            meanKroll += kalAngleX; meanKpitch += kalAngleY;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        meanKroll /= 10.0; meanKpitch /= 10.0;
        if (i2cMutex) xSemaphoreGive(i2cMutex);
        if (loraPayloadMutex) {
            xSemaphoreTake(loraPayloadMutex, portMAX_DELAY);
            size_t len = strlen(loraPayload);
            snprintf(loraPayload + len, sizeof(loraPayload) - len, ",%.3f,%.3f,%.3f,%.2f,%.2f",
                isnan(AccX) ? -999.0 : AccX, isnan(AccY) ? -999.0 : AccY, isnan(AccZ) ? -999.0 : AccZ,
                isnan(meanKroll) ? -999.0 : meanKroll, isnan(meanKpitch) ? -999.0 : meanKpitch);
            xSemaphoreGive(loraPayloadMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        taskYIELD();
    }
}

static void taskGPS(void*) {
    GPS_SERIAL.begin(9600);
    while (true) {
        while (GPS_SERIAL.available() > 0) gps.encode(GPS_SERIAL.read());
        if (gpsMutex) {
            xSemaphoreTake(gpsMutex, portMAX_DELAY);
            if (gps.location.isValid() && gps.location.isUpdated()) {
                latestGPS.lat = gps.location.lat(); latestGPS.lon = gps.location.lng();
                latestGPS.alt = gps.altitude.meters(); latestGPS.sat = gps.satellites.value(); latestGPS.valid = true;
            } else latestGPS.valid = false;
            xSemaphoreGive(gpsMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
        taskYIELD();
    }
}

static void taskLoRa(void*) {
    pinMode(LORA1_CS, arduino::OUTPUT); pinMode(LORA2_CS, arduino::OUTPUT);
    digitalWrite(LORA1_CS, arduino::HIGH); digitalWrite(LORA2_CS, arduino::HIGH);
    spiLoRa.begin();
    digitalWrite(LORA1_CS, arduino::LOW);
    LoRa.setSPI(SPI1); LoRa.setSPIFrequency(1000000);
    LoRa.setPins(LORA1_CS, static_cast<IRQ_NUMBER_t>(digitalPinToInterrupt(LORA1_DIO0)));
    LoRa.begin(915E6); LoRa.setTxPower(2); LoRa.setSpreadingFactor(9); LoRa.setSignalBandwidth(125E3); LoRa.setCodingRate4(5);
    digitalWrite(LORA1_CS, arduino::HIGH);
    digitalWrite(LORA2_CS, arduino::LOW);
    LoRa.setSPI(SPI1); LoRa.setSPIFrequency(1000000);
    LoRa.setPins(LORA2_CS, static_cast<IRQ_NUMBER_t>(digitalPinToInterrupt(LORA2_DIO0)));
    LoRa.begin(917E6); LoRa.setTxPower(2); LoRa.setSpreadingFactor(9); LoRa.setSignalBandwidth(125E3); LoRa.setCodingRate4(5);
    digitalWrite(LORA2_CS, arduino::HIGH);
    static SemaphoreHandle_t spiMutex = nullptr;
    if (!spiMutex) spiMutex = xSemaphoreCreateMutex();
    while (true) {
        if (spiMutex) xSemaphoreTake(spiMutex, portMAX_DELAY);
        char payloadToSend[192];
        if (loraPayloadMutex) {
            xSemaphoreTake(loraPayloadMutex, portMAX_DELAY);
            strncpy(payloadToSend, loraPayload, sizeof(payloadToSend));
            payloadToSend[sizeof(payloadToSend)-1] = '\0';
            xSemaphoreGive(loraPayloadMutex);
        } else strcpy(payloadToSend, "0,0,0,0,0,0,0,0,0");
        char gpsPart[64];
        if (gpsMutex) {
            xSemaphoreTake(gpsMutex, portMAX_DELAY);
            if (latestGPS.valid)
                snprintf(gpsPart, sizeof(gpsPart), ",%.6f,%.6f,%.2f,%d", latestGPS.lat, latestGPS.lon, latestGPS.alt, latestGPS.sat);
            else snprintf(gpsPart, sizeof(gpsPart), ",-,,-,-");
            xSemaphoreGive(gpsMutex);
        } else snprintf(gpsPart, sizeof(gpsPart), ",-,,-,-");
        strncat(payloadToSend, gpsPart, sizeof(payloadToSend) - strlen(payloadToSend) - 1);
        digitalWrite(LORA2_CS, arduino::HIGH); digitalWrite(LORA1_CS, arduino::LOW);
        LoRa.setSPI(SPI1); LoRa.setPins(LORA1_CS, static_cast<IRQ_NUMBER_t>(digitalPinToInterrupt(LORA1_DIO0))); LoRa.begin(915E6);
        LoRa.beginPacket(); LoRa.print(payloadToSend); LoRa.endPacket(true);
        digitalWrite(LORA1_CS, arduino::HIGH);
        vTaskDelay(pdMS_TO_TICKS(200));
        digitalWrite(LORA1_CS, arduino::HIGH); digitalWrite(LORA2_CS, arduino::LOW);
        LoRa.setSPI(SPI1); LoRa.setPins(LORA2_CS, static_cast<IRQ_NUMBER_t>(digitalPinToInterrupt(LORA2_DIO0))); LoRa.begin(917E6);
        LoRa.beginPacket(); LoRa.print(payloadToSend); LoRa.endPacket(true);
        digitalWrite(LORA2_CS, arduino::HIGH);
        if (spiMutex) xSemaphoreGive(spiMutex);
        loraPacketCounter++;
        char logLine[SD_LOG_LINE_MAXLEN];
        unsigned long now = millis();
        snprintf(logLine, sizeof(logLine), "%lu,%s", now, payloadToSend);
        queueSDLogLine(logLine);
        vTaskDelay(pdMS_TO_TICKS(300));
        taskYIELD();
    }
}

FLASHMEM __attribute__((noinline)) void setup() {
    Serial.begin(0); delay(2000);
    serialMutex = xSemaphoreCreateMutex();
    loraPayloadMutex = xSemaphoreCreateMutex();
    gpsMutex = xSemaphoreCreateMutex();
    sdMutex = xSemaphoreCreateMutex();
    sdLogQueue = xQueueCreate(SD_LOG_QUEUE_LENGTH, SD_LOG_LINE_MAXLEN);
    if (CrashReport) { Serial.print(CrashReport); Serial.println(); Serial.flush(); }
    Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));
    xTaskCreate(taskBME280, "taskBME280", 1024, nullptr, 4, nullptr);
    xTaskCreate(taskMPU9250, "taskMPU9250", 1024, nullptr, 3, nullptr);
    xTaskCreate(taskGPS, "taskGPS", 1024, nullptr, 2, nullptr);
    xTaskCreate(taskLoRa, "taskLoRa", 1024, nullptr, 1, nullptr);
    xTaskCreate(taskSDLogger, "taskSDLogger", 2048, nullptr, 2, nullptr);
    Serial.println("setup(): starting scheduler..."); Serial.flush();
    vTaskStartScheduler();
}

void loop() {}

