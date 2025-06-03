/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2020-2024 Timo Sandmann
 * ผมใช้ FreeRTOS ของคนนี้เป็น reference แล้วค่อยๆเขียนเลาะไปมันจะยมั่วๆหน่อย
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


static void taskBME280(void*) {
    // BME280 config
    #define SDA_PIN 17
    #define SCL_PIN 16
    Adafruit_BME280 bme;
    const int NUM_SAMPLES = 100;
    float referencePressure = 0;
    float filteredAltitude = 0;
    const float alpha = 0.3;
    static bool firstRun = true;


    Wire1.begin();
    bme.begin(0x76, &Wire1);

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

    while (true) {
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

        Serial.print("T=");
        Serial.print(temperature);
        Serial.print("P=");
        Serial.print(pressure);
        Serial.print("H=");
        Serial.print(humidity);
        Serial.print("EMA=");
        Serial.print(filteredAltitude);
        Serial.println();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


FLASHMEM __attribute__((noinline)) void setup() {
    Serial.begin(0);
    delay(2'000);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

    xTaskCreate(taskBME280, "taskBME280", 512, nullptr, 2, nullptr);

    Serial.println("setup(): starting scheduler...");
    Serial.flush();

    vTaskStartScheduler();
}

void loop() {}