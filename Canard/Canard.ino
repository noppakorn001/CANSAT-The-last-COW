
// ในโค้ดนี้ผมใข้ teensy4.0
// ผมใช้ค่า pitch และ yaw rate จาก MPU9255 เพื่อควบคุม servo canard

#include <Wire.h>
#include <PWMServo.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// === Servo Setup ===
PWMServo servo1;
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;

const int SERVO_PIN1 = 23;
const int SERVO_PIN2 = 22;
const int SERVO_PIN3 = 7;
const int SERVO_PIN4 = 14;

// === MPU9255 Setup ===
MPU9250_asukiaaa mpu;

#define SDA_PIN 18
#define SCL_PIN 19

// === BME280 Setup ===
#define BME_SDA_PIN 17
#define BME_SCL_PIN 16

Adafruit_BME280 bme;
const int NUM_SAMPLES = 100;
float referencePressure = 0;

// === Filter Variables ===
float filteredPitchRate = 0;
float filteredYawRate = 0; 
const float alpha = 0.2;  

float filteredAltitude = 0;
const float alpha_alt = 0.3;

// --- Launch detection variables ---
unsigned long tnow = 0, tread = 0;
float r0 = 0, r1 = 0, trend = 0;
int checklunch = 0;
bool launchDetected = false;
unsigned long launchTime = 0;

void setup()
{
    Serial.begin(115200);
    delay(500);

    Wire.begin();

    // Setup MPU9255
    mpu.setWire(&Wire);
    mpu.beginAccel();
    mpu.beginGyro();

    // Attach servos
    servo1.attach(SERVO_PIN1);
    servo2.attach(SERVO_PIN2);
    servo3.attach(SERVO_PIN3);
    servo4.attach(SERVO_PIN4);

    // Set initial positions
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    servo4.write(90);

    // --- BME280 setup ---
    Wire1.begin(); // ใช้ Wire1 สำหรับ Teensy 4.1 (SDA1=17, SCL1=16)
    bme.begin(0x76, &Wire1);

   
    float sumPressure = 0;
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        sumPressure += bme.readPressure() / 100.0F; // hPa
        delay(50);
    }
    referencePressure = sumPressure / NUM_SAMPLES;

    Serial.print("Reference Pressure (average of ");
    Serial.print(NUM_SAMPLES);
    Serial.print(" samples) = ");
    Serial.print(referencePressure);
    Serial.println(" hPa");
}

void loop()
{
    mpu.gyroUpdate();

    // --- BME280 Readings ---
    float temperature = bme.readTemperature();
    float pressure = bme.readPressure() / 100.0F;
    float humidity = bme.readHumidity();

    // คำนวณความสูงสัมพัทธ์ (relative altitude)
    float altitude = 44330.0 * (1.0 - pow(pressure / referencePressure, 0.1903));

    // Exponential Moving Average filter สำหรับความสูง
    static bool firstRun = true;
    if (firstRun)
    {
        filteredAltitude = altitude;
        firstRun = false;
    }
    else
    {
        filteredAltitude = alpha_alt * altitude + (1 - alpha_alt) * filteredAltitude;
    }

    // --- Launch detection ---
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
        if (!launchDetected && checklunch >= 5 && filteredAltitude >= 0.7)
        {
            launchDetected = true;
            launchTime = millis();
        }
    }

    // Read angular velocity in deg/s
    float pitchRate = mpu.gyroY();
    float yawRate = mpu.gyroZ();

    // === Low-pass filter ===
    filteredPitchRate = alpha * pitchRate + (1 - alpha) * filteredPitchRate;
    filteredYawRate = alpha * yawRate + (1 - alpha) * filteredYawRate;

    // --- Servo control logic ---
    bool canControlServo = false;
    if (launchDetected && (millis() - launchTime >= 8000))
    {
        canControlServo = true;
    }

    int pos1 = 90, pos2 = 90, pos3 = 90, pos4 = 90;
    if (canControlServo)
    {
        // Simple mapping to servo positions with ±30 degree limit
        pos1 = constrain(90 + constrain(filteredYawRate, -30, 30), 60, 120);   // yaw+
        pos2 = constrain(90 - constrain(filteredYawRate, -30, 30), 60, 120);   // yaw-
        pos3 = constrain(90 + constrain(filteredPitchRate, -30, 30), 60, 120); // pitch+
        pos4 = constrain(90 - constrain(filteredPitchRate, -30, 30), 60, 120); // pitch-
    }

    // Write to servos
    servo1.write(pos1);
    servo2.write(pos2);
    servo3.write(pos3);
    servo4.write(pos4);

    // === Serial Monitor Output ===
    /*
    Serial.print("T=");
    Serial.print(temperature);
    Serial.print(" P=");
    Serial.print(pressure);
    Serial.print(" H=");
    Serial.print(humidity);
    Serial.print(" EMA=");
    */
    Serial.print(filteredAltitude);
    // Serial.print(" | ");
    /*
    Serial.print("PitchRate: ");
    Serial.print(filteredPitchRate, 2);
    Serial.print(" deg/s, ");
    Serial.print("YawRate: ");
    Serial.print(filteredYawRate, 2);
    Serial.print(" deg/s | ");
    Serial.print("Servo Pos [1-4]: ");
    Serial.print(pos1);
    Serial.print(" ");
    Serial.print(pos2);
    Serial.print(" ");
    Serial.print(pos3);
    Serial.print(" ");
    Serial.print(pos4); */
    Serial.print(" | LaunchDetected: ");
    Serial.print(launchDetected);
    Serial.print(" | canControlServo: ");
    Serial.println(canControlServo);

    delay(50);
}