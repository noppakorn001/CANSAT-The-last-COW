# CANSAT - The Last COW 🐄

## 🛰️ Thailand CanSat-Rocket Competition 🚀

- **2024** → _COW is Back_ 🏆 **Rocket Mission Award**  
- **2025** → _The Last COW_ 🪵 **Consolation Award**

---

## 🔧 Hardware Overview

| Component      | Model/Description                      |
|----------------|----------------------------------------|
| **Master MCU** | Teensy 4.1, Teensy 4.0, ESP32          |
| **LoRa**       | RFM95W                                 |
| **Sensors**    | MPU9255, BME280, GY-NEO-8M (Ublox)     |
| **Storage**    | SD Card Module                         |
| **Actuator**   | SG90 Servo                             |

---

> ⚙️ _Most of the code has been modified for performance optimization._

---

## 📘 Project Note

_In this GitHub repository, I share the code that I have learned and developed over the course of one month.  
Please note that the code still requires significant improvement, especially in terms of logic and a deeper understanding of sensor behavior._

---

<p align="center">
  <img src="https://github.com/noppakorn001/CANSAT-The-last-COW/blob/main/image/overview.png?raw=true" alt="Project Overview" width="1000"/>
</p>

---

## 🖥️ Ground Station Features

- Read data from multiple Serial Ports simultaneously  
- Set baud rate  
- Save data to `.csv` file  
- Plot real-time graphs (requires proper data formatting)  
- Real-time data display interface  
- Automatically adds time stamps in milliseconds  

<p align="center">
  <img src="https://github.com/noppakorn001/CANSAT-The-last-COW/blob/main/Ground-Station/Interface.png?raw=true" alt="Ground Station Interface" width="1000"/>
</p>
