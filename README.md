# Smart Home Automation System using ESP and Firebase

## Overview
This project implements a smart home automation system that enables control and monitoring of multiple electrical appliances using a combination of embedded hardware, sensors, and cloud-based communication. The system is designed to support both local physical control and remote control through a mobile or web application.

An ESP-based microcontroller acts as the central controller, interfacing with touch sensors, environmental sensors, and relay modules, while Firebase is used as the cloud backend for real-time synchronization and remote access.

---

## Key Features
- Control of **four electrical appliances** using relay modules  
- **Capacitive touchpad-based local control** for manual operation  
- **Cloud-based remote control** using Firebase real-time database  
- **Two-way synchronization** between physical switches and cloud state  
- **Automatic control logic** using environmental sensors:
  - LDR for light-based automation
  - PIR for motion detection
- **Motor control logic** integrated for appliance actuation
- Designed with scalability and modularity in mind

---

## System Architecture
The system follows a hybrid control architecture:
- **Local Control Layer**: Touch sensors connected to the ESP microcontroller allow direct user interaction.
- **Sensor Layer**: LDR and PIR sensors provide environmental inputs for automation logic.
- **Control Layer**: Relay modules and motor driver logic control physical appliances.
- **Cloud Layer**: Firebase enables real-time remote control and state synchronization.

---

## Hardware Components
- ESP-based microcontroller (ESP32 / ESP8266)
- Capacitive touch sensors
- Relay module (4-channel)
- PIR motion sensor
- LDR (Light Dependent Resistor)
- Power supply and protection circuitry

---

## Software Components
- Embedded firmware developed using **ESP-IDF**
- Firebase Real-Time Database for cloud communication
- HTTP-based communication for data exchange
- State management logic to prevent desynchronization between local and remote control

---

## Functional Highlights
- Appliances can be switched ON/OFF using either touch input or cloud commands
- Sensor-driven automation operates independently of manual control
- Cloud updates reflect real-time appliance status
- Logic implemented to avoid false triggering and unintended relay toggling

---

## Applications
- Smart home and building automation
- Energy-efficient appliance control
- Contactless switching systems
- IoT-based remote monitoring and control

---

## Project Status
The system is under active development, with ongoing enhancements to control logic, sensor integration, and system robustness.

---

## Author
Harshitha  
Electronics and Communication Engineering
