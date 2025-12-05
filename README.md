

<p align="center">
  <img src="https://github.com/user-attachments/assets/5777ba83-79eb-4097-9217-79bb79c130ec" width="30%" alt="ESPot logo">
</p>

# ESPot 🌱  
**ESP32-C3 based smart pot demo (ESP-IDF + Home Assistant)**

ESPot is a small demo project showing how to make a smart pot using an **ESP32-C3**, the **ESP-IDF** framework and **Home Assistant**.

It’s meant both as a fun hardware project and as a learning example for people starting with ESP-IDF and Home Assistant integration.

---

## Features

- 🧠 **ESP-IDF firmware**  
  - Written in C using the official ESP-IDF build system (CMake + `idf.py`)
  - Shows a typical structure: `main/`, components, configuration, logging, etc.

- 🌿 **Smart pot capabilities**
  - Reads singals like soil moisture or light  
  - Controls actuators such as a water pump grow LEDs  
  - Basic automation logic in firmware (e.g. “water if soil is dry”)

- 🏠 **Home Assistant integration**
  - Connects to Wi-Fi, publishes sensor values and gets commands from Home Assistant over MQTT  
  - **TBD** Will rpovide detailed description how to connect to a Home Assistant instance and what is needed to be set up beforehand
  
- 🧩 **3D-printable enclosure**
  - `STL/` contains 3D models for the pot and mechanical parts  
  - Printed enclosure is sized for ESP32-C3 board + sensors + pump
  - **TBD**: Detailed list of components needed

- 🎨 **Graphics & branding**
  - `graphics/` contains logo and visuals used for presentations / documentation

- ⚙️ **TBD: make your own**
  - **TBD** Complete list of hardware needed
  - **TBD** Instructions for build, flash and HA integhration
 
---

## Project structure

```text
espot/
├─ FW/          # ESP-IDF firmware for ESP32-C3
│  └─ main/     # Application entry point and main logic
│     └─ ...    # Source files, components, Home Assistant integration, etc.
├─ STL/         # 3D models for the pot and mechanical parts
├─ graphics/    # Logos, renders, presentation graphics
└─ README.md    # This file
