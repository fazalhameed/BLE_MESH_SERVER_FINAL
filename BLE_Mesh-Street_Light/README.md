# 🚦 BLE Mesh Street Light Control (Generic OnOff)

| Supported Targets | ESP32 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 |
|-------------------|-------|----------|----------|----------|-----------|----------|----------|

This project demonstrates a simple **Bluetooth Low Energy (BLE) Mesh** implementation using ESP32 modules to control **street lights**. It consists of a **Generic OnOff Server** (street light node) and a **Generic OnOff Client** (controller node). The server receives mesh messages from the client to toggle an LED, simulating the on/off operation of a street light.

---

## ⚙️ BLE Mesh Node Demo Overview

This BLE Mesh device can be set up as a node with the following features:

- ✅ One element
- ✅ Two SIG models:
  - **Configuration Server model** – Handles AppKey binding, relay settings, TTL, subscriptions, etc.
  - **OnOff Server model** – Enables basic on/off functionality for controlling lights or relays.

---

## 🔧 Scenario Overview

### 1. **Server Node (Street Light)**
- Configured as a **BLE Mesh Generic OnOff Server**
- Provisioned using the **nRF Mesh app**
- Receives On/Off commands via Mesh
- Toggles onboard or external LED accordingly

### 2. **Client Node (Controller)**
- Configured as a **BLE Mesh Generic OnOff Client**
- Sends Generic OnOff Set messages to the server node
- Also provisioned via the **nRF Mesh app**

### 3. **Demo Flow**
1. Server node is powered and enters unprovisioned state
2. Use **nRF Mesh** to provision and bind the server
3. Configure client node and bind it to the same network
4. Toggle LED on server by sending On/Off messages from client

---

## 💡 Example Use Case

This demo mimics a real-world **street lighting system** where:
- Each street light (ESP32 node) is part of a BLE Mesh network
- A controller node or centralized system can **toggle each light remotely**

---

## 🛠️ Features

- BLE Mesh communication using ESP-IDF (v5.x)
- Provisioning via **nRF Mesh App**
- Generic OnOff Server/Client model implementation
- Fast 20ms non-connectable advertisement (optional)
- RGB LED GPIO support for hardware indication
- UART debug logs for testing and diagnostics

---

## 📲 Tools Required

- [x] **ESP32-DevKitC / ESP32**
- [x] **ESP-IDF v5.0 or higher**
- [x] **nRF Mesh App** (Android/iOS)
- [x] UART Serial Monitor (e.g., `idf.py monitor`)

---

