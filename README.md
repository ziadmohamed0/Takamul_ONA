<div align="center">

<img src="docs/images/pkg.jpeg" alt="Takamul iQ16 — Industrial IoT Controller" width="780"/>

<br/>
<br/>

# Takamul iQ16 PLC

### Industrial IoT Controller — تكامل

<br/>

[![Firmware](https://img.shields.io/badge/Firmware-ESP--IDF%20v5-blue?style=for-the-badge&logo=espressif)](https://docs.espressif.com/projects/esp-idf/en/latest/)
[![STM32](https://img.shields.io/badge/STM32-F405RGT6-03234B?style=for-the-badge&logo=stmicroelectronics)](https://www.st.com)
[![Cloud](https://img.shields.io/badge/Cloud-Supabase%20RLS-3ECF8E?style=for-the-badge&logo=supabase)](https://supabase.com)
[![PCB](https://img.shields.io/badge/PCB-KiCad%207-314CB0?style=for-the-badge)](https://www.kicad.org)
[![Protocol](https://img.shields.io/badge/Protocol-Modbus%20RTU-FF6B35?style=for-the-badge)](#)
[![License](https://img.shields.io/badge/License-Proprietary-red?style=for-the-badge)](#)

<br/>

> **Dual-core STM32 + ESP32-S3 · Supabase Multi-Tenant Cloud · Modbus RTU · DIN-Rail Mount**  
> Designed for water treatment, flow control, and process automation in harsh industrial environments.

</div>

---

## Gallery

<table>
  <tr>
    <td align="center" width="50%">
      <img src="docs/images/case.jpeg" alt="iQ16 mounted on DIN rail" width="100%"/>
      <br/><sub><b>iQ16 — DIN-Rail Mounted</b></sub>
    </td>
    <td align="center" width="50%">
      <img src="docs/images/top.jpeg" alt="PCB Top View" width="100%"/>
      <br/><sub><b>PCB — Top View (3D Render)</b></sub>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="docs/images/Back.jpeg" alt="PCB Back View" width="100%"/>
      <br/><sub><b>PCB — Back View (3D Render)</b></sub>
    </td>
    <td align="center" width="50%">
      <img src="docs/images/pcb_Layout.jpeg" alt="PCB Layout" width="100%"/>
      <br/><sub><b>PCB — KiCad Layout (All Copper Layers)</b></sub>
    </td>
  </tr>
</table>

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
- [Firmware](#firmware)
- [Cloud & Connectivity](#cloud--connectivity)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [Supported Instruments](#supported-instruments)

---

## Overview

The **Takamul iQ16** is a compact, DIN-rail mounted industrial PLC built for real-world process automation. It pairs a **STM32F405** real-time controller — handling deterministic Modbus RTU field communication — with an **ESP32-S3** managing Wi-Fi, MQTT, and secure cloud telemetry via **Supabase Row-Level Security**.

Every unit ships ready to deploy: scan the MAC QR code on the box, link it to your account, and the device self-configures with zero manual setup.

**Built for:**
- Water treatment & desalination plants
- Flow metering systems (Endress+Hauser Promag P300)
- Multi-parameter water quality monitoring (TDS, pressure, temperature, differential)
- VFD pump control automation

---

## Key Features

| | Feature | Detail |
|---|---|---|
| ⚙️ | **Dual-Core Processing** | STM32F405RGT6 (real-time control) + ESP32-S3 (connectivity) |
| 📡 | **Field Protocols** | Modbus RTU × 2 ports (RS-485), bridged to cloud via MQTT |
| 🌊 | **Sensor Inputs** | Flow · Pressure · TDS · Temperature · Differential (5 channels) |
| ⚡ | **Actuator Outputs** | VFD Control × 2 (Variable Frequency Drive) |
| 🔋 | **Power Supply** | 12–24V DC wide-range input, onboard LM2596S regulator |
| 📶 | **Connectivity** | Wi-Fi 802.11 b/g/n, external SMA antenna, USB-C programming |
| ☁️ | **Cloud Backend** | Supabase PostgreSQL + RLS — multi-tenant, per-device data isolation |
| 🔐 | **Security** | JWT auth, RLS policies, NVS encrypted credential storage |
| 💡 | **Status Indicators** | Power · Run · Error · Comms · Wi-Fi (5 onboard LEDs) |
| 📦 | **Form Factor** | DIN-rail 35mm TS35, 3D-printable enclosure included |
| 🔌 | **Programming** | ST-LINK V2 (STM32) + USB-C ESP-IDF (ESP32) |
| 📋 | **Zero-Config** | MAC-based QR pairing — no manual IP or credential entry |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Field Instruments                     │
│  [Flow Meter]  [Pressure]  [TDS]  [Temp]  [Diff]       │
└──────────────────────┬──────────────────────────────────┘
                       │ Modbus RTU (RS-485)
              ┌────────▼────────┐
              │   STM32F405     │  ← Real-time deterministic control
              │  Modbus Master  │  ← VFD pump output control
              │  FreeRTOS       │
              └────────┬────────┘
                       │ UART (115200 baud)
              ┌────────▼────────┐
              │    ESP32-S3     │  ← Wi-Fi / MQTT / Cloud
              │  WifiManager    │
              │  SupabaseClient │──────────► Supabase Cloud
              │  MqttManager    │              (PostgreSQL + RLS)
              │  TelemetryMgr   │
              │  WebServer      │──────────► Local Config Portal
              │  NVSManager     │
              └─────────────────┘
```

---

## Hardware

### PCB — `ONA_HARD/`

The iQ16 PCB is a **4-layer board** designed in KiCad 7. The stackup separates power planes from signal layers to ensure clean analog sensor readings alongside noisy RS-485 and switching regulator circuits.

| Spec | Value |
|---|---|
| Layers | 4 (F.Cu · In1.Cu · In2.Cu · B.Cu) |
| MCUs | STM32F405RGT6 + ESP32-S3 module |
| Power | LM2596S buck converter, 220µF/35V + 100µF/50V bulk caps |
| RS-485 | Dual Modbus ports (ModBus 1: A/B, ModBus 2: Y/Z) |
| Programming | USB-C (ESP32) + ST-Link header (STM32) |
| Mounting | 4× M3 corner holes for DIN-rail clip assembly |
| Design tool | KiCad 7 |
| Fabrication | Gerber + drill files ready (JLCPCB / PCBWay) |

### Mechanical Enclosure — `mechanical/cad_v2/`

Full DIN-rail enclosure designed for 3D printing, split into individual printable parts:

`assembly` · `base` · `front_lid` · `top_lid` · `top_connectors` · `bottom_connectors` · `din_clip` · `wifi_module` · `antenna` · `standoff` · `screw_m3x8`

All parts available as both `.stl` (print-ready) and `.obj` (CAD import).

---

## Firmware

### ESP32-S3 — `ONA_Software/main_esp32/` (ESP-IDF v5)

The ESP32 owns all network and cloud operations, acting as a transparent bridge between the STM32 field data and the Supabase backend.

```
Takamul:: namespace
│
├── NVSManager        Flash-backed credential & config storage
├── WifiManager       Auto-connect, reconnect, provisioning fallback
├── SupabaseClient    HTTPS REST client — insert/query with JWT RLS auth
├── TelemetryManager  Periodic sensor data batching & upload
├── MqttManager       Real-time bidirectional command channel
├── UartBridge        UART2 ↔ STM32 framing & protocol (GPIO 16/17)
└── WebServer         Local HTTP config portal for Zero-Config pairing
```

**Configuring credentials** (via `idf.py menuconfig → Takamul Config`):

```
CONFIG_TAKAMUL_SUPABASE_URL   → your Supabase project URL
CONFIG_TAKAMUL_SUPABASE_KEY   → your anon/service role key
```

### STM32F405 — `ONA_Software/modbus_v2/` (STM32CubeIDE)

Runs a deterministic FreeRTOS Modbus RTU master stack over RS-485. Polls all field instruments on configurable scan cycles and pushes structured telemetry frames to the ESP32 over UART.

### Modbus Mapping — `ONA_Software/modbus_mapping_controll/`

STM32 firmware variant with full register mapping and VFD control logic — maps physical Modbus registers from flow meters, TDS analyzers, and pressure transmitters to structured data objects sent upstream.

---

## Cloud & Connectivity

| Layer | Technology | Purpose |
|---|---|---|
| **Database** | Supabase PostgreSQL | Persistent sensor telemetry & device registry |
| **Security** | Row-Level Security (RLS) | Per-device data isolation, multi-tenant safe |
| **Auth** | JWT tokens | Device identity tied to MAC address |
| **Real-time** | Supabase Realtime | Live dashboard subscriptions |
| **Commands** | MQTT pub/sub | Low-latency actuator control (VFD setpoints) |
| **Local config** | HTTP Web Server | Zero-Config Wi-Fi & account pairing |
| **Pairing** | QR code on box | MAC address → account link, no manual entry |

---

## Repository Structure

```
Takamul_ONA/
│
├── ONA_HARD/                        # Hardware design files
│   ├── BOM.xlsx                     # Bill of Materials
│   ├── STM32 & ESP32.pdf            # Dual-core architecture diagram
│   ├── top.jpeg                     # PCB top 3D render
│   ├── Back.jpeg                    # PCB back 3D render
│   └── pcb_Layout.jpeg              # KiCad PCB layout screenshot
│
├── ONA_Software/
│   ├── main_esp32/                  # ESP32-S3 firmware (ESP-IDF v5)
│   │   └── main/
│   │       ├── main.cpp / main.h
│   │       ├── src/
│   │       │   ├── WifiManager.cpp
│   │       │   ├── SupabaseClient.cpp
│   │       │   ├── TelemetryManager.cpp
│   │       │   ├── UartBridge.cpp
│   │       │   ├── WebServer.cpp
│   │       │   ├── MqttManager.cpp
│   │       │   └── NVSManager.cpp
│   │       └── inc/                 # Corresponding headers
│   │
│   ├── stm32_master/                # STM32F103 base firmware (CubeIDE)
│   ├── modbus_v2/                   # STM32F405 Modbus RTU stack
│   ├── modbus_mapping_controll/     # Register mapping & VFD control logic
│   ├── test_db/                     # ESP32 Supabase integration tests
│   └── data_sheets/                 # Instrument Modbus documentation
│       ├── Liquiline CM44x Modbus map
│       └── Proline Promag P300 Modbus map
│
├── mechanical/
│   ├── cad_v2/                      # 3D-printable enclosure (STL + OBJ)
│   │   ├── assembly.stl
│   │   ├── din_clip.stl
│   │   └── ...
│   ├── docs/
│   │   ├── bom_v2.csv
│   │   └── takamul_iq16_drawing_pack.pdf
│   ├── case.jpeg                    # Enclosure render
│   └── pkg.jpeg                     # Product packaging render
│
└── docs/
    └── images/                      # README assets
```

---

## Getting Started

### What's in the Box

- Takamul iQ16 Controller Unit
- Quick Start Guide
- USB-C Programming Cable
- ST-LINK V2 Programmer

### Prerequisites

| Tool | Purpose | Link |
|---|---|---|
| ESP-IDF v5 | ESP32-S3 firmware build | [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/latest/) |
| STM32CubeIDE | STM32 firmware build & flash | [st.com](https://www.st.com/en/development-tools/stm32cubeide.html) |
| KiCad 7 | PCB schematic & layout | [kicad.org](https://www.kicad.org) |
| ST-LINK V2 | STM32 programmer (included) | — |

### Flash ESP32 Firmware

```bash
cd ONA_Software/main_esp32
idf.py set-target esp32s3
idf.py menuconfig        # Takamul Config → set Supabase URL & key
idf.py build flash monitor
```

### Flash STM32 Firmware

```bash
# Open in STM32CubeIDE
File → Open Projects from File System → ONA_Software/modbus_v2/

# Build & flash via ST-LINK V2
Project → Build (Ctrl+B)
Run → Debug (connects via ST-LINK header)
```

### Open PCB in KiCad

```bash
kicad ONA_HARD/
# Open ONA.kicad_pro → Schematic Editor or PCB Editor
```

### Print the Enclosure

All STL files are in `mechanical/cad_v2/`. Print order:
1. `base.stl` — main body
2. `din_clip.stl` — rail attachment
3. `front_lid.stl` + `top_lid.stl` — covers
4. `top_connectors.stl` + `bottom_connectors.stl` — terminal cutouts

---

## Supported Instruments

| Instrument | Protocol | Datasheet |
|---|---|---|
| Endress+Hauser Promag P300 | Modbus RTU | `data_sheets/Promag_P300_modbus_map.pdf` |
| Endress+Hauser Liquiline CM44x | Modbus RTU | `data_sheets/CM44x_modbus_map.pdf` |
| Generic TDS Sensor | Analog / RS-485 | — |
| Generic Pressure Transmitter | 4–20 mA | — |
| Generic Temperature Sensor | Analog | — |

---

<div align="center">

<br/>

**Takamul Smart Solutions — تكامل للحلول الذكية**

*Industrial IoT, engineered in Egypt 🇪🇬*

<br/>

</div>
