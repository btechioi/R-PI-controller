<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&amp;color=0:00e1ff,100:0055ff&amp;height=200&amp;section=header&amp;text=R-PI-controller&amp;fontSize=45&amp;fontAlignY=35&amp;animation=fadeIn&amp;fontColor=ffffff"/>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/-Python-3776AB?logo=python&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-C%2B%2B-00599C?logo=cplusplus&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-Arduino-00979D?logo=arduino&amp;logoColor=white&amp;style=for-the-badge"/>
</p>

<p align="center">
  <b>Raspberry Pi controller with Arduino subsystems</b>
</p>

---

## 🛸 Overview

A multi-module controller system using a Raspberry Pi Zero as the central orchestrator, with Arduino nodes handling IO, display, and wireless communication.

---

## 🛠️ Architecture

```mermaid
flowchart LR
    A[Pi Zero\nPython Orchestrator] --> B[Arduino IO]
    A --> C[Arduino Display]
    A --> D[Arduino Wireless]
    D --> E[Remote Controller]
    C --> F[Screen / UI]
    B --> G[Sensors & Actuators]
```

---

## 📁 Modules

| Module | Platform | Role |
|--------|----------|------|
| Pi-Zero | Python | Main controller logic |
| IO | Arduino | Sensor reading & actuation |
| Display | Arduino | Screen output |
| Wireless | Arduino | Remote communication |

---

## 🚀 Tech Stack

<img src="https://img.shields.io/badge/-Python-3776AB?logo=python&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-C%2B%2B-00599C?logo=cplusplus&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-Arduino-00979D?logo=arduino&amp;logoColor=white&amp;style=for-the-badge"/> <img src="https://img.shields.io/badge/-Raspberry%20Pi-A22846?logo=raspberrypi&amp;logoColor=white&amp;style=for-the-badge"/>

---

<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=soft&amp;color=0:00e1ff,100:0055ff&amp;height=100&amp;section=footer&amp;text=control%20everything&amp;fontSize=20&amp;fontAlignY=50&amp;fontColor=ffffff"/>
</p>
