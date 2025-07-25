# RapidMarble (RealTimeTasks + Sensor-Driven Flicker Demo)

**RapidMarble** is a precision motion control demonstration built using RSI’s [RMP EtherCAT Motion Controller](https://www.roboticsys.com/rmp-ethercat-motion-controller), showcasing the capabilities of [**RealTimeTasks**](https://support.roboticsys.com/rmp/rttasks.html) for deterministic, sensor-driven physical interactions — without requiring a real-time OS or low-level firmware code.

The demo features a marble that rolls down a guided track and is sensed mid-path by two digital sensors. The system calculates its speed in real-time and activates a high-speed “flicker” motor at the end of the track to launch the marble back into a funnel, allowing for continuous motion cycles.

---

## 🎯 Key Features

- 🔁 Continuous feedback loop using **RTTask** (1ms deterministic loop)
- ⏱️ Real-time speed calculation based on dual-sensor timestamping
- ⚙️ High-speed flicker mechanism driven by trapezoidal motion profiles
- 👇 Easily swappable with funnel-track or tube-based marble rails
- 🧠 Pure RMP logic — no OS-level real-time kernel or external PLC required

---

## 📁 Project Structure

- `src/` – Core motion and control source code (RMP-based C++)
- `scripts/` – Shell utilities to launch demo
- `assets/` – 3D print models, CADs, and build diagrams *(optional)*
- `README.md` – This document

---

## ⚙️ Hardware Requirements

- **1x** Flicker Motor (e.g. Mitsubishi MR-J5 or AKD)
- **2x** Digital Sensors (photo-interrupt, laser gate, etc.)
- **1x** RMP-enabled EtherCAT controller
- **1x** PC (Linux or Windows)
- **Optional**: Funnel/Track System (3D printed or clear tubing)

---

## 💻 Software Requirements

- RMP SDK (latest stable version)
- C++17 or newer compiler (e.g. `g++`)
- RealTimeTasks enabled in your RMP license
- Linux with kernel RT patch **(optional but recommended)**

---

## 🚀 Getting Started

1. ✅ Wire your sensors to the digital inputs of the drive (e.g., AKD / MR-J5)
2. 🔧 Update axis IDs and input indices in `src/main.cpp`
3. 🛠️ Build the code:
   ```bash
   g++ -o marble_demo marble_demo.cpp -I<path-to-rmp-includes> -L<path-to-rmp-libs> -lrapidcode
