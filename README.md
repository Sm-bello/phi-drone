# PHI-DRONE

> **The Digital Twin Reconnaissance Hub** — Simulating UAV Vision and Flight in a Unified Pipeline

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![YOLOv8](https://img.shields.io/badge/YOLO-v8-8A2BE2.svg)](https://github.com/ultralytics/ultralytics)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-green.svg)](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)

---

## Overview

**PHI-DRONE** is a fully simulated UAV reconnaissance hub that couples high-fidelity flight dynamics with a live computer-vision detection pipeline — **zero physical hardware required**.

The system integrates three independently-developed open-source technologies into a single working pipeline:

1. **ArduPilot SITL** — executes the real ArduCopter flight-control firmware against a simulated vehicle model
2. **FlightGear** — renders the 3D visual environment, driven by SITL's Flight Dynamics Model (FDM) output over UDP
3. **YOLOv8 + VisDrone** — performs real-time object detection on screen-captured frames of the simulated onboard view

A background MAVLink telemetry listener continuously records the simulated vehicle's GPS position and altitude, merging every detection event with the vehicle's true position at the moment of detection.

---

## 📄 Citation

If you use PHI-DRONE in your research, please cite:

```bibtex
@article{sani2026phidrone,
  title={PHI-DRONE: The Digital Twin Reconnaissance Hub --- Simulating UAV Vision and Flight in a Unified Pipeline},
  author={Sani, Mohammed Bello and Ibrahim, Tumsah Usman and Abbe, Godwin},
  journal={Aerospace Science and Technology},
  year={2026},
  publisher={Elsevier}
}
```

---

## 🏗️ Architecture

```
┌─────────────────┐     UDP FDM      ┌─────────────────┐
│  ArduPilot SITL │ ───────────────> │    FlightGear   │
│  (WSL2 Ubuntu)  │                  │  (Windows GUI)  │
└─────────────────┘                  └────────┬────────┘
       │                                      │
       │ MAVLink                              │ Screen Capture
       │                                      ▼
       │                              ┌─────────────────┐
       │                              │   YOLOv8 Inference
       │                              │  (Windows Python)
       │                              └────────┬────────┘
       │                                       │
       │         Telemetry Fusion              │ Detection Events
       │ <─────────────────────────────────────┤
       │                                       │
       ▼                                       ▼
┌─────────────────────────────────────────────────────────┐
│                    Unified CSV Log                       │
│  [timestamp, class, confidence, bbox, lat, lon, alt]   │
└─────────────────────────────────────────────────────────┘
```

---

## 🚀 Quick Start

### Prerequisites

| Component | Version | Notes |
|-----------|---------|-------|
| Windows 10/11 | 21H2+ | WSL2 required for SduPilot SITL |
| WSL2 Ubuntu | 24.04 (Noble) | Mirrored networking mode recommended |
| Python | 3.10+ | Conda environment recommended |
| FlightGear | 2024.1 | [Download](https://www.flightgear.org/download/) |
| ArduPilot SITL | Latest | Clone from [GitHub](https://github.com/ArduPilot/ardupilot) |
| YOLOv8 Weights | VisDrone fine-tune | [Download checkpoint](https://github.com/ultralytics/ultralytics) |

### 1. Clone the Repository

```bash
git clone https://github.com/username/phi-drone.git
cd phi-drone
```

### 2. Install Python Dependencies

```bash
conda create -n aerospace python=3.11
conda activate aerospace
pip install -r requirements.txt
```

### 3. Build ArduPilot SITL (inside WSL2)

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

### 4. Launch the Full Pipeline

```bash
# From the project root (Windows side)
cd phi-drone/src
launch_drone_sim.bat
```

This will:
1. Start FlightGear with the ArduCopter model
2. Launch SITL inside WSL2
3. Start MAVProxy ground control
4. Launch the detection and telemetry fusion script

### 5. Run a Mission

In the MAVProxy console:
```
mode guided
arm throttle
takeoff 10
guided -35.363 149.170 30
```

### 6. Shutdown

```bash
shutdown_drone_sim.bat
```

---

## 📁 Repository Structure

```
phi-drone/
├── README.md                          # This file
├── LICENSE                            # MIT License
├── requirements.txt                   # Python dependencies
├── .gitignore                         # Git ignore rules
│
├── src/                               # Source code
│   ├── detect_and_log_unified.py     # Main detection + telemetry script
│   ├── config.py                      # Configuration constants
│   ├── launch_drone_sim.bat          # Windows batch launcher
│   ├── shutdown_drone_sim.bat        # Windows batch shutdown
│   └── utils/
│       ├── telemetry_worker.py        # MAVLink background thread
│       ├── screen_capture.py          # FlightGear window capture
│       └── logger.py                  # CSV + video logging utilities
│
├── docs/                              # Documentation
│   ├── MANUSCRIPT.md                  # Link to published paper
│   └── SETUP_GUIDE.md                # Detailed installation walkthrough
│
├── data/
│   └── sample_missions/               # Example CSV logs and videos
│
└── figures/                           # Paper figures (PDF/EPS)
    ├── fig1_architecture.pdf
    ├── fig2_detection_overlay.pdf
    └── ...
```

---

## ⚙️ Configuration

Edit `src/config.py` to adjust:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CAPTURE_FPS` | 15 | Screen capture frame rate |
| `FRAME_SKIP` | 5 | Inference every N frames (3 fps effective) |
| `CONFIDENCE_THRESHOLD` | 0.25 | Minimum detection confidence |
| `FLIGHTGEAR_WINDOW` | "FlightGear" | Window title for pygetwindow |
| `MAVLINK_CONNECTION` | "tcp:127.0.0.1:14550" | MAVLink endpoint |
| `MODEL_PATH` | `"yolov8n-visdrone.pt"` | Path to YOLOv8 weights |

---

## 📊 Sample Output

After a mission, the following artefacts are generated in `data/sample_missions/`:

- **`detection_log_YYYYMMDD_HHMMSS.csv`** — structured detection + telemetry data
- **`mission_video_YYYYMMDD_HHMMSS.avi`** — full annotated video recording
- **`snapshots/`** — individual JPEG frames for each detection event

Example CSV row:

```csv
timestamp,class,confidence,x1,y1,x2,y2,drone_lat,drone_lon,drone_alt
2026-07-12T14:32:15.342,car,0.67,245,189,312,234,-35.363212,149.170341,30.2
```

---

## 🛠️ Development Environment

- **OS:** Windows 11 + WSL2 Ubuntu 24.04
- **Hardware:** HP ProBook 440 G10 (Intel i5, 16 GB RAM, integrated graphics)
- **Python:** 3.11 via Conda
- **Network:** WSL2 mirrored networking mode (allows `localhost` bridging)

---

## 🤝 Contributing

Contributions are welcome! Please open an issue or pull request for:

- Photorealistic scenery integration (OSM2City)
- Object-level geolocation (pixel-to-ground projection)
- Live dashboard interface (real-time web UI)
- MATLAB UAV Toolbox migration path
- Additional aircraft models (fixed-wing, VTOL)

See [CONTRIBUTING.md](docs/CONTRIBUTING.md) for guidelines.

---

## 📜 License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.

---

## 🙏 Acknowledgements

- [ArduPilot](https://ardupilot.org/) — open-source autopilot firmware
- [FlightGear](https://www.flightgear.org/) — open-source flight simulator
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) — object detection framework
- [MAVLink](https://mavlink.io/) — lightweight messaging protocol
- [VisDrone Dataset](https://github.com/VisDrone/VisDrone-Dataset) — aerial imagery benchmark
- Aerospace Engineering Department, Air Force Institute of Technology, Kaduna

---

## 📧 Contact

**Mohammed Bello Sani** — mohammed.sani@student.afit.edu.ng  
**Project Supervisor:** Group Captain Godwin Abbe — abbe.godwin@afit.edu.ng

---

*Built with discipline, open-source tools, and zero hardware cost.*
