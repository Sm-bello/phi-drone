<div align="center">

# PHI-Drone

### Digital Twin Reconnaissance Hub — Simulating UAV Vision and Flight in a Unified Pipeline

A home-built simulation testbed fusing **ArduPilot SITL** flight physics, **FlightGear** visualization, and a **YOLOv8** aerial object detector under one **MAVLink**-synchronized telemetry layer — no physical airframe required.

[![Status](https://img.shields.io/badge/status-prototype%20%2F%20validated-blue)]()
[![Python](https://img.shields.io/badge/python-3.11-informational)]()
[![Platform](https://img.shields.io/badge/platform-Windows%20%2B%20WSL2-lightgrey)]()
[![License](https://img.shields.io/badge/license-MIT-green)]()

</div>

---

## What this is

PHI-Drone runs a **real ArduPilot flight-control stack** (the same firmware used on physical autopilots) against a simulated vehicle, renders that flight live in **FlightGear**, and feeds the rendered onboard view into a **YOLOv8n detector fine-tuned on VisDrone** — with every detection stamped by a live GPS position pulled independently over MAVLink.

It exists to answer one question cheaply, before any hardware is procured: *what does a UAV's detector actually see along a real, physically-modeled flight path?*

This is **not** a novel detection algorithm and does not claim to be — it's an integration testbed, and the engineering effort of making independently-developed open-source tools interoperate reliably is the actual contribution. See [Known Issues & Engineering Log](#known-issues--engineering-log) for the honest account of what broke and how it was fixed.

---

## Architecture

<img width="1693" height="929" alt="architecture" src="https://github.com/user-attachments/assets/45c0a671-257d-4e3a-9e1d-082c1ed54136" />

| Stage | Component | Role |
|---|---|---|
| Flight physics | ArduPilot SITL (WSL2) | Executes real ArduCopter firmware against a simulated vehicle model |
| Visualization | FlightGear 2024.1 (Windows) | Renders the vehicle's onboard view, driven entirely by SITL over UDP — performs no physics of its own |
| Vision | YOLOv8n, VisDrone fine-tune | Real-time inference on screen-captured frames of the rendered view |
| Telemetry | pymavlink, background thread | Streams live GPS position into every detection, independent of the vision loop |
| Output | CSV + video + snapshots | Unified, timestamped log for post-mission review |

SITL and FlightGear communicate over a UDP Flight Dynamics Model (FDM) socket (port `5503`); the detection script's MAVLink listener runs on a separate UDP output (port `14551`) so it never contends with MAVProxy's own connection. The two channels — vision and telemetry — only converge at the moment a detection is logged, so a blocking network read can never stall frame capture.

---

## Status

Verified independently. Fusion into one continuous live feed is the active engineering phase — stated plainly, not smoothed over.

| Component | Status |
|---|---|
| Flight control (SITL build, arm/takeoff/guided/circle) | ✅ Verified |
| Aerial detection (YOLOv8n VisDrone, static + live imagery) | ✅ Verified |
| Telemetry fusion (MAVLink → CSV) | ✅ Verified |
| FlightGear ↔ SITL visual sync | ⚠️ In progress — see engineering log |
| Live dashboard | ⬜ Not started |
| Object-level geolocation | ⬜ Not started |
| Quantitative evaluation (precision/recall/mAP, latency) | ⬜ Not started |

---

## Repository structure

```
phi-drone/
├── scripts/
│   ├── detect_and_log_unified.py   # Screen-capture + YOLOv8 inference + MAVLink telemetry fusion
│   ├── analyze_session.py          # Post-session metrics: detections, confidence, frame rate
│   ├── launch_drone_sim.bat        # One-command orchestration: FlightGear → SITL → detection
│   └── shutdown_drone_sim.bat      # Clean shutdown, releases all ports
├── config/
│   └── telemetry_in.xml            # Optional FlightGear generic-protocol motor telemetry (cosmetic only)
├── docs/
│   └── architecture.png
├── requirements.txt
├── .gitignore
├── LICENSE
└── README.md
```

---

## Prerequisites

- **Windows 10/11** with **WSL2** (Ubuntu 24.04 / Noble), configured with **mirrored networking**
- **FlightGear 2024.1** installed natively on Windows
- **Python 3.11** (Conda environment recommended)
- **ArduPilot** source, built for the `sitl` board target
- A GPU is not required — inference runs adequately on CPU with frame-skipping enabled

### Enable WSL2 mirrored networking

Create `C:\Users\<you>\.wslconfig`:
```ini
[wsl2]
networkingMode=mirrored
```
Then `wsl --shutdown` and reopen your terminal. This is what allows `127.0.0.1` to resolve identically between Windows and WSL2 — no manual IP lookup, no network bridge needed.

---

## Setup

### 1. Build ArduPilot SITL (inside WSL2 Ubuntu)

```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
```

> If your connection is unstable, prefer `aria2c` over plain `git clone`/`curl` — segmented, independently-retried downloads survive packet loss far better than a single continuous stream. See the engineering log below.

### 2. Get the ArduPilot-bundled aircraft model into FlightGear

ArduPilot ships its own FlightGear-compatible quadcopter model — you do **not** need to search FlightGear's aircraft hangar.

```bash
# from Windows, in File Explorer:
\\wsl.localhost\Ubuntu\home\<you>\ardupilot\Tools\autotest\aircraft
```
Copy the `arducopter` folder to a local Windows path, e.g. `C:\FlightGearAircraft\arducopter`.

### 3. Python environment

```bash
conda create -n aerospace python=3.11
conda activate aerospace
pip install -r requirements.txt
```

### 4. Configure paths

Open `scripts/detect_and_log_unified.py` and confirm:
```python
OUTPUT_DIR = r"C:\path\to\your\detections\folder"
MAVLINK_CONNECTION = "udp:127.0.0.1:14551"
```
Open `scripts/launch_drone_sim.bat` and confirm the FlightGear install path and aircraft folder path match your machine.

---

## Usage

```cmd
scripts\launch_drone_sim.bat
```

This sequences: cleanup of any leftover processes → port availability check → FlightGear launch → SITL launch (with a dedicated MAVLink output on `14551` for the detection script) → a **mandatory manual verification checkpoint**.

**At the checkpoint:** type `takeoff 10` in the MAVProxy terminal and confirm FlightGear's own on-screen altitude climbs in sync with MAVProxy's console. Do not proceed to detection until this is confirmed — this single check is what catches a stale/ghost FlightGear process before it silently corrupts a session (see engineering log).

Once confirmed, the detection script starts, opens a live preview window, and begins logging.

### Flying a mission

In the MAVProxy terminal:
```
mode guided
arm throttle
takeoff 10
guided <lat> <lon> <alt>
```
Or right-click any point on the MAVProxy Map → *Fly Here*.

### Output

```
detections/
├── session_<timestamp>.mp4          # Full annotated video
├── detections_<timestamp>.csv       # timestamp, class, confidence, bbox, drone_lat, drone_lon, drone_alt
└── snapshots/
    └── detection_<timestamp>_<frame>.jpg
```

Run `scripts/analyze_session.py <csv> <video>` for a quick summary (total detections, mean confidence, measured frame rate).

---

## Known Issues & Engineering Log

Documented here deliberately, because a pipeline that admits what broke is more trustworthy than one that doesn't.

| Issue | Root Cause | Resolution |
|---|---|---|
| **Detector reading the desktop, not the sim** | Uncalibrated/stale screen-capture region — window geometry queried before Windows finished repositioning the window | Programmatic window-geometry verification (`pygetwindow`) + mandatory pre-flight `capture_region_check.jpg` + bounds checking that aborts loudly rather than silently capturing garbage |
| **FlightGear frozen while SITL genuinely flew** | A minimized "ghost" FlightGear instance from a prior session held the UDP FDM port binding | Explicit process enumeration/kill of `fgfs.exe` and all SITL/MAVProxy processes before every session; port-in-use check via `netstat` in the launcher |
| **Repeated `git clone`/download failures under packet loss** | HTTP/2 stream cancellation on an unstable connection (~3% packet loss) | Forced HTTP/1.1 + increased `http.postBuffer`; fell back to `aria2c` segmented, independently-retried downloads for large transfers |
| **Git submodules silently empty after tarball extraction** | A tarball export has no git object/index metadata, so `git submodule update --init` had nothing to act on | `git reset --hard origin/master` against a properly fetched commit to populate submodule gitlinks before retrying the submodule update |
| **Detection loop stalling / choppy video** | Blocking MAVLink telemetry reads sharing the same loop as frame capture | Telemetry moved to a dedicated background daemon thread updating a lock-protected shared state; capture loop never blocks on network I/O |
| **Video/CSV corrupted after Ctrl+C** | `VideoWriter.release()` / file-close never ran on interrupt | Entire capture loop wrapped in `try/finally` — cleanup runs regardless of how the loop exits |
| **FlightGear `--generic` motor-telemetry protocol mistaken for position sync** | `--generic` + `--fdm=external` carries only motor duty-cycle data (`/apm/motor_*`), not position/attitude — cannot move the aircraft | Reverted to `--native-fdm=socket,in` as the sole position-sync channel; motor telemetry (if wanted) is a separate, additive, cosmetic-only channel |

---

## Roadmap

- [ ] Re-verify FlightGear ↔ SITL sync end-to-end after the latest process-cleanup hardening
- [ ] Object-level geolocation — project bounding boxes to ground coordinates using altitude + camera geometry
- [ ] Live dashboard (telemetry + detection panes, real-time)
- [ ] Quantitative evaluation — precision/recall/mAP@0.5 against a labeled frame sample; latency and throughput benchmarks
- [ ] Richer scenery (e.g. OSM2City) to increase real-world object density beyond FlightGear's default terrain

---

## Acknowledgments & Attribution

- [ArduPilot](https://ardupilot.org/) — flight-control firmware and SITL
- [FlightGear](https://www.flightgear.org/) — open-source flight simulator
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- [VisDrone Dataset](https://github.com/VisDrone/VisDrone-Dataset) (Zhu et al.)
- [`mshamrai/yolov8n-visdrone`](https://huggingface.co/mshamrai/yolov8n-visdrone) — pretrained checkpoint used directly
- [MAVLink](https://mavlink.io/) / [pymavlink](https://github.com/ArduPilot/pymavlink)

Development was assisted by AI tools (Claude, Grok) for debugging support and documentation drafting; all engineering decisions, verification, and testing were carried out and confirmed by the author.

---

## Author

**Mohammed Bello Sani** — Aerospace Engineering, Air Force Institute of Technology (AFIT), Kaduna, Nigeria
Portfolio: [smbello.vercel.app](https://smbello.vercel.app) · GitHub: [@Sm-bello](https://github.com/Sm-bello)

## License

MIT — see [LICENSE](LICENSE).
