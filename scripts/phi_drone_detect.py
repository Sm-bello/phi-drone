#!/usr/bin/env python3
"""
PHI-DRONE Unified Detection & Logging
- Screen-capture + YOLOv8 inference
- MAVLink telemetry fusion (background thread)
- Performance metrics logging
- Robust session saving (handles Ctrl+C / battery death)
"""

import mss
import numpy as np
import cv2
import csv
import os
import time
import threading
import json
from datetime import datetime
from ultralytics import YOLO
from huggingface_hub import hf_hub_download
from pymavlink import mavutil
import pygetwindow as gw

# ═════════════════════════════════════════════════════════════
# USER CONFIGURATION — EDIT THESE
# ═════════════════════════════════════════════════════════════
OUTPUT_BASE_DIR = r"C:\Users\User\Desktop\COMPLETED_PROJECTS\Drone_Sim\detections"

# MAVLink — type 'output add 127.0.0.1:14551' in MAVProxy console
MAVLINK_CONNECTION = "udp:127.0.0.1:14551"

# Detection tuning
CONFIDENCE_THRESHOLD = 0.45
INFERENCE_SIZE = 640
FRAME_SKIP = 5          # Run inference every N frames
CAPTURE_FPS = 15

# Window crop (tune during live preview)
CROP_TOP = 80
CROP_LEFT = 10
CROP_WIDTH = 800
CROP_HEIGHT = 500

# ═════════════════════════════════════════════════════════════
# SESSION SETUP
# ═════════════════════════════════════════════════════════════
session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
SESSION_DIR = os.path.join(OUTPUT_BASE_DIR, f"session_{session_id}")
SNAPSHOTS_DIR = os.path.join(SESSION_DIR, "snapshots")
os.makedirs(SNAPSHOTS_DIR, exist_ok=True)

video_path = os.path.join(SESSION_DIR, f"video_{session_id}.mp4")
log_path = os.path.join(SESSION_DIR, f"detections_{session_id}.csv")
meta_path = os.path.join(SESSION_DIR, f"meta_{session_id}.json")

# ═════════════════════════════════════════════════════════════
# TELEMETRY STATE (thread-safe)
# ═════════════════════════════════════════════════════════════
telemetry = {"lat": None, "lon": None, "alt": None, "connected": False, "messages": 0}
telemetry_lock = threading.Lock()
stop_telemetry = threading.Event()

# ═════════════════════════════════════════════════════════════
# PERFORMANCE BUFFERS
# ═════════════════════════════════════════════════════════════
perf = {
    "capture_times_ms": [],
    "inference_times_ms": [],
    "loop_times_ms": [],
    "telemetry_reads": 0,
    "frames_captured": 0,
    "frames_inferred": 0,
    "detections_total": 0,
    "session_start": time.time(),
}

# ═════════════════════════════════════════════════════════════
# TELEMETRY WORKER THREAD
# ═════════════════════════════════════════════════════════════
def telemetry_worker():
    print(f"[Telemetry] Connecting to {MAVLINK_CONNECTION}...")
    print("[Telemetry] If this hangs, type 'output add 127.0.0.1:14551' in MAVProxy")
    
    try:
        master = mavutil.mavlink_connection(MAVLINK_CONNECTION)
        master.wait_heartbeat(timeout=10)
        print(f"[Telemetry] ✓ Heartbeat from system {master.target_system}")
        
        with telemetry_lock:
            telemetry["connected"] = True
        
        while not stop_telemetry.is_set():
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=1.0)
            if msg:
                with telemetry_lock:
                    telemetry["lat"] = msg.lat / 1.0e7
                    telemetry["lon"] = msg.lon / 1.0e7
                    telemetry["alt"] = msg.relative_alt / 1000.0
                    telemetry["messages"] += 1
            time.sleep(0.05)
    except Exception as e:
        print(f"[Telemetry] ✗ ERROR: {e}")

telemetry_thread = threading.Thread(target=telemetry_worker, daemon=True)
telemetry_thread.start()

# ═════════════════════════════════════════════════════════════
# GPS LOCK WAIT
# ═════════════════════════════════════════════════════════════
print("\nWaiting for GPS lock (max 60s)...")
gps_ready = False
for i in range(60):
    with telemetry_lock:
        if telemetry["lat"] is not None:
            gps_ready = True
            print(f"✓ GPS locked: {telemetry['lat']:.6f}, {telemetry['lon']:.6f}, {telemetry['alt']:.1f}m")
            break
    time.sleep(1)
    if (i + 1) % 10 == 0:
        print(f"  ... {i+1}s elapsed")

if not gps_ready:
    print("⚠️ WARNING: No GPS lock. Starting anyway — telemetry will be NULL.")

# ═════════════════════════════════════════════════════════════
# WINDOW GEOMETRY
# ═════════════════════════════════════════════════════════════
def get_flightgear_region():
    windows = gw.getWindowsWithTitle("FlightGear")
    if not windows:
        raise RuntimeError("FlightGear window not found")
    
    win = windows[0]
    for action in [win.activate, win.restore]:
        try:
            action()
        except Exception:
            pass
    time.sleep(2)
    
    windows = gw.getWindowsWithTitle("FlightGear")
    win = windows[0]
    
    full = {"top": win.top, "left": win.left, "width": win.width, "height": win.height}
    print(f"[Window] FlightGear geometry: {full}")
    
    return {
        "top": full["top"] + CROP_TOP,
        "left": full["left"] + CROP_LEFT,
        "width": min(CROP_WIDTH, full["width"] - CROP_LEFT),
        "height": min(CROP_HEIGHT, full["height"] - CROP_TOP),
    }

monitor = get_flightgear_region()
print(f"[Window] Capture region: {monitor}\n")

# ═════════════════════════════════════════════════════════════
# LIVE PREVIEW
# ═════════════════════════════════════════════════════════════
print("=" * 60)
print("LIVE PREVIEW — Press 'c' to confirm, 'q' to quit")
print("=" * 60)

with mss.MSS() as sct:
    while True:
        frame = np.array(sct.grab(monitor))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        cv2.imshow("PHI-DRONE Preview", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            print("Preview confirmed. Starting detection...\n")
            break
        elif key == ord('q'):
            stop_telemetry.set()
            cv2.destroyAllWindows()
            exit()

cv2.destroyAllWindows()

# ═════════════════════════════════════════════════════════════
# LOAD MODEL
# ═════════════════════════════════════════════════════════════
print("Loading YOLOv8n (VisDrone)...")
model_path = hf_hub_download(repo_id="mshamrai/yolov8n-visdrone", filename="best.pt")
model = YOLO(model_path)
print("Model loaded.\n")

# ═════════════════════════════════════════════════════════════
# VIDEO + CSV SETUP
# ═════════════════════════════════════════════════════════════
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
video_writer = cv2.VideoWriter(video_path, fourcc, CAPTURE_FPS,
                               (monitor["width"], monitor["height"]))

csv_file = open(log_path, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    "timestamp", "object_class", "confidence", "x1", "y1", "x2", "y2",
    "drone_lat", "drone_lon", "drone_alt_m", "inference_latency_ms", "capture_latency_ms"
])

frame_count = 0
detection_count = 0
last_annotated = None

# ═════════════════════════════════════════════════════════════
# MAIN LOOP — with robust cleanup on ANY exit
# ═════════════════════════════════════════════════════════════
def save_metadata():
    """Write session metadata + performance summary to JSON."""
    perf["session_end"] = time.time()
    perf["session_duration_sec"] = perf["session_end"] - perf["session_start"]
    
    with telemetry_lock:
        tel_copy = dict(telemetry)
    
    meta = {
        "session_id": session_id,
        "config": {
            "confidence_threshold": CONFIDENCE_THRESHOLD,
            "inference_size": INFERENCE_SIZE,
            "frame_skip": FRAME_SKIP,
            "capture_fps": CAPTURE_FPS,
            "crop": {"top": CROP_TOP, "left": CROP_LEFT, "width": CROP_WIDTH, "height": CROP_HEIGHT},
        },
        "paths": {"video": video_path, "csv": log_path, "snapshots": SNAPSHOTS_DIR},
        "telemetry": tel_copy,
        "performance": {
            "frames_captured": perf["frames_captured"],
            "frames_inferred": perf["frames_inferred"],
            "detections_total": perf["detections_total"],
            "session_duration_sec": round(perf["session_duration_sec"], 2),
            "capture_fps_actual": round(perf["frames_captured"] / perf["session_duration_sec"], 2) if perf["session_duration_sec"] > 0 else 0,
            "inference_fps_actual": round(perf["detections_total"] / perf["session_duration_sec"], 2) if perf["session_duration_sec"] > 0 else 0,
            "inference_latency_ms": {
                "mean": round(float(np.mean(perf["inference_times_ms"])), 2) if perf["inference_times_ms"] else None,
                "median": round(float(np.median(perf["inference_times_ms"])), 2) if perf["inference_times_ms"] else None,
                "std": round(float(np.std(perf["inference_times_ms"])), 2) if perf["inference_times_ms"] else None,
                "min": round(float(np.min(perf["inference_times_ms"])), 2) if perf["inference_times_ms"] else None,
                "max": round(float(np.max(perf["inference_times_ms"])), 2) if perf["inference_times_ms"] else None,
                "p95": round(float(np.percentile(perf["inference_times_ms"], 95)), 2) if perf["inference_times_ms"] else None,
            },
        },
    }
    
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"[Meta] Saved to {meta_path}")

try:
    with mss.MSS() as sct:
        while True:
            loop_start = time.time()
            
            # Capture
            t_cap0 = time.time()
            frame = np.array(sct.grab(monitor))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            t_cap = (time.time() - t_cap0) * 1000
            
            perf["frames_captured"] += 1
            perf["capture_times_ms"].append(t_cap)
            
            # Inference (every FRAME_SKIP frames)
            if frame_count % FRAME_SKIP == 0:
                perf["frames_inferred"] += 1
                resized = cv2.resize(frame, (INFERENCE_SIZE, INFERENCE_SIZE))
                
                t_inf0 = time.time()
                results = model(resized, verbose=False, conf=CONFIDENCE_THRESHOLD)
                t_inf = (time.time() - t_inf0) * 1000
                
                perf["inference_times_ms"].append(t_inf)
                boxes = results[0].boxes
                
                with telemetry_lock:
                    lat = telemetry["lat"]
                    lon = telemetry["lon"]
                    alt = telemetry["alt"]
                
                if len(boxes) > 0:
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    scale_x = monitor["width"] / INFERENCE_SIZE
                    scale_y = monitor["height"] / INFERENCE_SIZE
                    
                    for box in boxes:
                        cls_name = model.names[int(box.cls)]
                        conf = float(box.conf)
                        x1, y1, x2, y2 = [round(float(v) * scale_x, 1) for v in box.xyxy[0]]
                        
                        gps_str = f"{lat:.6f}, {lon:.6f}, {alt:.1f}m" if lat else "NO GPS"
                        print(f"[{timestamp}] {cls_name} ({conf:.2f}) | {t_inf:.0f}ms | {gps_str}")
                        
                        csv_writer.writerow([
                            timestamp, cls_name, f"{conf:.2f}",
                            x1, y1, x2, y2, lat, lon, alt,
                            f"{t_inf:.1f}", f"{t_cap:.1f}"
                        ])
                    
                    csv_file.flush()
                    detection_count += 1
                    perf["detections_total"] += len(boxes)
                    
                    snapshot_name = f"det_{timestamp.replace(':', '-').replace(' ', '_')}.jpg"
                    cv2.imwrite(os.path.join(SNAPSHOTS_DIR, snapshot_name), frame)
                
                annotated_resized = results[0].plot()
                last_annotated = cv2.resize(annotated_resized, (monitor["width"], monitor["height"]))
            
            # Display
            annotated = last_annotated if last_annotated is not None else frame
            video_writer.write(annotated)
            cv2.imshow("PHI-DRONE Detection", annotated)
            
            frame_count += 1
            loop_time = (time.time() - loop_start) * 1000
            perf["loop_times_ms"].append(loop_time)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[User] Quit signal received.")
                break

except KeyboardInterrupt:
    print("\n[Interrupt] Ctrl+C caught — saving session...")

finally:
    # ═════════════════════════════════════════════════════════
    # ROBUST CLEANUP — runs even on battery death / crash
    # ═════════════════════════════════════════════════════════
    stop_telemetry.set()
    
    if video_writer is not None:
        video_writer.release()
    if csv_file is not None:
        csv_file.close()
    cv2.destroyAllWindows()
    
    save_metadata()
    
    print("\n" + "=" * 60)
    print("SESSION ENDED")
    print("=" * 60)
    print(f"Duration:      {perf['session_duration_sec']:.1f} s ({perf['session_duration_sec']/60:.1f} min)")
    print(f"Frames:        {perf['frames_captured']}")
    print(f"Inferences:    {perf['frames_inferred']}")
    print(f"Detections:    {perf['detections_total']}")
    print(f"Output dir:    {SESSION_DIR}")
    print("=" * 60)