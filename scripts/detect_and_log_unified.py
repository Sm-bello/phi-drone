import mss
import numpy as np
import cv2
import csv
import os
import time
import threading
from datetime import datetime
from ultralytics import YOLO
from huggingface_hub import hf_hub_download
from pymavlink import mavutil
import pygetwindow as gw

# ---- Configuration ----
OUTPUT_DIR = r"C:\Users\User\Desktop\COMPLETED_PROJECTS\Drone_Sim\detections"
SNAPSHOTS_DIR = os.path.join(OUTPUT_DIR, "snapshots")
os.makedirs(SNAPSHOTS_DIR, exist_ok=True)

CONFIDENCE_THRESHOLD = 0.45
INFERENCE_SIZE = 640
FRAME_SKIP = 5
CAPTURE_FPS = 15

# Use the correct MAVLink port from your MAVProxy
MAVLINK_CONNECTION = "udp:127.0.0.1:14551"   # <--- Updated as per your setup

# Tune these CROP values while LIVE PREVIEW is running
CROP_TOP = 80      # Increase if you see menu bar
CROP_LEFT = 10
CROP_WIDTH = 800
CROP_HEIGHT = 500

session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
video_path = os.path.join(OUTPUT_DIR, f"session_{session_id}.mp4")
log_path = os.path.join(OUTPUT_DIR, f"detections_{session_id}.csv")

telemetry = {"lat": None, "lon": None, "alt": None}
telemetry_lock = threading.Lock()
stop_telemetry = threading.Event()

def telemetry_worker():
    print("Connecting to MAVLink telemetry stream...")
    master = mavutil.mavlink_connection(MAVLINK_CONNECTION)
    master.wait_heartbeat()
    print(f"MAVLink heartbeat received (system {master.target_system}). Telemetry live.\n")
    
    while not stop_telemetry.is_set():
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            with telemetry_lock:
                telemetry["lat"] = msg.lat / 1.0e7
                telemetry["lon"] = msg.lon / 1.0e7
                telemetry["alt"] = msg.alt / 1000.0

telemetry_thread = threading.Thread(target=telemetry_worker, daemon=True)
telemetry_thread.start()

# Improved window detection
def get_flightgear_region():
    windows = gw.getWindowsWithTitle("FlightGear")
    if not windows:
        raise RuntimeError("FlightGear window not found — make sure it's open and visible")
    
    win = windows[0]
    win.activate()
    win.restore()
    time.sleep(2)  # Give more time for window to settle
    
    # Re-query
    windows = gw.getWindowsWithTitle("FlightGear")
    win = windows[0]
    
    full = {"top": win.top, "left": win.left, "width": win.width, "height": win.height}
    print(f"FlightGear window geometry: {full}")
    
    return {
        "top": full["top"] + CROP_TOP,
        "left": full["left"] + CROP_LEFT,
        "width": min(CROP_WIDTH, full["width"] - CROP_LEFT),
        "height": min(CROP_HEIGHT, full["height"] - CROP_TOP),
    }

monitor = get_flightgear_region()
print(f"Capture region: {monitor}\n")

# ==================== LIVE PREVIEW ====================
print("=" * 60)
print("LIVE PREVIEW MODE - Adjust CROP if needed")
print("=" * 60)

with mss.mss() as sct:
    while True:
        frame = np.array(sct.grab(monitor))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        cv2.imshow("LIVE PREVIEW - FlightGear View", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            print("Preview confirmed. Starting detection...\n")
            break
        elif key == ord('q'):
            print("Aborted.")
            stop_telemetry.set()
            cv2.destroyAllWindows()
            exit()

cv2.destroyAllWindows()

# Load Model
print("Loading YOLOv8 model...")
model_path = hf_hub_download(repo_id="mshamrai/yolov8n-visdrone", filename="best.pt")
model = YOLO(model_path)
print("Model loaded.\n")

# Video + CSV setup
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
video_writer = cv2.VideoWriter(video_path, fourcc, CAPTURE_FPS, (monitor["width"], monitor["height"]))

csv_file = open(log_path, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["timestamp", "object_class", "confidence", "x1", "y1", "x2", "y2", "drone_lat", "drone_lon", "drone_alt_m"])

frame_count = 0
detection_count = 0
last_annotated = None

try:
    with mss.mss() as sct:
        while True:
            frame = np.array(sct.grab(monitor))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            if frame_count % FRAME_SKIP == 0:
                resized = cv2.resize(frame, (INFERENCE_SIZE, INFERENCE_SIZE))
                results = model(resized, verbose=False, conf=CONFIDENCE_THRESHOLD)
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

                        print(f"[{timestamp}] Detected: {cls_name} ({conf:.2f}) | Drone @ {lat}, {lon}, {alt}m")
                        csv_writer.writerow([timestamp, cls_name, f"{conf:.2f}", x1, y1, x2, y2, lat, lon, alt])

                    csv_file.flush()
                    detection_count += 1
                    snapshot_name = f"detection_{timestamp.replace(':', '-').replace(' ', '_')}.jpg"
                    cv2.imwrite(os.path.join(SNAPSHOTS_DIR, snapshot_name), frame)

                annotated_resized = results[0].plot()
                last_annotated = cv2.resize(annotated_resized, (monitor["width"], monitor["height"]))

            annotated = last_annotated if last_annotated is not None else frame
            video_writer.write(annotated)
            cv2.imshow("Drone View - YOLO Detection", annotated)

            frame_count += 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

finally:
    stop_telemetry.set()
    video_writer.release()
    csv_file.close()
    cv2.destroyAllWindows()
    print(f"\nSession ended. Frames: {frame_count} | Detections: {detection_count}")
