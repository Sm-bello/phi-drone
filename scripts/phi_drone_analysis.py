#!/usr/bin/env python3
"""
PHI-DRONE Unified Analysis & Benchmark Suite
- Mission CSV analysis (single or batch)
- COCO mAP evaluation (requires labeled GT)
- Domain gap: VisDrone vs FlightGear
- Latency benchmark (robust stats: Median ± IQR)
- Auto-generates LaTeX tables for paper
"""

import os
import sys
import json
import time
import argparse
import glob
import statistics
from pathlib import Path
from collections import defaultdict
from datetime import datetime

import numpy as np
import pandas as pd
import cv2
from ultralytics import YOLO
from huggingface_hub import hf_hub_download

# Optional: only needed if you have COCO GT
try:
    from pycocotools.coco import COCO
    from pycocotools.cocoeval import COCOeval
    PYCOCO_AVAILABLE = True
except ImportError:
    PYCOCO_AVAILABLE = False
    print("[Warning] pycocotools not installed. mAP evaluation disabled.")

try:
    import mss
    MSS_AVAILABLE = True
except ImportError:
    MSS_AVAILABLE = False


# ═════════════════════════════════════════════════════════════
# CONFIG
# ═════════════════════════════════════════════════════════════
MODEL_REPO = "mshamrai/yolov8n-visdrone"
MODEL_FILE = "best.pt"
CONFIDENCE_THRESHOLD = 0.45
INFERENCE_SIZE = 640


# ═════════════════════════════════════════════════════════════
# UTILITIES
# ═════════════════════════════════════════════════════════════
def load_model():
    print("Loading model...")
    path = hf_hub_download(repo_id=MODEL_REPO, filename=MODEL_FILE)
    return YOLO(path)


def robust_stats(arr, label=""):
    """Median ± IQR. No outlier hand-waving."""
    a = np.array(arr)
    q1, med, q3 = np.percentile(a, [25, 50, 75])
    iqr = q3 - q1
    p95 = np.percentile(a, 95)
    p99 = np.percentile(a, 99)
    return {
        "label": label,
        "n": len(a),
        "median_ms": float(med),
        "iqr_ms": float(iqr),
        "q1_ms": float(q1),
        "q3_ms": float(q3),
        "mean_ms": float(a.mean()),
        "std_ms": float(a.std()),
        "min_ms": float(a.min()),
        "max_ms": float(a.max()),
        "p95_ms": float(p95),
        "p99_ms": float(p99),
    }


def print_stats_table(d):
    print(f"\n  {'Metric':<20} {'Value':>12}")
    print(f"  {'-'*34}")
    print(f"  {'N':<20} {d['n']:>12}")
    print(f"  {'Median (ms)':<20} {d['median_ms']:>12.2f}")
    print(f"  {'IQR (ms)':<20} {d['iqr_ms']:>12.2f}")
    print(f"  {'Mean ± Std (ms)':<20} {d['mean_ms']:>12.2f} ± {d['std_ms']:.2f}")
    print(f"  {'Min / Max (ms)':<20} {d['min_ms']:>12.2f} / {d['max_ms']:.2f}")
    print(f"  {'95th %ile (ms)':<20} {d['p95_ms']:>12.2f}")


# ═════════════════════════════════════════════════════════════
# 1. MISSION CSV ANALYSIS
# ═════════════════════════════════════════════════════════════
def analyze_mission(csv_path):
    print(f"\n{'='*70}")
    print(f"MISSION ANALYSIS: {csv_path}")
    print(f"{'='*70}")
    
    df = pd.read_csv(csv_path)
    total = len(df)
    
    print(f"\n[1] Basic Stats")
    print(f"  Total detections: {total:,}")
    
    # Confidence
    if 'confidence' in df.columns:
        c = df['confidence']
        print(f"\n[2] Confidence")
        print(f"  Mean:   {c.mean():.3f} ± {c.std():.3f}")
        print(f"  Median: {c.median():.3f}")
        print(f"  Range:  {c.min():.3f} – {c.max():.3f}")
    
    # Classes
    if 'object_class' in df.columns:
        print(f"\n[3] Class Distribution")
        for cls, cnt in df['object_class'].value_counts().items():
            print(f"  {cls}: {cnt:,} ({cnt/total*100:.1f}%)")
        print(f"  Unique classes: {df['object_class'].nunique()}")
    
    # GPS
    if 'drone_lat' in df.columns:
        gps_valid = df['drone_lat'].notna().sum()
        print(f"\n[4] Telemetry Fusion")
        print(f"  GPS-annotated detections: {gps_valid:,} / {total:,} ({gps_valid/total*100:.1f}%)")
    
    # Latency
    if 'inference_latency_ms' in df.columns:
        lat = df['inference_latency_ms'].dropna().values
        if len(lat) > 0:
            stats = robust_stats(lat, "Mission Inference Latency")
            print(f"\n[5] Inference Latency")
            print_stats_table(stats)
    
    # Duration estimate
    if 'timestamp' in df.columns:
        try:
            df['ts'] = pd.to_datetime(df['timestamp'])
            dur = (df['ts'].max() - df['ts'].min()).total_seconds()
            print(f"\n[6] Mission Duration")
            print(f"  ~{dur:.0f} seconds ({dur/60:.1f} minutes)")
        except Exception:
            pass
    
    print(f"\n{'='*70}")


# ═════════════════════════════════════════════════════════════
# 2. COCO mAP EVALUATION
# ═════════════════════════════════════════════════════════════
def evaluate_map(model, image_dir, gt_json, out_pred="coco_preds.json"):
    if not PYCOCO_AVAILABLE:
        print("ERROR: Install pycocotools first: pip install pycocotools")
        return None
    
    print(f"\n{'='*70}")
    print(f"mAP EVALUATION")
    print(f"{'='*70}")
    
    coco_gt = COCO(gt_json)
    img_ids = coco_gt.getImgIds()
    img_infos = {img['id']: img['file_name'] for img in coco_gt.loadImgs(img_ids)}
    
    results = []
    print(f"Running inference on {len(img_ids)} labeled frames...")
    
    for img_id, fname in img_infos.items():
        p = Path(image_dir) / fname
        if not p.exists():
            continue
        preds = model(str(p), verbose=False, conf=CONFIDENCE_THRESHOLD)[0]
        for box in preds.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            w, h = x2 - x1, y2 - y1
            results.append({
                "image_id": img_id,
                "category_id": int(box.cls) + 1,
                "bbox": [float(x1), float(y1), float(w), float(h)],
                "score": float(box.conf)
            })
    
    with open(out_pred, "w") as f:
        json.dump(results, f)
    
    coco_dt = coco_gt.loadRes(out_pred)
    coco_eval = COCOeval(coco_gt, coco_dt, iouType='bbox')
    coco_eval.evaluate()
    coco_eval.accumulate()
    coco_eval.summarize()
    
    stats = {
        "mAP@0.5": float(coco_eval.stats[1]),
        "mAP@0.5:0.95": float(coco_eval.stats[0]),
        "mAP@0.75": float(coco_eval.stats[2]),
    }
    
    # Per-class AP
    precisions = coco_eval.eval['precision']
    cat_ids = coco_gt.getCatIds()
    per_class = {}
    for i, cid in enumerate(cat_ids):
        info = coco_gt.loadCats(cid)[0]
        cls_prec = precisions[:, :, i, 0, 2]
        valid = cls_prec[cls_prec > -1]
        per_class[info['name']] = round(float(valid.mean()), 4) if len(valid) > 0 else 0.0
    
    print(f"\n[RESULTS]")
    print(f"  mAP@0.5      : {stats['mAP@0.5']:.4f}")
    print(f"  mAP@0.5:0.95 : {stats['mAP@0.5:0.95']:.4f}")
    print(f"  mAP@0.75     : {stats['mAP@0.75']:.4f}")
    print(f"\n  Per-class AP:")
    for cls, ap in sorted(per_class.items(), key=lambda x: -x[1]):
        print(f"    {cls:20s}: {ap:.4f}")
    
    # Save
    summary = {"overall": stats, "per_class": per_class}
    with open("map_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\nSaved to map_summary.json")
    
    return stats, per_class


# ═════════════════════════════════════════════════════════════
# 3. DOMAIN GAP BENCHMARK
# ═════════════════════════════════════════════════════════════
def domain_gap_benchmark(model, visdrone_dir, fg_dir):
    print(f"\n{'='*70}")
    print(f"DOMAIN GAP BENCHMARK")
    print(f"{'='*70}")
    
    def run_domain(image_dir, label):
        paths = sorted(Path(image_dir).glob("*"))
        paths = [p for p in paths if p.suffix.lower() in {".jpg", ".jpeg", ".png"}]
        
        confs = []
        counts = []
        class_dist = defaultdict(int)
        
        print(f"\n  Processing {len(paths)} images from {label}...")
        for p in paths:
            res = model(str(p), verbose=False, conf=CONFIDENCE_THRESHOLD)[0]
            counts.append(len(res.boxes))
            for box in res.boxes:
                confs.append(float(box.conf))
                class_dist[model.names[int(box.cls)]] += 1
        
        return {
            "label": label,
            "images": len(paths),
            "total_dets": sum(counts),
            "mean_dets_per_img": np.mean(counts),
            "mean_conf": np.mean(confs) if confs else 0.0,
            "median_conf": np.median(confs) if confs else 0.0,
            "std_conf": np.std(confs) if confs else 0.0,
            "class_dist": dict(class_dist),
        }
    
    vd = run_domain(visdrone_dir, "VisDrone (Real)")
    fg = run_domain(fg_dir, "FlightGear (Synthetic)")
    
    print(f"\n{'='*70}")
    print(f"  {'Metric':<30} {'VisDrone':>12} {'FlightGear':>12}")
    print(f"  {'-'*56}")
    print(f"  {'Images':<30} {vd['images']:>12} {fg['images']:>12}")
    print(f"  {'Total detections':<30} {vd['total_dets']:>12} {fg['total_dets']:>12}")
    print(f"  {'Mean detections/image':<30} {vd['mean_dets_per_img']:>12.2f} {fg['mean_dets_per_img']:>12.2f}")
    print(f"  {'Mean confidence':<30} {vd['mean_conf']:>12.3f} {fg['mean_conf']:>12.3f}")
    print(f"  {'Median confidence':<30} {vd['median_conf']:>12.3f} {fg['median_conf']:>12.3f}")
    print(f"  {'Std confidence':<30} {vd['std_conf']:>12.3f} {fg['std_conf']:>12.3f}")
    print(f"{'='*70}")
    
    # LaTeX table
    latex = f"""
\\begin{{table}}[h]
\\centering
\\caption{{Domain gap comparison: detection statistics on VisDrone (real) vs FlightGear (synthetic) imagery.}}
\\label{{tab:domaingap}}
\\begin{{tabular}}{{@{{}}lcc@{{}}}}
\\toprule
\\textbf{{Metric}} & \\textbf{{VisDrone}} & \\textbf{{FlightGear}} \\\\
\\midrule
Images evaluated & {vd['images']} & {fg['images']} \\\\
Total detections & {vd['total_dets']} & {fg['total_dets']} \\\\
Mean detections/image & {vd['mean_dets_per_img']:.2f} & {fg['mean_dets_per_img']:.2f} \\\\
Mean confidence & {vd['mean_conf']:.3f} & {fg['mean_conf']:.3f} \\\\
Median confidence & {vd['median_conf']:.3f} & {fg['median_conf']:.3f} \\\\
\\bottomrule
\\end{{tabular}}
\\end{{table}}
"""
    print("\n[LATEX TABLE]\n" + latex)
    
    with open("domain_gap_report.json", "w") as f:
        json.dump({"visdrone": vd, "flightgear": fg}, f, indent=2)
    
    return vd, fg


# ═════════════════════════════════════════════════════════════
# 4. LATENCY BENCHMARK (Robust: Median ± IQR)
# ═════════════════════════════════════════════════════════════
def latency_benchmark(model, image_path=None, screencap=False, monitor=None, n_runs=100, warmup=10):
    print(f"\n{'='*70}")
    print(f"LATENCY BENCHMARK (Robust Statistics)")
    print(f"{'='*70}")
    
    times = []
    
    if not screencap:
        if not image_path or not Path(image_path).exists():
            print("ERROR: Provide --latency-image for static benchmark")
            return None
        
        img = cv2.imread(str(image_path))
        img = cv2.resize(img, (INFERENCE_SIZE, INFERENCE_SIZE))
        
        # Warmup
        print(f"Warmup: {warmup} runs...")
        for _ in range(warmup):
            model(img, verbose=False)
        
        print(f"Benchmarking static image ({n_runs} runs)...")
        for i in range(n_runs):
            t0 = time.perf_counter()
            model(img, verbose=False)
            t1 = time.perf_counter()
            times.append((t1 - t0) * 1000)
    
    else:
        if not MSS_AVAILABLE:
            print("ERROR: Install mss: pip install mss")
            return None
        
        sct = mss.MSS()
        if monitor is None:
            monitor = {"top": 100, "left": 100, "width": 800, "height": 600}
        
        # Warmup
        print(f"Warmup: {warmup} screen-capture runs...")
        for _ in range(warmup):
            frame = np.array(sct.grab(monitor))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            resized = cv2.resize(frame, (INFERENCE_SIZE, INFERENCE_SIZE))
            model(resized, verbose=False)
        
        print(f"Benchmarking screen-capture ({n_runs} runs)...")
        for i in range(n_runs):
            t0 = time.perf_counter()
            frame = np.array(sct.grab(monitor))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            resized = cv2.resize(frame, (INFERENCE_SIZE, INFERENCE_SIZE))
            model(resized, verbose=False)
            t1 = time.perf_counter()
            times.append((t1 - t0) * 1000)
    
    stats = robust_stats(times, "Latency")
    print_stats_table(stats)
    
    # LaTeX table
    latex = f"""
\\begin{{table}}[h]
\\centering
\\caption{{Inference latency comparison ($n={stats['n']}$ runs, {warmup}-run warmup). Reported as median $\pm$ IQR.}}
\\label{{tab:latency}}
\\begin{{tabular}}{{@{{}}lccccc@{{}}}}
\\toprule
\\textbf{{Configuration}} & \\textbf{{Median (ms)}} & \\textbf{{IQR (ms)}} & \\textbf{{Mean (ms)}} & \\textbf{{95th \\%ile}} & \\textbf{{Max (ms)}} \\\\
\\midrule
{stats['label']} & {stats['median_ms']:.1f} & {stats['iqr_ms']:.1f} & {stats['mean_ms']:.1f} & {stats['p95_ms']:.1f} & {stats['max_ms']:.1f} \\\\
\\bottomrule
\\end{{tabular}}
\\end{{table}}
"""
    print("\n[LATEX TABLE]\n" + latex)
    
    with open("latency_results.json", "w") as f:
        json.dump(stats, f, indent=2)
    
    return stats


# ═════════════════════════════════════════════════════════════
# MAIN CLI
# ═════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="PHI-DRONE Analysis Suite")
    parser.add_argument("--analyze", help="Path to detection CSV to analyze")
    parser.add_argument("--map", action="store_true", help="Run COCO mAP evaluation")
    parser.add_argument("--map-images", help="Directory of labeled images for mAP")
    parser.add_argument("--map-gt", help="COCO ground-truth JSON for mAP")
    parser.add_argument("--domain-gap", action="store_true", help="Run domain gap benchmark")
    parser.add_argument("--visdrone-dir", help="VisDrone image directory")
    parser.add_argument("--fg-dir", help="FlightGear snapshot directory")
    parser.add_argument("--latency", action="store_true", help="Run latency benchmark")
    parser.add_argument("--latency-image", help="Static image for latency benchmark")
    parser.add_argument("--latency-screencap", action="store_true", help="Use screen capture for latency")
    parser.add_argument("--latency-monitor", help="mss monitor JSON for screen capture")
    parser.add_argument("--runs", type=int, default=100, help="Number of benchmark runs")
    parser.add_argument("--warmup", type=int, default=10, help="Warmup runs")
    args = parser.parse_args()
    
    model = None
    if args.map or args.domain_gap or args.latency:
        model = load_model()
    
    if args.analyze:
        analyze_mission(args.analyze)
    
    if args.map:
        if not args.map_images or not args.map_gt:
            print("ERROR: --map requires --map-images and --map-gt")
            sys.exit(1)
        evaluate_map(model, args.map_images, args.map_gt)
    
    if args.domain_gap:
        if not args.visdrone_dir or not args.fg_dir:
            print("ERROR: --domain-gap requires --visdrone-dir and --fg-dir")
            sys.exit(1)
        domain_gap_benchmark(model, args.visdrone_dir, args.fg_dir)
    
    if args.latency:
        mon = None
        if args.latency_monitor:
            import json
            mon = json.loads(args.latency_monitor)
        latency_benchmark(model, args.latency_image, args.latency_screencap, mon, args.runs, args.warmup)


if __name__ == "__main__":
    main()