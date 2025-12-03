#!/usr/bin/env python3
"""
Modern Web-Based Spine Movement Dashboard
With real-time spine curve visualization and daily average analysis

IMPORTANT: This dashboard correctly groups sensor readings by "reading cycle".
The firmware reads sensors sequentially through the MUX (3-4ms apart), so each sensor
gets a slightly different timestamp. A "reading cycle" is a ~30ms window where all 
sensors are read in sequence. We group by reading cycle (not exact timestamp) to 
reconstruct the complete spine position at each moment in time.

Example CSV pattern:
  505,0,...  <- Cycle 1 start
  509,2,...
  ...
  529,7,...  <- Cycle 1 end
  1105,0,... <- Cycle 2 start (sensors repeat, new cycle detected)
"""

import http.server
import socketserver
import webbrowser
import json
import csv
import math
import threading
import time
from pathlib import Path
from collections import defaultdict

PORT = 8765

def compute_angle_from_accel(ax, ay, az):
    """CurvImu algorithm for angle calculation"""
    scale = 8192.0
    xRaw = ax if ax != 0 else 10
    
    deadzoneRaw = int(0.03 * scale)
    yUsed = 0 if abs(ay) < deadzoneRaw else ay
    zUsed = 0 if abs(az) < deadzoneRaw else az
    
    angle = math.atan2(math.sqrt(yUsed * yUsed + zUsed * zUsed), abs(xRaw)) * 180.0 / math.pi
    
    if xRaw < 0:
        angle = 180.0 - angle
    if az < 0:
        angle = -angle
    
    if abs(xRaw) > int(0.98 * scale) and abs(yUsed) < deadzoneRaw and abs(zUsed) < deadzoneRaw:
        angle = 180.0 if xRaw < 0 else 0.0
    
    return angle


def build_spine_from_angles(angles, spacing_cm=9.0):
    """Build spine curve from angles using CurvImu algorithm"""
    if not angles:
        return []
    
    n = len(angles)
    radius = spacing_cm / 100.0  # Convert to meters
    
    # Reverse angles
    angles_rev = list(reversed(angles))
    angles_centered = [a - 0.0 for a in angles_rev]
    
    # Backward integration
    locs = []
    curr_x = 0.0
    curr_y = -float(n) * radius * 0.5
    
    for i in range(n - 1, -1, -1):
        rad = math.radians(angles_centered[i])
        dx = radius * math.sin(rad)
        dy = radius * math.cos(rad)
        curr_x += dx
        curr_y += dy
        locs.append((curr_x, curr_y))
    
    locs = list(reversed(locs))
    
    # Center
    if locs:
        avg_x = sum(loc[0] for loc in locs) / len(locs)
        avg_y = sum(loc[1] for loc in locs) / len(locs)
        locs = [(x - avg_x, y - avg_y) for x, y in locs]
    
    # Convert to cm
    return [(x * 100, y * 100) for x, y in locs]


def load_and_analyze_csv(file_path):
    """Load CSV and compute comprehensive metrics"""
    print(f"📂 Loading: {file_path}")
    
    # Load all data
    rows = []
    with open(file_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                ts = int(row['Timestamp_ms'])
                sid = int(row['Sensor_ID'])
                ax = int(row['Accel_X'])
                ay = int(row['Accel_Y'])
                az = int(row['Accel_Z'])
                rows.append((ts, sid, ax, ay, az))
            except:
                continue
    
    if not rows:
        return None
    
    print(f"✓ Loaded {len(rows)} samples")
    
    # Chop first and last 5 minutes of data
    if len(rows) > 0:
        # Get first and last timestamps (first element of each tuple)
        timestamps = [row[0] for row in rows]
        first_timestamp = min(timestamps)
        last_timestamp = max(timestamps)
        
        # 5 minutes in milliseconds
        FIVE_MINUTES_MS = 5 * 60 * 1000
        
        # Calculate cutoff times
        start_cutoff = first_timestamp + FIVE_MINUTES_MS
        end_cutoff = last_timestamp - FIVE_MINUTES_MS
        
        # Filter rows to exclude first and last 5 minutes
        original_count = len(rows)
        rows = [row for row in rows if start_cutoff <= row[0] <= end_cutoff]
        
        removed_count = original_count - len(rows)
        if removed_count > 0:
            print(f"✓ Removed {removed_count} samples (first/last 5 minutes)")
            print(f"✓ Remaining {len(rows)} samples")
        else:
            print(f"⚠ Warning: Data duration is less than 10 minutes, no data removed")
    
    if not rows:
        return None
    
    # CRITICAL FIX: Sensors are read sequentially through MUX (3-4ms apart)
    # We need to group by "reading cycle" not exact timestamp
    # A reading cycle is ~30ms window where all sensors are read in sequence
    
    # Group into reading cycles based on timestamp proximity
    reading_cycles = []
    current_cycle = {}
    current_cycle_start_time = None
    CYCLE_WINDOW_MS = 100  # Max time between first and last sensor in a cycle
    
    for ts, sid, ax, ay, az in rows:
        angle = compute_angle_from_accel(ax, ay, az)
        
        # Start new cycle if:
        # 1. This is the first reading, OR
        # 2. We've seen this sensor ID already in current cycle, OR
        # 3. Too much time has passed since cycle start
        if (current_cycle_start_time is None or 
            sid in current_cycle or 
            ts - current_cycle_start_time > CYCLE_WINDOW_MS):
            
            # Save previous cycle if it has data
            if current_cycle:
                # Use average timestamp of the cycle
                avg_time = sum(current_cycle.values(), start=(0, []))[0] / len(current_cycle) if current_cycle else current_cycle_start_time
                reading_cycles.append((current_cycle_start_time, dict(current_cycle)))
            
            # Start new cycle
            current_cycle = {sid: (ts, angle)}
            current_cycle_start_time = ts
        else:
            # Add to current cycle
            current_cycle[sid] = (ts, angle)
    
    # Don't forget the last cycle
    if current_cycle:
        reading_cycles.append((current_cycle_start_time, dict(current_cycle)))
    
    print(f"✓ {len(reading_cycles)} reading cycles detected")
    
    # Build time_frames from reading cycles
    time_frames = {}
    for cycle_time, cycle_data in reading_cycles:
        time_frames[cycle_time] = {sid: angle for sid, (ts, angle) in cycle_data.items()}
    
    # Build sensor_data for per-sensor tracking
    sensor_data = defaultdict(list)
    all_sensor_ids = set()
    for cycle_time, cycle_data in reading_cycles:
        for sid, (ts, angle) in cycle_data.items():
            all_sensor_ids.add(sid)
            sensor_data[sid].append({'time': ts/1000.0, 'angle': angle, 'ts_ms': ts})
    
    print(f"✓ {len(all_sensor_ids)} sensors detected")
    
    # Calculate spine curves over time
    # CRITICAL: Each timestamp represents ONE complete spine position with ALL sensors
    timestamps = sorted(time_frames.keys())
    spine_curves = []
    sensor_order = sorted(sensor_data.keys())
    
    # Sample every 100ms for visualization (reduce data size)
    sampled_timestamps = timestamps[::max(1, len(timestamps)//500)]  # Max 500 frames
    
    for ts in sampled_timestamps:
        if ts in time_frames:
            # Get ALL sensor angles for this single timestamp
            # This represents the complete spine position at this moment
            frame_angles = [time_frames[ts].get(sid, 0) for sid in sensor_order]
            if any(frame_angles):
                spine_points = build_spine_from_angles(frame_angles)
                spine_curves.append({
                    'time': ts/1000.0,
                    'points': spine_points,
                    'angles': frame_angles
                })
    
    print(f"✓ Generated {len(spine_curves)} spine curve frames")
    
    # Calculate average spine position (average angles across all time)
    avg_angles = []
    for sid in sensor_order:
        angles_for_sensor = [d['angle'] for d in sensor_data[sid]]
        avg_angles.append(sum(angles_for_sensor) / len(angles_for_sensor) if angles_for_sensor else 0)
    
    avg_spine = build_spine_from_angles(avg_angles)
    print(f"✓ Computed average spine position")
    
    # Calculate metrics
    all_angles = []
    sensor_roms = {}
    
    for sid, data in sensor_data.items():
        angles = [d['angle'] for d in data]
        all_angles.extend(angles)
        sensor_roms[sid] = max(angles) - min(angles) if angles else 0
    
    duration = (timestamps[-1] - timestamps[0]) / 1000.0
    total_rom = max(all_angles) - min(all_angles) if all_angles else 0
    
    # Section ROMs
    n = len(sensor_order)
    upper = sensor_order[:n//3]
    middle = sensor_order[n//3:2*n//3]
    lower = sensor_order[2*n//3:]
    
    upper_angles = [a for sid in upper for a in [d['angle'] for d in sensor_data[sid]]]
    middle_angles = [a for sid in middle for a in [d['angle'] for d in sensor_data[sid]]]
    lower_angles = [a for sid in lower for a in [d['angle'] for d in sensor_data[sid]]]
    
    upper_rom = max(upper_angles) - min(upper_angles) if upper_angles else 0
    middle_rom = max(middle_angles) - min(middle_angles) if middle_angles else 0
    lower_rom = max(lower_angles) - min(lower_angles) if lower_angles else 0
    
    # Calculate max curvature (max distance from straight line)
    max_curvature = 0
    for curve in spine_curves:
        if len(curve['points']) >= 2:
            # Calculate deviation from straight line
            first = curve['points'][0]
            last = curve['points'][-1]
            for point in curve['points'][1:-1]:
                # Distance from point to line between first and last
                dist = abs((last[1]-first[1])*point[0] - (last[0]-first[0])*point[1] + last[0]*first[1] - last[1]*first[0]) / \
                       math.sqrt((last[1]-first[1])**2 + (last[0]-first[0])**2) if first != last else 0
                max_curvature = max(max_curvature, dist)
    
    print("✓ Analysis complete")
    
    return {
        'filename': Path(file_path).name,
        'sensors': len(sensor_data),
        'samples': len(rows),
        'duration': duration,
        'total_rom': total_rom,
        'upper_rom': upper_rom,
        'middle_rom': middle_rom,
        'lower_rom': lower_rom,
        'avg_angle': sum(all_angles) / len(all_angles) if all_angles else 0,
        'movement_score': total_rom / duration if duration > 0 else 0,
        'max_curvature': max_curvature,
        'sensor_roms': {str(sid): rom for sid, rom in sensor_roms.items()},
        'spine_curves': spine_curves,
        'avg_spine': avg_spine,
        'sensor_order': [str(s) for s in sensor_order]
    }


HTML_TEMPLATE = """<!DOCTYPE html>
<html>
<head>
    <title>Spine Movement Dashboard</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <script src="https://cdn.plot.ly/plotly-2.26.0.min.js"></script>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800;900&display=swap');
        
        * { 
            margin: 0; 
            padding: 0; 
            box-sizing: border-box; 
        }
        
        body {
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
            background: #0a0a0a;
            background-image: 
                radial-gradient(at 27% 37%, hsla(215, 98%, 61%, 0.15) 0px, transparent 50%),
                radial-gradient(at 97% 21%, hsla(125, 98%, 72%, 0.1) 0px, transparent 50%),
                radial-gradient(at 52% 99%, hsla(354, 98%, 61%, 0.1) 0px, transparent 50%),
                radial-gradient(at 10% 29%, hsla(256, 96%, 67%, 0.15) 0px, transparent 50%),
                radial-gradient(at 97% 96%, hsla(38, 60%, 74%, 0.1) 0px, transparent 50%),
                radial-gradient(at 33% 50%, hsla(222, 67%, 73%, 0.1) 0px, transparent 50%),
                radial-gradient(at 79% 53%, hsla(343, 68%, 79%, 0.1) 0px, transparent 50%);
            min-height: 100vh;
            padding: 30px;
            position: relative;
            overflow-x: hidden;
        }
        
        body::before {
            content: '';
            position: fixed;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: linear-gradient(180deg, transparent 0%, rgba(10,10,10,0.5) 100%);
            pointer-events: none;
            z-index: 0;
        }
        
        .container {
            max-width: 1600px;
            margin: 0 auto;
            position: relative;
            z-index: 1;
        }
        
        .header {
            background: rgba(255, 255, 255, 0.03);
            backdrop-filter: blur(20px) saturate(180%);
            -webkit-backdrop-filter: blur(20px) saturate(180%);
            border-radius: 24px;
            padding: 50px;
            margin-bottom: 30px;
            box-shadow: 
                0 8px 32px 0 rgba(0, 0, 0, 0.37),
                inset 0 1px 0 0 rgba(255, 255, 255, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.08);
            position: relative;
            overflow: hidden;
        }
        
        .header::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 200%;
            height: 100%;
            background: linear-gradient(90deg, 
                transparent, 
                rgba(255,255,255,0.03), 
                transparent);
            animation: shimmer 3s infinite;
        }
        
        @keyframes shimmer {
            0% { transform: translateX(-100%); }
            100% { transform: translateX(100%); }
        }
        
        h1 {
            background: linear-gradient(135deg, #3b82f6 0%, #8b5cf6 50%, #ec4899 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            font-size: 48px;
            font-weight: 900;
            margin-bottom: 12px;
            letter-spacing: -2px;
            position: relative;
            z-index: 1;
        }
        
        .subtitle {
            color: rgba(255, 255, 255, 0.6);
            font-size: 16px;
            margin-bottom: 30px;
            font-weight: 500;
            position: relative;
            z-index: 1;
        }
        
        .file-upload {
            margin-top: 20px;
            padding: 40px;
            background: rgba(255, 255, 255, 0.02);
            backdrop-filter: blur(10px);
            border-radius: 20px;
            border: 2px dashed rgba(139, 92, 246, 0.3);
            text-align: center;
            transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
            cursor: pointer;
            position: relative;
            z-index: 1;
        }
        
        .file-upload:hover {
            border-color: rgba(139, 92, 246, 0.6);
            transform: translateY(-4px);
            background: rgba(255, 255, 255, 0.04);
            box-shadow: 0 20px 60px rgba(139, 92, 246, 0.2);
        }
        
        input[type="file"] { display: none; }
        
        .upload-btn {
            background: linear-gradient(135deg, #3b82f6 0%, #8b5cf6 100%);
            color: white;
            padding: 18px 45px;
            border-radius: 14px;
            cursor: pointer;
            display: inline-block;
            font-weight: 700;
            font-size: 16px;
            transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
            box-shadow: 
                0 10px 30px rgba(139, 92, 246, 0.4),
                inset 0 1px 0 rgba(255,255,255,0.2);
            border: none;
            position: relative;
            overflow: hidden;
        }
        
        .upload-btn::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.2), transparent);
            transition: left 0.5s;
        }
        
        .upload-btn:hover::before {
            left: 100%;
        }
        
        .upload-btn:hover {
            transform: translateY(-4px) scale(1.02);
            box-shadow: 0 20px 50px rgba(139, 92, 246, 0.5);
        }
        
        .metrics-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
            gap: 20px;
            margin-bottom: 25px;
        }
        
        .metric-card {
            background: rgba(255, 255, 255, 0.04);
            backdrop-filter: blur(20px) saturate(180%);
            -webkit-backdrop-filter: blur(20px) saturate(180%);
            border-radius: 20px;
            padding: 32px;
            box-shadow: 
                0 8px 32px 0 rgba(0, 0, 0, 0.37),
                inset 0 1px 0 0 rgba(255, 255, 255, 0.1);
            transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
            border: 1px solid rgba(255, 255, 255, 0.08);
            position: relative;
            overflow: hidden;
        }
        
        .metric-card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 3px;
            background: linear-gradient(90deg, #3b82f6 0%, #8b5cf6 50%, #ec4899 100%);
            opacity: 0;
            transition: opacity 0.3s;
        }
        
        .metric-card:hover {
            transform: translateY(-6px) scale(1.02);
            box-shadow: 
                0 20px 60px rgba(0, 0, 0, 0.5),
                inset 0 1px 0 0 rgba(255, 255, 255, 0.2);
            border-color: rgba(255, 255, 255, 0.15);
        }
        
        .metric-card:hover::before {
            opacity: 1;
        }
        
        .metric-label {
            color: rgba(255, 255, 255, 0.5);
            font-size: 12px;
            text-transform: uppercase;
            letter-spacing: 1.5px;
            margin-bottom: 14px;
            font-weight: 700;
        }
        
        .metric-value {
            color: #ffffff;
            font-size: 42px;
            font-weight: 900;
            line-height: 1;
            text-shadow: 0 2px 10px rgba(0,0,0,0.3);
        }
        
        .metric-unit {
            background: linear-gradient(135deg, #3b82f6 0%, #8b5cf6 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            font-size: 22px;
            margin-left: 6px;
            font-weight: 800;
        }
        
        .metric-subtitle {
            color: rgba(255, 255, 255, 0.4);
            font-size: 11px;
            margin-top: 10px;
            font-weight: 500;
            letter-spacing: 0.5px;
        }
        
        .charts-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 25px;
            margin-bottom: 25px;
        }
        
        .chart-card {
            background: rgba(255, 255, 255, 0.04);
            backdrop-filter: blur(20px) saturate(180%);
            -webkit-backdrop-filter: blur(20px) saturate(180%);
            border-radius: 24px;
            padding: 35px;
            box-shadow: 
                0 8px 32px 0 rgba(0, 0, 0, 0.37),
                inset 0 1px 0 0 rgba(255, 255, 255, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.08);
            transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
        }
        
        .chart-card:hover {
            transform: translateY(-4px);
            box-shadow: 
                0 20px 60px rgba(0, 0, 0, 0.5),
                inset 0 1px 0 0 rgba(255, 255, 255, 0.15);
            border-color: rgba(255, 255, 255, 0.12);
        }
        
        .spine-viz-container {
            background: rgba(255, 255, 255, 0.04);
            backdrop-filter: blur(20px) saturate(180%);
            -webkit-backdrop-filter: blur(20px) saturate(180%);
            border-radius: 24px;
            padding: 35px;
            margin-bottom: 30px;
            box-shadow: 
                0 8px 32px 0 rgba(0, 0, 0, 0.37),
                inset 0 1px 0 0 rgba(255, 255, 255, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.08);
        }
        
        .chart-title {
            font-size: 22px;
            font-weight: 800;
            color: #ffffff;
            margin-bottom: 25px;
            display: flex;
            align-items: center;
            gap: 12px;
            letter-spacing: -0.5px;
        }
        
        .chart-icon {
            font-size: 28px;
            filter: drop-shadow(0 2px 4px rgba(0,0,0,0.3));
        }
        
        #loading {
            position: fixed;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            background: rgba(20, 20, 20, 0.95);
            backdrop-filter: blur(20px);
            -webkit-backdrop-filter: blur(20px);
            padding: 60px 80px;
            border-radius: 24px;
            box-shadow: 
                0 30px 90px rgba(0,0,0,0.8),
                inset 0 1px 0 rgba(255,255,255,0.1);
            border: 1px solid rgba(255,255,255,0.1);
            display: none;
            z-index: 1000;
            text-align: center;
        }
        
        .spinner {
            border: 6px solid rgba(255,255,255,0.1);
            border-top: 6px solid #8b5cf6;
            border-radius: 50%;
            width: 70px;
            height: 70px;
            animation: spin 0.6s linear infinite;
            margin: 0 auto 30px;
            box-shadow: 0 0 30px rgba(139, 92, 246, 0.3);
        }
        
        @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
        }
        
        .loading-text {
            color: #ffffff;
            font-weight: 700;
            font-size: 18px;
            text-shadow: 0 2px 10px rgba(0,0,0,0.5);
        }
        
        .playback-controls {
            display: flex;
            align-items: center;
            gap: 18px;
            margin-top: 25px;
            padding: 24px;
            background: rgba(255, 255, 255, 0.03);
            backdrop-filter: blur(10px);
            border-radius: 16px;
            border: 1px solid rgba(255, 255, 255, 0.06);
        }
        
        .playback-btn {
            background: linear-gradient(135deg, #3b82f6 0%, #8b5cf6 100%);
            color: white;
            border: none;
            padding: 14px 28px;
            border-radius: 12px;
            font-weight: 700;
            cursor: pointer;
            transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
            font-size: 15px;
            box-shadow: 
                0 10px 30px rgba(139, 92, 246, 0.3),
                inset 0 1px 0 rgba(255,255,255,0.2);
        }
        
        .playback-btn:hover {
            transform: translateY(-3px) scale(1.05);
            box-shadow: 0 15px 40px rgba(139, 92, 246, 0.5);
        }
        
        .slider {
            flex: 1;
            height: 8px;
            border-radius: 4px;
            background: rgba(255, 255, 255, 0.1);
            outline: none;
            -webkit-appearance: none;
            cursor: pointer;
        }
        
        .slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 24px;
            height: 24px;
            border-radius: 50%;
            background: linear-gradient(135deg, #3b82f6 0%, #8b5cf6 100%);
            cursor: pointer;
            box-shadow: 
                0 4px 16px rgba(139, 92, 246, 0.5),
                0 0 0 4px rgba(139, 92, 246, 0.2);
            transition: all 0.3s;
        }
        
        .slider::-webkit-slider-thumb:hover {
            transform: scale(1.2);
            box-shadow: 
                0 6px 20px rgba(139, 92, 246, 0.7),
                0 0 0 6px rgba(139, 92, 246, 0.3);
        }
        
        select {
            background: rgba(255, 255, 255, 0.05);
            color: #ffffff;
            border: 1px solid rgba(255, 255, 255, 0.1);
            padding: 10px 16px;
            border-radius: 10px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s;
            font-size: 14px;
        }
        
        select:hover {
            background: rgba(255, 255, 255, 0.08);
            border-color: rgba(139, 92, 246, 0.5);
        }
        
        #timeDisplay {
            font-weight: 700;
            color: #ffffff;
            min-width: 130px;
            font-size: 15px;
            text-shadow: 0 2px 8px rgba(0,0,0,0.3);
        }
        
        .file-list-item {
            background: rgba(255, 255, 255, 0.05);
            padding: 12px 18px;
            border-radius: 10px;
            margin-bottom: 8px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border: 1px solid rgba(255, 255, 255, 0.08);
            transition: all 0.3s;
        }
        
        .file-list-item:hover {
            background: rgba(255, 255, 255, 0.08);
            border-color: rgba(139, 92, 246, 0.4);
        }
        
        .file-list-item.selected {
            background: rgba(139, 92, 246, 0.2);
            border-color: rgba(139, 92, 246, 0.6);
        }
        
        .file-name {
            color: rgba(255, 255, 255, 0.9);
            font-weight: 600;
            font-size: 13px;
        }
        
        .file-metrics {
            color: rgba(255, 255, 255, 0.5);
            font-size: 11px;
            font-weight: 500;
        }
        
        .comparison-toggle {
            background: rgba(139, 92, 246, 0.15);
            color: #8b5cf6;
            border: 1px solid rgba(139, 92, 246, 0.3);
            padding: 8px 16px;
            border-radius: 8px;
            font-weight: 700;
            font-size: 12px;
            cursor: pointer;
            transition: all 0.3s;
        }
        
        .comparison-toggle:hover {
            background: rgba(139, 92, 246, 0.3);
            border-color: rgba(139, 92, 246, 0.5);
        }
        
        @media (max-width: 1200px) {
            .charts-grid { grid-template-columns: 1fr; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🏥 Spine Movement Dashboard</h1>
            <p class="subtitle">Advanced spine analytics powered by medical-grade CurvImu algorithm</p>
            
            <div class="file-upload">
                <label for="csvFile" class="upload-btn">📁 Select CSV Files (Multiple)</label>
                <input type="file" id="csvFile" accept=".csv" multiple onchange="uploadFiles()">
                <div id="fileName" style="margin-top: 20px; color: rgba(255,255,255,0.8); font-weight: 700; font-size: 15px; text-shadow: 0 2px 8px rgba(0,0,0,0.3);"></div>
            </div>
            
            <div id="fileList" style="margin-top: 20px; display: none;"></div>
        </div>

        <div id="loading">
            <div class="spinner"></div>
            <p class="loading-text">Analyzing spine movement...</p>
        </div>

        <div id="dashboard" style="display: none;">
            <div class="metrics-grid">
                <div class="metric-card">
                    <div class="metric-label">⚡ Total ROM</div>
                    <div class="metric-value"><span id="totalRom">--</span><span class="metric-unit">°</span></div>
                    <div class="metric-subtitle">Full spine flexibility</div>
                </div>
                <div class="metric-card">
                    <div class="metric-label">🔴 Upper Spine</div>
                    <div class="metric-value"><span id="upperRom">--</span><span class="metric-unit">°</span></div>
                    <div class="metric-subtitle">Cervical/Upper thoracic</div>
                </div>
                <div class="metric-card">
                    <div class="metric-label">🟡 Middle Spine</div>
                    <div class="metric-value"><span id="middleRom">--</span><span class="metric-unit">°</span></div>
                    <div class="metric-subtitle">Mid thoracic region</div>
                </div>
                <div class="metric-card">
                    <div class="metric-label">🔵 Lower Spine</div>
                    <div class="metric-value"><span id="lowerRom">--</span><span class="metric-unit">°</span></div>
                    <div class="metric-subtitle">Lumbar region</div>
                </div>
                <div class="metric-card">
                    <div class="metric-label">⏱ Duration</div>
                    <div class="metric-value"><span id="duration">--</span><span class="metric-unit">s</span></div>
                    <div class="metric-subtitle" id="durationMin"></div>
                </div>
                <div class="metric-card">
                    <div class="metric-label">📡 Sensors</div>
                    <div class="metric-value"><span id="sensors">--</span></div>
                    <div class="metric-subtitle"><span id="samples">--</span> samples</div>
                </div>
                <div class="metric-card">
                    <div class="metric-label">🎯 Movement Score</div>
                    <div class="metric-value"><span id="score">--</span><span class="metric-unit">°/s</span></div>
                    <div class="metric-subtitle" id="activityLevel"></div>
                </div>
                <div class="metric-card">
                    <div class="metric-label">📏 Max Curvature</div>
                    <div class="metric-value"><span id="curvature">--</span><span class="metric-unit">cm</span></div>
                    <div class="metric-subtitle">Peak spine bend</div>
                </div>
            </div>

            <!-- Spine Visualization -->
            <div class="spine-viz-container">
                <div class="chart-title"><span class="chart-icon">🦴</span> Live Spine Visualization</div>
                <div id="spineViz" style="height: 500px;"></div>
                
                <div class="playback-controls">
                    <button class="playback-btn" id="playBtn" onclick="togglePlayback()">▶️ Play</button>
                    <input type="range" class="slider" id="timeSlider" min="0" max="100" value="0" 
                           oninput="seekToFrame(this.value)">
                    <span id="timeDisplay" style="font-weight: 700; color: #667eea; min-width: 120px;">0.0s / 0.0s</span>
                    <select id="speedControl" onchange="changeSpeed()" style="padding: 8px; border-radius: 8px; border: 2px solid #ddd; font-weight: 600;">
                        <option value="0.5">0.5x</option>
                        <option value="1" selected>1.0x</option>
                        <option value="2">2.0x</option>
                        <option value="5">5.0x</option>
                    </select>
                </div>
            </div>

            <!-- Average Spine Position / Comparison -->
            <div class="spine-viz-container">
                <div class="chart-title">
                    <span class="chart-icon">📊</span> 
                    <span id="avgSpineTitle">Average Spine Position (Session)</span>
                </div>
                <div id="avgSpineViz" style="height: 400px;"></div>
                <p id="avgSpineSubtitle" style="color: rgba(255,255,255,0.5); margin-top: 20px; font-size: 14px; font-weight: 500; letter-spacing: 0.3px;">
                    📍 This shows your average spine curvature across the entire recording session.
                </p>
            </div>

            <div class="charts-grid">
                <div class="chart-card">
                    <div class="chart-title"><span class="chart-icon">📊</span> ROM Distribution</div>
                    <div id="romChart" style="height: 320px;"></div>
                </div>
                <div class="chart-card">
                    <div class="chart-title"><span class="chart-icon">📈</span> Angles Over Time</div>
                    <div id="timeChart" style="height: 320px;"></div>
                </div>
            </div>

            <div class="chart-card">
                <div class="chart-title"><span class="chart-icon">🎯</span> Per-Sensor ROM Analysis</div>
                <div id="sensorChart" style="height: 350px;"></div>
            </div>
        </div>
    </div>

    <script>
        let allDatasets = [];  // Array of all uploaded datasets
        let currentData = null;  // Currently displayed dataset
        let currentDatasetIndex = 0;
        let animationTimer = null;
        let currentFrame = 0;
        let isPlaying = false;
        let playbackSpeed = 1.0;

        async function uploadFiles() {
            console.log('📁 uploadFiles() called');
            
            const input = document.getElementById('csvFile');
            const files = input.files;
            
            console.log('   Files selected:', files.length);
            
            if (files.length === 0) {
                console.log('   ❌ No files selected');
                return;
            }

            document.getElementById('fileName').textContent = `📄 ${files.length} file(s) selected`;
            document.getElementById('loading').style.display = 'block';
            
            allDatasets = [];  // Reset datasets

            try {
                // Upload each file
                for (let i = 0; i < files.length; i++) {
                    const file = files[i];
                    console.log(`   Processing file ${i+1}/${files.length}: ${file.name}`);
                    
                    const formData = new FormData();
                    formData.append('file', file);
                    
                    const response = await fetch('/upload', {
                        method: 'POST',
                        body: formData
                    });
                    
                    console.log(`   Response status for ${file.name}:`, response.status);
                    
                    if (!response.ok) {
                        const text = await response.text();
                        console.error(`   Server error for ${file.name}:`, text);
                        throw new Error(`Failed to analyze ${file.name}: ${response.status}`);
                    }
                    
                    const data = await response.json();
                    console.log(`   ✅ Analyzed ${file.name}`);
                    
                    allDatasets.push(data);
                }
                
                console.log(`✅ All ${allDatasets.length} files processed`);
                
                // Display file list
                displayFileList();
                
                // Show comparison or single view
                if (allDatasets.length > 1) {
                    displayComparison();
                } else {
                    currentData = allDatasets[0];
                    displayResults(allDatasets[0]);
                }
                
            } catch (error) {
                console.error('❌ Error:', error);
                alert('Error analyzing files: ' + error.message + '\\n\\nCheck browser console (F12) and terminal for details.');
            } finally {
                document.getElementById('loading').style.display = 'none';
            }
        }
        
        function displayFileList() {
            const listDiv = document.getElementById('fileList');
            listDiv.innerHTML = '<div style="color: rgba(255,255,255,0.7); font-weight: 700; margin-bottom: 10px; font-size: 14px;">📁 Loaded Sessions:</div>';
            
            allDatasets.forEach((data, idx) => {
                const item = document.createElement('div');
                item.className = 'file-list-item' + (idx === currentDatasetIndex ? ' selected' : '');
                item.onclick = () => selectDataset(idx);
                
                item.innerHTML = `
                    <div>
                        <div class="file-name">${data.filename}</div>
                        <div class="file-metrics">${data.duration.toFixed(0)}s • ${data.total_rom.toFixed(1)}° ROM • ${data.sensors} sensors</div>
                    </div>
                    <button class="comparison-toggle" onclick="event.stopPropagation(); selectDataset(${idx})">View</button>
                `;
                
                listDiv.appendChild(item);
            });
            
            if (allDatasets.length > 1) {
                const compareBtn = document.createElement('button');
                compareBtn.className = 'upload-btn';
                compareBtn.style.marginTop = '15px';
                compareBtn.textContent = '📊 Show Comparison View';
                compareBtn.onclick = displayComparison;
                listDiv.appendChild(compareBtn);
            }
            
            listDiv.style.display = 'block';
        }
        
        function selectDataset(idx) {
            currentDatasetIndex = idx;
            currentData = allDatasets[idx];
            displayResults(currentData);
            displayFileList();  // Refresh to update selected state
        }
        
        function displayComparison() {
            console.log('📊 displayComparison() called with', allDatasets.length, 'datasets');
            
            if (allDatasets.length < 2) {
                alert('Need at least 2 files for comparison');
                return;
            }
            
            document.getElementById('dashboard').style.display = 'block';
            
            // Show aggregated metrics
            const avgROM = allDatasets.reduce((sum, d) => sum + d.total_rom, 0) / allDatasets.length;
            const avgUpper = allDatasets.reduce((sum, d) => sum + d.upper_rom, 0) / allDatasets.length;
            const avgMiddle = allDatasets.reduce((sum, d) => sum + d.middle_rom, 0) / allDatasets.length;
            const avgLower = allDatasets.reduce((sum, d) => sum + d.lower_rom, 0) / allDatasets.length;
            const totalDuration = allDatasets.reduce((sum, d) => sum + d.duration, 0);
            const totalSamples = allDatasets.reduce((sum, d) => sum + d.samples, 0);
            
            document.getElementById('totalRom').textContent = avgROM.toFixed(1);
            document.getElementById('upperRom').textContent = avgUpper.toFixed(1);
            document.getElementById('middleRom').textContent = avgMiddle.toFixed(1);
            document.getElementById('lowerRom').textContent = avgLower.toFixed(1);
            document.getElementById('duration').textContent = totalDuration.toFixed(1);
            document.getElementById('durationMin').textContent = (totalDuration / 60).toFixed(1) + ' minutes total';
            document.getElementById('sensors').textContent = allDatasets[0].sensors;
            document.getElementById('samples').textContent = totalSamples.toLocaleString();
            document.getElementById('score').textContent = (avgROM / (totalDuration / allDatasets.length)).toFixed(2);
            document.getElementById('curvature').textContent = (allDatasets.reduce((sum, d) => sum + d.max_curvature, 0) / allDatasets.length).toFixed(2);
            document.getElementById('activityLevel').textContent = `Avg across ${allDatasets.length} sessions`;
            
            // ROM comparison trend
            const sessionNames = allDatasets.map((d, i) => d.filename);
            Plotly.newPlot('romChart', [
                {
                    x: sessionNames,
                    y: allDatasets.map(d => d.upper_rom),
                    name: 'Upper',
                    type: 'bar',
                    marker: { color: '#3b82f6' }
                },
                {
                    x: sessionNames,
                    y: allDatasets.map(d => d.middle_rom),
                    name: 'Middle',
                    type: 'bar',
                    marker: { color: '#8b5cf6' }
                },
                {
                    x: sessionNames,
                    y: allDatasets.map(d => d.lower_rom),
                    name: 'Lower',
                    type: 'bar',
                    marker: { color: '#ec4899' }
                }
            ], {
                margin: { t: 30, b: 100, l: 60, r: 20 },
                yaxis: { 
                    title: 'ROM (°)',
                    gridcolor: 'rgba(255,255,255,0.08)',
                    color: 'rgba(255,255,255,0.7)'
                },
                xaxis: { 
                    tickangle: -45,
                    color: 'rgba(255,255,255,0.7)'
                },
                barmode: 'group',
                plot_bgcolor: 'rgba(0,0,0,0)',
                paper_bgcolor: 'rgba(0,0,0,0)',
                font: { family: 'Inter', color: 'rgba(255,255,255,0.7)' },
                legend: {
                    bgcolor: 'rgba(0,0,0,0.5)',
                    bordercolor: 'rgba(255,255,255,0.1)',
                    font: { color: '#ffffff' }
                }
            }, {responsive: true, displayModeBar: false});
            
            // Total ROM trend
            Plotly.newPlot('timeChart', [{
                x: sessionNames,
                y: allDatasets.map(d => d.total_rom),
                type: 'scatter',
                mode: 'lines+markers',
                line: { color: '#8b5cf6', width: 3 },
                marker: { size: 12, color: '#ec4899', line: { color: '#ffffff', width: 2 } },
                name: 'Total ROM Trend'
            }], {
                margin: { t: 30, b: 100, l: 60, r: 20 },
                yaxis: { 
                    title: 'Total ROM (°)',
                    gridcolor: 'rgba(255,255,255,0.08)',
                    color: 'rgba(255,255,255,0.7)'
                },
                xaxis: { 
                    tickangle: -45,
                    color: 'rgba(255,255,255,0.7)'
                },
                plot_bgcolor: 'rgba(0,0,0,0)',
                paper_bgcolor: 'rgba(0,0,0,0)',
                font: { family: 'Inter', color: 'rgba(255,255,255,0.7)' }
            }, {responsive: true, displayModeBar: false});
            
            // Movement score comparison
            Plotly.newPlot('sensorChart', [{
                x: sessionNames,
                y: allDatasets.map(d => d.movement_score),
                type: 'bar',
                marker: {
                    color: allDatasets.map(d => d.movement_score),
                    colorscale: [[0, '#3b82f6'], [0.5, '#8b5cf6'], [1, '#ec4899']],
                    showscale: true,
                    colorbar: {
                        title: 'Score',
                        titlefont: { color: '#ffffff' },
                        tickfont: { color: '#ffffff' },
                        bgcolor: 'rgba(0,0,0,0.5)',
                        bordercolor: 'rgba(255,255,255,0.1)'
                    }
                },
                text: allDatasets.map(d => d.movement_score.toFixed(2)),
                textposition: 'outside',
                textfont: { color: '#ffffff', family: 'Inter', weight: 'bold' }
            }], {
                margin: { t: 30, b: 100, l: 60, r: 100 },
                yaxis: { 
                    title: 'Movement Score (°/s)',
                    gridcolor: 'rgba(255,255,255,0.08)',
                    color: 'rgba(255,255,255,0.7)'
                },
                xaxis: { 
                    tickangle: -45,
                    color: 'rgba(255,255,255,0.7)'
                },
                plot_bgcolor: 'rgba(0,0,0,0)',
                paper_bgcolor: 'rgba(0,0,0,0)',
                font: { family: 'Inter', color: 'rgba(255,255,255,0.7)' }
            }, {responsive: true, displayModeBar: false});
            
            // Update titles for comparison view
            document.getElementById('avgSpineTitle').textContent = `Comparison: ${allDatasets.length} Sessions`;
            document.getElementById('avgSpineSubtitle').textContent = 
                `🔄 Overlaid average spine positions from all ${allDatasets.length} sessions. Each color represents a different recording.`;
            
            // Overlaid average spine positions
            drawComparisonSpines();
        }
        
        function drawComparisonSpines() {
            console.log('Drawing comparison spines...');
            
            const traces = [];
            const colors = ['#3b82f6', '#8b5cf6', '#ec4899', '#f59e0b', '#10b981', '#06b6d4'];
            
            allDatasets.forEach((data, idx) => {
                if (data.avg_spine && data.avg_spine.length > 0) {
                    const x = data.avg_spine.map(p => p[0]);
                    const y = data.avg_spine.map(p => p[1]);
                    
                    traces.push({
                        x: x,
                        y: y,
                        type: 'scatter',
                        mode: 'lines+markers',
                        name: data.filename,
                        line: { 
                            color: colors[idx % colors.length], 
                            width: 7,
                            shape: 'spline',
                            smoothing: 1.3
                        },
                        marker: { 
                            size: 12, 
                            color: colors[idx % colors.length],
                            line: { color: 'rgba(255,255,255,0.4)', width: 3 }
                        },
                        hovertemplate: `<b>${data.filename}</b><br>(%{x:.1f}, %{y:.1f}) cm<extra></extra>`
                    });
                }
            });
            
            // Reference lines
            traces.push({
                x: [0, 0],
                y: [-50, 50],
                type: 'scatter',
                mode: 'lines',
                line: { color: 'rgba(255,255,255,0.15)', width: 2, dash: 'dash' },
                showlegend: false,
                hoverinfo: 'skip'
            });
            
            Plotly.newPlot('avgSpineViz', traces, {
                margin: { t: 20, b: 50, l: 60, r: 60 },
                xaxis: { 
                    title: 'Lateral (cm)', 
                    gridcolor: 'rgba(255,255,255,0.05)',
                    color: 'rgba(255,255,255,0.7)',
                    zeroline: true,
                    zerolinecolor: 'rgba(255,255,255,0.2)',
                    zerolinewidth: 2
                },
                yaxis: { 
                    title: 'Vertical (cm)', 
                    gridcolor: 'rgba(255,255,255,0.05)',
                    scaleanchor: 'x',
                    scaleratio: 1,
                    color: 'rgba(255,255,255,0.7)',
                    zeroline: true,
                    zerolinecolor: 'rgba(255,255,255,0.2)',
                    zerolinewidth: 2
                },
                plot_bgcolor: 'rgba(0,0,0,0)',
                paper_bgcolor: 'rgba(0,0,0,0)',
                legend: {
                    x: 1,
                    xanchor: 'right',
                    y: 1,
                    bgcolor: 'rgba(0,0,0,0.7)',
                    bordercolor: 'rgba(255,255,255,0.15)',
                    borderwidth: 1,
                    font: { color: '#ffffff', size: 12 }
                },
                font: { family: 'Inter', color: 'rgba(255,255,255,0.7)' }
            }, {responsive: true, displayModeBar: false});
            
            // Hide live playback for comparison view
            const liveViz = document.querySelector('#spineViz').closest('.spine-viz-container');
            if (liveViz) {
                liveViz.style.display = 'none';
            }
        }

        function displayResults(data) {
            console.log('📊 displayResults() called');
            console.log('   Data:', data);
            
            document.getElementById('dashboard').style.display = 'block';
            console.log('   Dashboard shown');
            
            // Show live visualization for single view
            const liveVizContainer = document.querySelector('#spineViz').closest('.spine-viz-container');
            if (liveVizContainer) {
                liveVizContainer.style.display = 'block';
            }
            
            // Reset titles for single view
            document.getElementById('avgSpineTitle').textContent = 'Average Spine Position (Session)';
            document.getElementById('avgSpineSubtitle').textContent = 
                '📍 This shows your average spine curvature across the entire recording session.';
            
            // Update metrics
            console.log('   Updating metrics...');
            document.getElementById('totalRom').textContent = data.total_rom.toFixed(1);
            document.getElementById('upperRom').textContent = data.upper_rom.toFixed(1);
            document.getElementById('middleRom').textContent = data.middle_rom.toFixed(1);
            document.getElementById('lowerRom').textContent = data.lower_rom.toFixed(1);
            document.getElementById('duration').textContent = data.duration.toFixed(1);
            document.getElementById('durationMin').textContent = (data.duration / 60).toFixed(1) + ' minutes';
            document.getElementById('sensors').textContent = data.sensors;
            document.getElementById('score').textContent = data.movement_score.toFixed(2);
            document.getElementById('samples').textContent = data.samples.toLocaleString();
            document.getElementById('curvature').textContent = data.max_curvature.toFixed(2);
            
            // Activity level
            const score = data.movement_score;
            let activityLevel = score > 5 ? 'High activity' : (score > 2 ? 'Moderate' : 'Low activity');
            document.getElementById('activityLevel').textContent = activityLevel;

            // ROM Distribution Chart
            Plotly.newPlot('romChart', [{
                x: ['Upper', 'Middle', 'Lower'],
                y: [data.upper_rom, data.middle_rom, data.lower_rom],
                type: 'bar',
                marker: {
                    color: ['#3b82f6', '#8b5cf6', '#ec4899'],
                    line: { color: 'rgba(255,255,255,0.2)', width: 2 }
                },
                text: [data.upper_rom.toFixed(1) + '°', data.middle_rom.toFixed(1) + '°', data.lower_rom.toFixed(1) + '°'],
                textposition: 'outside',
                textfont: { size: 14, family: 'Inter', weight: 'bold', color: '#ffffff' }
            }], {
                margin: { t: 30, b: 50, l: 60, r: 20 },
                yaxis: { 
                    title: 'Range of Motion (°)', 
                    gridcolor: 'rgba(255,255,255,0.08)',
                    color: 'rgba(255,255,255,0.7)',
                    zerolinecolor: 'rgba(255,255,255,0.1)'
                },
                xaxis: { color: 'rgba(255,255,255,0.7)' },
                plot_bgcolor: 'rgba(0,0,0,0)',
                paper_bgcolor: 'rgba(0,0,0,0)',
                font: { family: 'Inter', color: 'rgba(255,255,255,0.7)' }
            }, {responsive: true, displayModeBar: false});

            // Time series chart (per sensor)
            const traces = [];
            const colors = ['#3b82f6', '#8b5cf6', '#ec4899', '#f59e0b', '#10b981', '#06b6d4', '#6366f1', '#ef4444'];
            
            for (let i = 0; i < data.sensor_order.length; i++) {
                const sid = data.sensor_order[i];
                const points = data.sensor_roms[sid] ? (data.spine_curves.map(c => ({
                    time: c.time,
                    angle: c.angles[i]
                })).filter(p => p.angle !== undefined)) : [];
                
                if (points.length > 0) {
                    // Sample down to max 500 points for performance
                    const step = Math.max(1, Math.floor(points.length / 500));
                    const sampled = points.filter((_, idx) => idx % step === 0);
                    
                    traces.push({
                        x: sampled.map(p => p.time),
                        y: sampled.map(p => p.angle),
                        type: 'scatter',
                        mode: 'lines',
                        name: 'Sensor ' + sid,
                        line: { color: colors[i % colors.length], width: 2.5 }
                    });
                }
            }

            Plotly.newPlot('timeChart', traces, {
                margin: { t: 20, b: 50, l: 60, r: 20 },
                xaxis: { 
                    title: 'Time (s)', 
                    gridcolor: 'rgba(255,255,255,0.08)',
                    color: 'rgba(255,255,255,0.7)',
                    zerolinecolor: 'rgba(255,255,255,0.1)'
                },
                yaxis: { 
                    title: 'Angle (°)', 
                    gridcolor: 'rgba(255,255,255,0.08)',
                    color: 'rgba(255,255,255,0.7)',
                    zerolinecolor: 'rgba(255,255,255,0.1)'
                },
                showlegend: true,
                legend: { 
                    x: 1, 
                    xanchor: 'right', 
                    y: 1,
                    bgcolor: 'rgba(0,0,0,0.5)',
                    bordercolor: 'rgba(255,255,255,0.1)',
                    borderwidth: 1,
                    font: { color: '#ffffff' }
                },
                plot_bgcolor: 'rgba(0,0,0,0)',
                paper_bgcolor: 'rgba(0,0,0,0)',
                font: { family: 'Inter', color: 'rgba(255,255,255,0.7)' }
            }, {responsive: true, displayModeBar: false});

            // Sensor ROM chart
            const sensorIds = data.sensor_order;
            const sensorRomValues = sensorIds.map(sid => data.sensor_roms[sid]);
            
            Plotly.newPlot('sensorChart', [{
                x: sensorIds.map(s => 'Sensor ' + s),
                y: sensorRomValues,
                type: 'bar',
                marker: {
                    color: sensorRomValues,
                    colorscale: [[0, '#3b82f6'], [0.5, '#8b5cf6'], [1, '#ec4899']],
                    showscale: true,
                    colorbar: { 
                        title: 'ROM (°)',
                        titlefont: { color: '#ffffff' },
                        tickfont: { color: '#ffffff' },
                        bgcolor: 'rgba(0,0,0,0.5)',
                        bordercolor: 'rgba(255,255,255,0.1)',
                        borderwidth: 1
                    },
                    line: { color: 'rgba(255,255,255,0.2)', width: 2 }
                },
                text: sensorRomValues.map(v => v.toFixed(1) + '°'),
                textposition: 'outside',
                textfont: { size: 13, family: 'Inter', weight: 'bold', color: '#ffffff' }
            }], {
                margin: { t: 30, b: 70, l: 60, r: 100 },
                yaxis: { 
                    title: 'ROM (degrees)',
                    gridcolor: 'rgba(255,255,255,0.08)',
                    color: 'rgba(255,255,255,0.7)',
                    zerolinecolor: 'rgba(255,255,255,0.1)'
                },
                xaxis: { 
                    tickangle: -45,
                    color: 'rgba(255,255,255,0.7)'
                },
                plot_bgcolor: 'rgba(0,0,0,0)',
                paper_bgcolor: 'rgba(0,0,0,0)',
                font: { family: 'Inter', color: 'rgba(255,255,255,0.7)' }
            }, {responsive: true, displayModeBar: false});

            // Draw average spine position
            console.log('   Drawing average spine...');
            drawAverageSpine(data.avg_spine);
            
            // Initialize spine animation
            console.log('   Initializing spine animation...');
            if (data.spine_curves.length > 0) {
                console.log('   Spine curves available:', data.spine_curves.length);
                document.getElementById('timeSlider').max = data.spine_curves.length - 1;
                drawSpineFrame(0);
            }
            
            console.log('✅ displayResults() complete!');
        }

        function drawAverageSpine(spinePoints) {
            if (!spinePoints || spinePoints.length === 0) return;
            
            const x = spinePoints.map(p => p[0]);
            const y = spinePoints.map(p => p[1]);
            
            // Create gradient colors for sensors (top to bottom)
            const n = spinePoints.length;
            const sensorColors = Array.from({length: n}, (_, i) => {
                const ratio = i / (n - 1);
                if (ratio < 0.33) return '#3b82f6';      // Top - blue
                else if (ratio < 0.67) return '#8b5cf6';  // Middle - purple
                else return '#ec4899';                     // Bottom - pink
            });
            
            Plotly.newPlot('avgSpineViz', [
                {
                    x: x,
                    y: y,
                    type: 'scatter',
                    mode: 'lines+markers',
                    line: { 
                        color: 'rgba(139, 92, 246, 0.6)', 
                        width: 8,
                        shape: 'spline',
                        smoothing: 1.3
                    },
                    marker: { 
                        size: 14, 
                        color: sensorColors,
                        line: { color: 'rgba(255,255,255,0.3)', width: 3 },
                        symbol: 'circle'
                    },
                    text: x.map((_, i) => `Sensor ${i+1}`),
                    hovertemplate: '%{text}<br>Position: (%{x:.1f}, %{y:.1f}) cm<extra></extra>',
                    name: 'Average Position'
                },
                {
                    x: [0, 0],
                    y: [Math.min(...y) - 10, Math.max(...y) + 10],
                    type: 'scatter',
                    mode: 'lines',
                    line: { color: 'rgba(255,255,255,0.15)', width: 2, dash: 'dash' },
                    showlegend: false,
                    hoverinfo: 'skip'
                },
                {
                    x: [Math.min(...x) - 10, Math.max(...x) + 10],
                    y: [0, 0],
                    type: 'scatter',
                    mode: 'lines',
                    line: { color: 'rgba(255,255,255,0.15)', width: 2, dash: 'dash' },
                    showlegend: false,
                    hoverinfo: 'skip'
                }
            ], {
                margin: { t: 20, b: 50, l: 60, r: 60 },
                xaxis: { 
                    title: 'Lateral (cm)', 
                    gridcolor: 'rgba(255,255,255,0.05)',
                    zeroline: true,
                    zerolinecolor: 'rgba(255,255,255,0.2)',
                    zerolinewidth: 2,
                    color: 'rgba(255,255,255,0.7)'
                },
                yaxis: { 
                    title: 'Vertical (cm)', 
                    gridcolor: 'rgba(255,255,255,0.05)',
                    scaleanchor: 'x',
                    scaleratio: 1,
                    color: 'rgba(255,255,255,0.7)'
                },
                plot_bgcolor: 'rgba(0,0,0,0)',
                paper_bgcolor: 'rgba(0,0,0,0)',
                showlegend: false,
                font: { family: 'Inter', color: 'rgba(255,255,255,0.7)' }
            }, {responsive: true, displayModeBar: false});
        }

        function drawSpineFrame(frameIdx) {
            if (!currentData || !currentData.spine_curves[frameIdx]) return;
            
            const curve = currentData.spine_curves[frameIdx];
            const points = curve.points;
            
            if (!points || points.length === 0) return;
            
            const x = points.map(p => p[0]);
            const y = points.map(p => p[1]);
            const n = points.length;
            
            // Color gradient from blue (top) to purple (middle) to pink (bottom)
            const colors = [];
            for (let i = 0; i < n; i++) {
                const ratio = i / (n - 1);
                if (ratio < 0.33) {
                    colors.push('#3b82f6');  // Blue - upper
                } else if (ratio < 0.67) {
                    colors.push('#8b5cf6');  // Purple - middle
                } else {
                    colors.push('#ec4899');  // Pink - lower
                }
            }
            
            Plotly.newPlot('spineViz', [
                {
                    x: x,
                    y: y,
                    type: 'scatter',
                    mode: 'lines',
                    line: { 
                        color: 'rgba(139, 92, 246, 0.7)', 
                        width: 10,
                        shape: 'spline',
                        smoothing: 1.3
                    },
                    name: 'Spine Curve',
                    hoverinfo: 'skip'
                },
                {
                    x: x,
                    y: y,
                    type: 'scatter',
                    mode: 'markers',
                    marker: { 
                        size: 18, 
                        color: colors,
                        line: { color: 'rgba(255,255,255,0.4)', width: 3 },
                        symbol: 'circle'
                    },
                    text: x.map((_, i) => `<b>Sensor ${i+1}</b><br>Angle: ${curve.angles[i].toFixed(1)}°<br>Position: (${x[i].toFixed(1)}, ${y[i].toFixed(1)}) cm`),
                    hovertemplate: '%{text}<extra></extra>',
                    name: 'Sensors'
                },
                {
                    x: [0, 0],
                    y: [Math.min(...y) - 10, Math.max(...y) + 10],
                    type: 'scatter',
                    mode: 'lines',
                    line: { color: 'rgba(255,255,255,0.15)', width: 2, dash: 'dash' },
                    showlegend: false,
                    hoverinfo: 'skip'
                },
                {
                    x: [Math.min(...x) - 10, Math.max(...x) + 10],
                    y: [0, 0],
                    type: 'scatter',
                    mode: 'lines',
                    line: { color: 'rgba(255,255,255,0.15)', width: 2, dash: 'dash' },
                    showlegend: false,
                    hoverinfo: 'skip'
                }
            ], {
                margin: { t: 20, b: 50, l: 60, r: 60 },
                xaxis: { 
                    title: 'Lateral (cm)', 
                    gridcolor: 'rgba(255,255,255,0.05)',
                    range: [Math.min(...x) - 10, Math.max(...x) + 10],
                    color: 'rgba(255,255,255,0.7)',
                    zerolinecolor: 'rgba(255,255,255,0.2)',
                    zerolinewidth: 2
                },
                yaxis: { 
                    title: 'Vertical (cm)', 
                    gridcolor: 'rgba(255,255,255,0.05)',
                    scaleanchor: 'x',
                    scaleratio: 1,
                    range: [Math.min(...y) - 10, Math.max(...y) + 10],
                    color: 'rgba(255,255,255,0.7)',
                    zerolinecolor: 'rgba(255,255,255,0.2)',
                    zerolinewidth: 2
                },
                plot_bgcolor: 'rgba(0,0,0,0)',
                paper_bgcolor: 'rgba(0,0,0,0)',
                showlegend: false,
                font: { family: 'Inter', color: 'rgba(255,255,255,0.7)' }
            }, {responsive: true, displayModeBar: false});
            
            // Update time display
            const totalTime = currentData.spine_curves[currentData.spine_curves.length - 1].time;
            document.getElementById('timeDisplay').textContent = 
                `${curve.time.toFixed(1)}s / ${totalTime.toFixed(1)}s`;
        }

        function togglePlayback() {
            isPlaying = !isPlaying;
            const btn = document.getElementById('playBtn');
            
            if (isPlaying) {
                btn.textContent = '⏸ Pause';
                animate();
            } else {
                btn.textContent = '▶️ Play';
                if (animationTimer) {
                    clearTimeout(animationTimer);
                    animationTimer = null;
                }
            }
        }

        function animate() {
            if (!isPlaying || !currentData) return;
            
            currentFrame++;
            if (currentFrame >= currentData.spine_curves.length) {
                currentFrame = 0;
            }
            
            drawSpineFrame(currentFrame);
            document.getElementById('timeSlider').value = currentFrame;
            
            const delay = 50 / playbackSpeed;
            animationTimer = setTimeout(animate, delay);
        }

        function seekToFrame(value) {
            currentFrame = parseInt(value);
            drawSpineFrame(currentFrame);
            
            if (isPlaying) {
                isPlaying = false;
                document.getElementById('playBtn').textContent = '▶️ Play';
                if (animationTimer) clearTimeout(animationTimer);
            }
        }

        function changeSpeed() {
            playbackSpeed = parseFloat(document.getElementById('speedControl').value);
        }
    </script>
</body>
</html>"""


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_TEMPLATE.encode())
        else:
            super().do_GET()
    
    def do_POST(self):
        if self.path == '/upload':
            print("\n📥 Received upload request")
            
            try:
                content_length = int(self.headers['Content-Length'])
                print(f"   Content length: {content_length} bytes")
                
                post_data = self.rfile.read(content_length)
                print(f"   Read {len(post_data)} bytes")
                
                # Parse multipart form data
                content_type = self.headers.get('Content-Type', '')
                if 'boundary=' not in content_type:
                    print("   ❌ No boundary in Content-Type")
                    self.send_error(400, "Invalid Content-Type")
                    return
                
                boundary = content_type.split('boundary=')[1].encode()
                print(f"   Boundary: {boundary[:20]}...")
                
                parts = post_data.split(b'--' + boundary)
                print(f"   Found {len(parts)} parts")
                
                for idx, part in enumerate(parts):
                    if b'filename=' in part:
                        print(f"   Processing part {idx} with filename")
                        
                        # Extract CSV content
                        csv_content = part.split(b'\r\n\r\n', 1)[1]
                        csv_content = csv_content.rsplit(b'\r\n', 1)[0]
                        
                        print(f"   CSV content size: {len(csv_content)} bytes")
                        
                        # Save temp file
                        temp_path = '/tmp/spine_upload.csv'
                        with open(temp_path, 'wb') as f:
                            f.write(csv_content)
                        
                        print(f"   Saved to {temp_path}")
                        print("   🔬 Starting analysis...")
                        
                        # Analyze
                        result = load_and_analyze_csv(temp_path)
                        
                        if result:
                            print("   ✅ Analysis successful!")
                            print(f"   Returning {len(json.dumps(result))} bytes of JSON")
                            
                            self.send_response(200)
                            self.send_header('Content-type', 'application/json')
                            self.send_header('Access-Control-Allow-Origin', '*')
                            self.end_headers()
                            self.wfile.write(json.dumps(result).encode())
                            return
                        else:
                            print("   ❌ Analysis returned None")
                
                print("   ❌ No valid file part found")
                self.send_error(400, "No valid file uploaded")
                
            except Exception as e:
                print(f"   ❌ Exception in do_POST: {e}")
                import traceback
                traceback.print_exc()
                self.send_error(500, f"Server error: {str(e)}")
    
    def log_message(self, format, *args):
        # Only log errors
        if '40' in format or '50' in format:
            super().log_message(format, *args)


def main():
    print("╔═══════════════════════════════════════════════════════════╗")
    print("║       🏥 Spine Movement Analysis Dashboard 🏥            ║")
    print("╚═══════════════════════════════════════════════════════════╝")
    print()
    print("🚀 Starting web server...")
    print(f"📡 Server running at: http://localhost:{PORT}")
    print()
    print("✅ Opening dashboard in your browser...")
    print("   (If it doesn't open, manually visit the URL above)")
    print()
    print("📝 Instructions:")
    print("   1. Click the upload area in your browser")
    print("   2. Select your CSV file (e.g., LOG00001.CSV)")
    print("   3. Watch the beautiful spine visualization!")
    print()
    print("✨ Features:")
    print("   • Real-time spine curve animation")
    print("   • Average spine position analysis")
    print("   • Interactive charts with zoom/pan")
    print("   • Complete ROM metrics")
    print()
    print("🛑 Press Ctrl+C to stop the server")
    print("═" * 63)
    print()
    
    # Start server
    handler = DashboardHandler
    with socketserver.TCPServer(("", PORT), handler) as httpd:
        server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
        server_thread.start()
        
        # Open browser
        time.sleep(1)
        webbrowser.open(f'http://localhost:{PORT}')
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\n✅ Dashboard stopped. Thanks for using Spine Analyzer!")


if __name__ == '__main__':
    main()