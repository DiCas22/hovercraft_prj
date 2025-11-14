#!/usr/bin/env python3
# mpu_simple.py: Clean, working MPU6050 position via MQTT - No complex dependencies

import argparse
import json
import time
import sys
import math
from collections import deque
import numpy as np
from collections import deque

import threading
import os

# GUI imports (used only in main)
import paho.mqtt.client as mqtt
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# --------- Args ---------
parser = argparse.ArgumentParser(description="Simple MPU6050 position tracking")
parser.add_argument("--broker", default="broker.hivemq.com")
parser.add_argument("--port", type=int, default=1883)
parser.add_argument("--topic", default="hovercraft/imu")
parser.add_argument("--max-points", type=int, default=1000)
parser.add_argument("--update-ms", type=int, default=100)
parser.add_argument("--debug", action="store_true")
args = parser.parse_args()

# --------- Globals ---------
buf = {}
plots_ready = False

# --------- Buffers ---------
def init_buffers():
    keys = ["t", "ax", "ay", "az", "gx", "gy", "gz", "vx", "vy", "vz", "sx", "sy", "sz", "roll", "pitch", "yaw"]
    return {k: deque(maxlen=args.max_points) for k in keys}

buf = init_buffers()
lock = threading.Lock()
msg_count = 0
calib_samples_list = []

# --------- State ---------
state = {
    "t_last": None,
    "roll_g": 0.0, "pitch_g": 0.0, "yaw_g": 0.0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "v": np.array([0.0, 0.0, 0.0]),
    "s": np.array([0.0, 0.0, 0.0]),
    "offsets_acc": np.array([0.0, 0.0, 0.0]),
    "offsets_gyro": np.array([0.0, 0.0, 0.0]),
    "calib_samples": 0,
    "warmed_up": False,
    "dt_history": deque(maxlen=10)
}

# Constants
G = 9.80665
ALPHA = 0.98
CALIB_SAMPLES = 500
M_SENSOR2BODY = np.eye(3)
CALIB_FILE = 'mpu_calib.json'
calib_done = False

# --------- Calibration ---------
def calibrate():
    global state
    if len(calib_samples_list) < CALIB_SAMPLES // 10:
        print("Warning: Insufficient calibration data")
        return False
    
    acc_samples = np.array([s[0] for s in calib_samples_list])
    gyro_samples = np.array([s[1] for s in calib_samples_list])
    
    # Simple bias calibration
    acc_bias = np.mean(acc_samples, axis=0)
    gyro_bias = np.mean(gyro_samples, axis=0)
    
    state["offsets_acc"] = acc_bias
    state["offsets_gyro"] = gyro_bias
    
    # Save to file
    calib_data = {
        'acc_bias': acc_bias.tolist(),
        'gyro_bias': gyro_bias.tolist(),
        'timestamp': time.time()
    }
    with open(CALIB_FILE, 'w') as f:
        json.dump(calib_data, f)
    
    print(f"✓ Calibration done: acc_bias={acc_bias:.3f}, gyro_bias={gyro_bias:.1f}")
    return True

def load_calib():
    if not os.path.exists(CALIB_FILE):
        return False
    try:
        with open(CALIB_FILE, 'r') as f:
            calib = json.load(f)
        state["offsets_acc"] = np.array(calib['acc_bias'])
        state["offsets_gyro"] = np.array(calib['gyro_bias'])
        print(f"✓ Loaded calibration: acc_bias={state['offsets_acc']:.3f}")
        return True
    except:
        return False

# --------- Data Processing ---------
def ingest_point(pt):
    global msg_count, state, calib_samples_list, calib_done, plots_ready
    
    with lock:
        # Time
        t_ms = pt.get("t_ms", 0)
        t = t_ms / 1000.0 if t_ms else time.time()
        buf["t"].append(t)
        
        # dt
        dt = 0.0 if state["t_last"] is None else (t - state["t_last"])
        state["dt_history"].append(dt)
        if len(state["dt_history"]) == 10:
            dt = np.median(list(state["dt_history"]))
        dt = max(0.001, min(dt, 0.2))
        state["t_last"] = t

        # Parse raw data
        mpu = pt.get("mpu_raw", {})
        if not mpu:
            print("No data")
            # Fill all buffers with zeros
            for k in buf:
                if k != "t":
                    buf[k].append(0.0)
            return

        ax = float(mpu.get("ax", 0.0))
        ay = float(mpu.get("ay", 0.0))
        az = float(mpu.get("az", 0.0))
        gx = float(mpu.get("gx", 0.0))
        gy = float(mpu.get("gy", 0.0))
        gz = float(mpu.get("gz", 0.0))

        # Transform
        a_raw = np.array([ax, ay, az])
        g_raw = np.array([gx, gy, gz])
        a_b = M_SENSOR2BODY @ a_raw  # g
        g_b = M_SENSOR2BODY @ g_raw  # deg/s

        # ===== CALIBRATION PHASE =====
        if state["calib_samples"] < CALIB_SAMPLES and not calib_done:
            calib_samples_list.append((a_b, g_b))
            state["calib_samples"] += 1
            
            # Terminal progress
            progress = state["calib_samples"]
            bar = "█" * (progress // 10) + "░" * (50 - progress // 10)
            print(f"\rCalibrating [{progress:3d}/500] {bar} - Hold still!", end="")
            
            # Fill ALL buffers (sync length)
            buf["ax"].append(a_b[0]); buf["ay"].append(a_b[1]); buf["az"].append(a_b[2])
            buf["gx"].append(g_b[0]); buf["gy"].append(g_b[1]); buf["gz"].append(g_b[2])
            for k in ["vx", "vy", "vz", "sx", "sy", "sz", "roll", "pitch", "yaw"]:
                buf[k].append(0.0)
            
            if state["calib_samples"] == CALIB_SAMPLES:
                print(f"\n\n✓ CALIBRATION COMPLETE! Computing...")
                calib_done = calibrate()
                if calib_done:
                    print("🚀 POSITION TRACKING READY!")
        else:
            # NORMAL OPERATION
            if not calib_done:
                print("⚠️ Calibration failed - check sensor")
                # Fill buffers to prevent plotting errors
                buf["ax"].append(a_b[0]); buf["ay"].append(a_b[1]); buf["az"].append(a_b[0])
                buf["gx"].append(g_b[0]); buf["gy"].append(g_b[1]); buf["gz"].append(g_b[2])
                for k in ["vx", "vy", "vz", "sx", "sy", "sz", "roll", "pitch", "yaw"]:
                    buf[k].append(0.0)
                return

            # Bias correction
            a_corrected = a_b - state["offsets_acc"]
            g_corrected = g_b - state["offsets_gyro"]
            g_rad = np.deg2rad(g_corrected)

            # Gyro integration
            state["roll_g"] += g_rad[0] * dt
            state["pitch_g"] += g_rad[1] * dt
            state["yaw_g"] += g_rad[2] * dt

            # Accelerometer angles
            roll_acc = math.atan2(a_corrected[1], a_corrected[2])
            pitch_acc = math.atan2(-a_corrected[0], math.sqrt(a_corrected[1]**2 + a_corrected[2]**2))

            # Complementary filter
            state["roll"] = ALPHA * state["roll_g"] + (1 - ALPHA) * roll_acc
            state["pitch"] = ALPHA * state["pitch_g"] + (1 - ALPHA) * pitch_acc
            state["yaw"] = state["yaw_g"]

            # Gravity compensation
            gravity_x = math.sin(state["pitch"]) * G
            gravity_y = -math.sin(state["roll"]) * math.cos(state["pitch"]) * G
            gravity_z = math.cos(state["pitch"]) * math.cos(state["roll"]) * G

            # Linear acceleration (remove gravity)
            lin_a = np.array([
                a_corrected[0] * G - gravity_x,
                a_corrected[1] * G - gravity_y,
                a_corrected[2] * G - gravity_z
            ])

            # Double integration
            v_prev = state["v"].copy()
            state["v"] += lin_a * dt
            state["s"] += (v_prev + state["v"]) * 0.5 * dt

            # Fill ALL buffers
            buf["ax"].append(a_b[0]); buf["ay"].append(a_b[1]); buf["az"].append(a_b[2])
            buf["gx"].append(g_b[0]); buf["gy"].append(g_b[1]); buf["gz"].append(g_b[2])
            buf["vx"].append(state["v"][0]); buf["vy"].append(state["v"][1]); buf["vz"].append(state["v"][2])
            buf["sx"].append(state["s"][0]); buf["sy"].append(state["s"][1]); buf["sz"].append(state["s"][2])
            buf["roll"].append(math.degrees(state["roll"]))
            buf["pitch"].append(math.degrees(state["pitch"]))
            buf["yaw"].append(math.degrees(state["yaw"]))

            msg_count += 1

# --------- MQTT ---------
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"✅ Connected to {args.broker}:{args.port} | Topic: {args.topic}")
        client.subscribe(args.topic, qos=0)
        print("🎯 Hold sensor still for calibration (500 samples, ~50s at 10Hz)")
    else:
        print(f"❌ Connection failed: rc={rc}")

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        series = payload.get("series", [payload]) if isinstance(payload, dict) else [payload]
        for pt in series:
            ingest_point(pt)
    except Exception as e:
        print(f"⚠️ MQTT message error: {e}")

client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.connect_async(args.broker, args.port, keepalive=60)
client.loop_start()

# --------- MAIN - UI CREATION AND EVENT LOOP ---------
if __name__ == "__main__":
    # Load existing calibration
    if not load_calib():
        print("No existing calibration found. Will calibrate from scratch.")

    app = QtWidgets.QApplication([])

    # PyQtGraph setup
    pg.setConfigOptions(antialias=False)
    app = QtWidgets.QApplication(sys.argv)
    
    # Create window
    win = pg.GraphicsLayoutWidget(show=True, title="MPU6050 Live Position Tracking")
    win.resize(1200, 800)

    # Create plots
    p1 = win.addPlot(title="Raw Accelerometer (g)")
    p1.addLegend()
    p1.showGrid(x=True, y=True)
    c_ax = p1.plot(pen=(255, 0, 0))  # Red for x
    c_ay = p1

    win.show()       # Show the main window
    sys.exit(app.exec())  # Enter Qt event loop, keep app running
