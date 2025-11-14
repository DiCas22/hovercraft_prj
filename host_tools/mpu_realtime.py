#!/usr/bin/env python3
# host_tools/mpu_realtime.py
# Realtime IMU (MQTT) + posição relativa no plano: x(t), y(t)
# Dep.: paho-mqtt, pyqtgraph, numpy

import argparse, json, threading, time, sys, math
from collections import deque

import numpy as np
import paho.mqtt.client as mqtt
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# --------- Args ---------
parser = argparse.ArgumentParser(description="Realtime IMU viewer (MQTT) c/ posição x(t), y(t)")
parser.add_argument("--broker", default="broker.hivemq.com")
parser.add_argument("--port", type=int, default=1883)
parser.add_argument("--topic", default="hovercraft/imu", help="tópico publicado pelo ESP")
parser.add_argument("--max-points", type=int, default=6000)
parser.add_argument("--update-ms", type=int, default=20)
args = parser.parse_args()

# --------- Buffers + lock ---------
keys = ["t","ax","ay","az","gx","gy","gz","sx","sy","vx","vy","roll","pitch","yaw"]
buf  = {k: deque(maxlen=args.max_points) for k in keys}
lock = threading.Lock()
msg_count = 0

# --------- Constantes ---------
G = 9.80665
ALPHA = 0.96               # Slightly lower for more acc influence (per tutorials)
CALIB_SAMPLES = 500        # Long calibration: 500 samples at startup (~5s at 100Hz)
BIAS_ALPHA = 0.001         # Soft running bias update (slow)
VEL_THRESHOLD = 0.1        # m/s below which update bias softly (no ZUPT)
M_SENSOR2BODY = np.eye(3)



# Mapeamento opcional sensor->corpo (ajuste se seus eixos estiverem diferentes)
M_SENSOR2BODY = np.eye(3)

# --------- Estado ---------
state = dict(
    t_last=None,
    # ângulos gyro-integrados (rad)
    roll_g=0.0, pitch_g=0.0, yaw_g=0.0,
    # fundidos (rad)
    roll=0.0, pitch=0.0, yaw=0.0,
    # vel/pos no mundo (XY)
    v=np.zeros(2),
    s=np.zeros(2),
    # calibração
    calib_start=time.time(),
    calib_done=False,
    sum_g_raw=np.zeros(3),      # soma gyro raw para bias (before integration)
    sum_a_raw=np.zeros(3),      # soma acc raw para bias
    n_calib=0,
    b_gyro=np.zeros(3),         # bias gyro (deg/s)
    b_acc=np.zeros(3),          # bias acc (g)

    bias_gyro_run=np.zeros(3),  # Running gyro bias
    bias_acc_run=np.zeros(3),   # Running acc bias
    calib_samples=0,

    # HPF state
    a_hpf_prev=np.zeros(3),
    # ZUPT state
    quiet_timer=0.0,
    zupt_history=deque(maxlen=ZUPT_HISTORY),  # list of (gyro_norm, a_xy_norm)
    # dt smoothing
    dt_history=deque(maxlen=10)
)

# --------- Helpers ---------
def euler_world_from_body(yaw, pitch, roll):
    cy, sy = math.cos(yaw),  math.sin(yaw)
    cp, sp = math.cos(pitch),math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    # R_world_body = Rz(yaw) * Ry(pitch) * Rx(roll)  (ZYX)
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return Rz @ Ry @ Rx

def estimate_rp_from_acc(ax, ay, az):
    # convenção padrão: roll ~ atan2(ay, az), pitch ~ atan2(-ax, sqrt(ay^2+az^2))
    roll  = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    return roll, pitch

def apply_hpf(a_current, a_prev, alpha=HPF_ALPHA):
    # Simple 1st-order high-pass filter: removes low-freq drift
    return alpha * (a_prev + (a_current - a_prev))

def ingest_point(pt):
    global msg_count, state
    with lock:
        # tempo
        t_ms = pt.get("t_ms")
        t = (t_ms / 1000.0) if isinstance(t_ms, (int, float)) else time.time()
        buf["t"].append(t)
        dt = 0.0 if state["t_last"] is None else (t - state["t_last"])
        
        # Smooth dt with median of history to handle outliers
        state["dt_history"].append(dt)
        if len(state["dt_history"]) == 10:
            dt = np.median(list(state["dt_history"]))
        if not (0.001 < dt < 0.2):  # clamp extremes
            dt = 0.01
        state["t_last"] = t

        # leitura IMU bruta
        mpu = pt.get("mpu_raw", {})
        ax_s = float(mpu.get("ax", 0.0))
        ay_s = float(mpu.get("ay", 0.0))
        az_s = float(mpu.get("az", 0.0))
        gx_d = float(mpu.get("gx", 0.0))
        gy_d = float(mpu.get("gy", 0.0))
        gz_d = float(mpu.get("gz", 0.0))

        # sensor->corpo
        a_s = np.array([ax_s, ay_s, az_s], dtype=float)
        g_d = np.array([gx_d, gy_d, gz_d], dtype=float)
        a_b = M_SENSOR2BODY @ a_s                 # g (gravidades)
        g_b = M_SENSOR2BODY @ g_d                 # deg/s

        # Calibração: accumulate raw biases first (before correction)
        if not state["calib_done"]:
            if time.time() - state["calib_start"] < CALIB_SEC:
                state["sum_g_raw"] += g_b
                state["sum_a_raw"] += a_b
                state["n_calib"] += 1
            else:
                if state["n_calib"] > 10:  # require min samples
                    state["b_gyro"] = state["sum_g_raw"] / state["n_calib"]
                    state["b_acc"] = state["sum_a_raw"] / state["n_calib"]
                state["calib_done"] = True

        # Correct biases early: gyro before integration, acc before gravity
        g_corr = np.deg2rad(g_b - state["b_gyro"])
        a_corr = a_b - state["b_acc"]

        # Integrate gyro for angles (now with bias corrected)
        state["roll_g"]  += g_corr[0]*dt
        state["pitch_g"] += g_corr[1]*dt
        state["yaw_g"]   += g_corr[2]*dt

        # roll/pitch from corrected ACC
        roll_acc, pitch_acc = estimate_rp_from_acc(a_corr[0], a_corr[1], a_corr[2])

        # Fusão roll/pitch; yaw só gyro
        state["roll"]  = ALPHA*state["roll_g"]  + (1.0-ALPHA)*roll_acc
        state["pitch"] = ALPHA*state["pitch_g"] + (1.0-ALPHA)*pitch_acc
        state["yaw"]   = state["yaw_g"]

        # Dynamic bias update (soft, post-calib)
        if state["calib_done"]:
            state["b_gyro"] = 0.999 * state["b_gyro"] + 0.001 * g_b
            state["b_acc"] = 0.999 * state["b_acc"] + 0.001 * a_b

        # Gravity in body frame
        R_wb = euler_world_from_body(state["yaw"], state["pitch"], state["roll"])
        g_world = np.array([0.0, 0.0, G])
        g_body  = R_wb.T @ g_world

        # Linear acc in body (use corrected acc)
        a_meas_body = a_corr * G
        a_lin_body  = a_meas_body - g_body      # m/s^2

        # Apply HPF to linear acc (remove drift)
        a_lin_body = apply_hpf(a_lin_body, state["a_hpf_prev"])
        state["a_hpf_prev"] = a_lin_body

        # Accel to world frame
        a_world = R_wb @ a_lin_body
        a_xy = a_world[:2]

        # ZUPT with history check
        gyro_norm = np.linalg.norm(g_corr)
        a_xy_norm = np.linalg.norm(a_xy)
        state["zupt_history"].append((gyro_norm, a_xy_norm))
        
        is_quiet = gyro_norm < QUIET_G and a_xy_norm < QUIET_A
        quiet_count = sum(1 for gn, an in state["zupt_history"] if gn < QUIET_G and an < QUIET_A)
        sustained_quiet = quiet_count >= 3  # 3/5 recent samples quiet
        
        v_prev = state["v"].copy()
                # ZUPT with relaxed check
        gyro_norm = np.linalg.norm(g_corr)
        a_xy_norm = np.linalg.norm(a_xy)
        state["zupt_history"].append((gyro_norm, a_xy_norm))
        
        is_quiet = gyro_norm < QUIET_G and a_xy_norm < QUIET_A
        quiet_count = sum(1 for gn, an in state["zupt_history"] if gn < QUIET_G and an < QUIET_A)
        sustained_quiet = quiet_count >= 4  # Stricter: 4/5 samples quiet
        
        v_prev = state["v"].copy()
        if sustained_quiet:
            # Slower decay; only if very low accel (true stop)
            if a_xy_norm < 0.05:
                state["v"] *= 0.995  # Slower damping
            state["quiet_timer"] += dt
            # Yaw reset if long quiet
            if state["quiet_timer"] > YAW_RESET_SEC:
                state["yaw"] = 0.0
                state["yaw_g"] = 0.0
                state["quiet_timer"] = 0.0
            # Pause position during true ZUPT
            a_xy_zupt = np.zeros(2) if a_xy_norm < 0.05 else a_xy
            state["v"] = v_prev + a_xy_zupt * dt
        else:
            state["v"] = v_prev + a_xy * dt
            state["quiet_timer"] = 0.0

        # Clip yaw for planar assumption
        state["yaw"] = np.clip(state["yaw"], -math.pi/2, math.pi/2)

        # Position integration
        state["s"] = state["s"] + 0.5*(v_prev + state["v"])*dt


        # Position integration (trapezoidal)
        state["s"] = state["s"] + 0.5*(v_prev + state["v"])*dt

        # Buffers for plot
        buf["ax"].append(a_b[0]); buf["ay"].append(a_b[1]); buf["az"].append(a_b[2])
        buf["gx"].append(g_b[0]); buf["gy"].append(g_b[1]); buf["gz"].append(g_b[2])
        buf["vx"].append(state["v"][0]); buf["vy"].append(state["v"][1])
        buf["sx"].append(state["s"][0]); buf["sy"].append(state["s"][1])
        buf["roll"].append(state["roll"]); buf["pitch"].append(state["pitch"]); buf["yaw"].append(state["yaw"])

        msg_count += 1

# --------- MQTT ---------
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"[MQTT] conectado em {args.broker}:{args.port} — tópico: {args.topic}")
        client.subscribe(args.topic, qos=0)
    else:
        print(f"[MQTT] erro rc={rc}")

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        series = payload.get("series", [payload])
        for pt in series:
            ingest_point(pt)
    except Exception as e:
        print(f"[MQTT] decode error: {e}")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.connect_async(args.broker, args.port, keepalive=60)
client.loop_start()

# --------- UI ---------
pg.setConfigOptions(antialias=False, useOpenGL=True)
app  = QtWidgets.QApplication([])
win  = pg.GraphicsLayoutWidget(show=True, title="Hovercraft — x(t), y(t) relativos a t(0)")
win.resize(1200, 900)

# Linha 1: acel
p_acc = win.addPlot(title="Accelerometer (g)")
p_acc.addLegend(); p_acc.showGrid(x=True, y=True)
c_ax = p_acc.plot(name="ax"); c_ay = p_acc.plot(name="ay"); c_az = p_acc.plot(name="az")

# Linha 2: gyro
win.nextRow()
p_gyr = win.addPlot(title="Gyroscope (deg/s)")
p_gyr.addLegend(); p_gyr.showGrid(x=True, y=True)
c_gx = p_gyr.plot(name="gx"); c_gy = p_gyr.plot(name="gy"); c_gz = p_gyr.plot(name="gz")

# Linha 3: X(t)
win.nextRow()
p_sx = win.addPlot(title="Posição X vs tempo (m)")
p_sx.showGrid(x=True, y=True); c_sx = p_sx.plot()
p_sx.setLabel("bottom", "tempo (s)")

# Linha 4: Y(t)
win.nextRow()
p_sy = win.addPlot(title="Posição Y vs tempo (m)")
p_sy.showGrid(x=True, y=True); c_sy = p_sy.plot()
p_sy.setLabel("bottom", "tempo (s)")

fps_label = pg.LabelItem(justify='right'); win.addItem(fps_label)

last_wall = time.time(); frames = 0

def update():
    global last_wall, frames
    with lock:
        N = len(buf["t"])
        if N == 0: return
        t0 = buf["t"][0]
        tt = np.array(buf["t"], dtype=float) - t0

        if buf["ax"]: c_ax.setData(tt, np.array(buf["ax"]))
        if buf["ay"]: c_ay.setData(tt, np.array(buf["ay"]))
        if buf["az"]: c_az.setData(tt, np.array(buf["az"]))
        if buf["gx"]: c_gx.setData(tt, np.array(buf["gx"]))
        if buf["gy"]: c_gy.setData(tt, np.array(buf["gy"]))
        if buf["gz"]: c_gz.setData(tt, np.array(buf["gz"]))
        if buf["sx"]: c_sx.setData(tt, np.array(buf["sx"]))
        if buf["sy"]: c_sy.setData(tt, np.array(buf["sy"]))

    frames += 1
    now = time.time()
    if now - last_wall >= 1.0:
        fps_label.setText(f"msgs: {msg_count}  |  fps: {frames/(now-last_wall):.1f}")
        last_wall = now; frames = 0

timer = QtCore.QTimer(); timer.timeout.connect(update); timer.start(args.update_ms)

# Atalhos úteis
def keyPressEvent(evt):
    key = evt.key()
    # R = reset origem (posição, velocidade, angles)
    if key == QtCore.Qt.Key_R:
        with lock:
            state["s"][:] = 0.0
            state["v"][:] = 0.0
            state["roll_g"] = state["pitch_g"] = state["yaw_g"] = 0.0
            state["roll"] = state["pitch"] = state["yaw"] = 0.0
            # Reset calib for fresh start
            state["calib_done"] = False
            state["n_calib"] = 0
            state["sum_g_raw"][:] = state["sum_a_raw"][:] = 0.0
            state["b_gyro"][:] = state["b_acc"][:] = 0.0
            state["a_hpf_prev"][:] = 0.0
            state["quiet_timer"] = 0.0
            state["zupt_history"].clear()
            state["dt_history"].clear()

win.keyPressEvent = keyPressEvent

if __name__ == "__main__":
    try:
        sys.exit(app.exec_())
    finally:
        client.loop_stop(); client.disconnect()
