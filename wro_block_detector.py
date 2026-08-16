#!/usr/bin/env python3
# ASCII-only file so a Pi paste does not turn dashes into garbage.
"""
Block detector - detect.py (canonical race Python in this repo).

Official name: Block detector.
Same source as wro_block_detector.py. On the Pi, paste into round2.py and
start with systemd unit round2 (deploy/round2.service).

Runs on the Raspberry Pi: Lenovo webcam + best_ncnn.onnx, votes 5/7 frames,
freezes A/B/C at STOP_HEIGHT_PX, sends STOP then WAYPOINT (also REVERSE/CLEAR)
at 115200 to ESP32_Robot.ino. Logs to wro_detect.log.

Does not steer or read LiDAR. Named inventory: docs/CODE_CATALOG.md
"""

import math
import os
import sys
import time
import threading
import queue
import subprocess
import numpy as np
import cv2
import onnxruntime as ort
from collections import deque

# ---------------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------------
ONNX_MODEL_PATH = "best_ncnn.onnx"
MODEL_INPUT_SIZE = 224          # keep at 224 - this is what costs inference time
CLASS_NAMES = {0: "green", 1: "red"}
CONF_THRESHOLD = 0.55
USE_CUDA_IF_AVAILABLE = False   # Pi is CPU; True only if you run this on a CUDA box

# Capture is 640x480 MJPEG, then resized to a square for the detector/display.
CAPTURE_W = 640
CAPTURE_H = 480
FRAME_SIZE = 240                # square working frame (not the ONNX input size)

CAMERA_ID = 0                    # Lenovo: 0 = picture, 1 = metadata (skip 1 if 0 works)
CAMERA_INDEXES = (0, 2)          # do not hammer /dev/video1 (UVC metadata)
CAMERA_EXPOSURE = 200
CAMERA_WB_TEMP = 4500
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15                  # 30 fps on Pi USB3 often overruns xHCI (dmesg buffer overrun)
SERIAL_PORTS = ("/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyAMA0")
SERIAL_BAUD = 115200
LOG_NAME = "wro_detect.log"     # Pi prints + ESP serial, next to this script

# How many recent frames must see the same colour before we trust it.
VOTE_HISTORY = 7
MIN_VOTES = 5
CLEAR_HISTORY = 10              # consecutive CLEAR frames before we drop a waypoint
LOCK_HOLD_FRAMES = 2            # STOP-height frames in a row before freeze (skip a spike)

# Too close - abort / reverse.
REVERSE_HEIGHT_PX = 80
WAYPOINT_RESEND_S = 0.4         # re-send WAYPOINT while locked (USB often drops a one-shot)
WAYPOINT_RESEND_WINDOW_S = 1.2  # then stop; ESP ignores extras after GOTO-C until CLEAR

# ---------------------------------------------------------------------------
# Waypoint geometry - measure AB_DISTANCE_CM on the table at AB_CAL_HEIGHT_PX
# ---------------------------------------------------------------------------
# When box height hits this, treat current robot pose as B and the block as A.
# Start at 45 px (closer, tighter arc). If the turn to C is too sharp after a
# run, drop this to 30 so the bot stops farther away and the arc is gentler.
STOP_HEIGHT_PX = 30  # freeze A/B/C and send WAYPOINT at this box height

# AC: nudge of robot CENTER toward the pass side (red +X, green -X).
# Field: 10 cm sideways. Depth of C follows A (see AB_CAL_HEIGHT_PX).
AC_OFFSET_CM = 12.0

# AB: tape AB_DISTANCE_CM at AB_CAL_HEIGHT_PX (original 40 cm at 45 px).
# Lock is STOP_HEIGHT_PX=30, which is farther than 45 px. Using STOP in the
# scale made y_A=40 cm and C sat well in front of the block. Depth is
# AB * (cal_height / box_height) ~= 60 cm at a 30 px freeze.
AB_DISTANCE_CM = 40.0
AB_CAL_HEIGHT_PX = 45  # pixel height when AB_DISTANCE_CM was measured
# If you retape AB at 30 px, set AB_CAL_HEIGHT_PX = 30 and AB_DISTANCE_CM to that tape.

# Real pillar height in cm (WRO traffic-sign / pillar). Used only for lateral (X)
# Depth Y uses AB_DISTANCE_CM scaled by AB_CAL_HEIGHT_PX/height.
REAL_BLOCK_HEIGHT_CM = 10.0

# ---------------------------------------------------------------------------
# ONNX session setup
# ---------------------------------------------------------------------------
def load_onnx_session(model_path: str) -> tuple:
    providers = ["CPUExecutionProvider"]
    if USE_CUDA_IF_AVAILABLE and "CUDAExecutionProvider" in ort.get_available_providers():
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]

    session = ort.InferenceSession(model_path, providers=providers)
    input_name = session.get_inputs()[0].name
    output_names = [o.name for o in session.get_outputs()]
    print(f"Loaded ONNX model '{model_path}' | providers={session.get_providers()} "
          f"| input={input_name} | outputs={output_names}")
    return session, input_name, output_names

# ---------------------------------------------------------------------------
# Preprocessing - letterbox to a square, track scale/offset to map boxes back
# ---------------------------------------------------------------------------
def preprocess(frame: np.ndarray, size: int) -> tuple:
    h, w = frame.shape[:2]
    scale = size / max(h, w)
    nh, nw = int(h * scale), int(w * scale)
    resized = cv2.resize(frame, (nw, nh))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    top = (size - nh) // 2
    left = (size - nw) // 2
    canvas[top:top + nh, left:left + nw] = resized

    tensor = canvas.astype(np.float32) / 255.0
    tensor = tensor.transpose(2, 0, 1)          # HWC -> CHW
    tensor = np.expand_dims(tensor, axis=0)     # -> NCHW
    return np.ascontiguousarray(tensor), scale, left, top

# ---------------------------------------------------------------------------
# Postprocessing - decode Ultralytics-style output + NMS, map back to frame
# ---------------------------------------------------------------------------
def decode_onnx_output(raw_output: np.ndarray, scale: float, left: int, top: int,
                        conf_thresh: float) -> dict:
    """raw_output: (1, 300, 6) -> [x1,y1,x2,y2,conf,cls_id]."""
    preds = raw_output[0]   # (300, 6)

    best_per_class = {}
    for pred in preds:
        x1, y1, x2, y2, conf, cls_id = pred
        if conf < conf_thresh:
            continue
        cls_id = int(cls_id)
        if cls_id not in best_per_class or conf > best_per_class[cls_id][0]:
            best_per_class[cls_id] = (float(conf), x1, y1, x2, y2)

    results = {}
    for cls_id, (conf, x1, y1, x2, y2) in best_per_class.items():
        color = CLASS_NAMES.get(cls_id)
        if color is None:
            continue
        ox1 = (x1 - left) / scale
        oy1 = (y1 - top) / scale
        ox2 = (x2 - left) / scale
        oy2 = (y2 - top) / scale
        ow = ox2 - ox1
        oh = oy2 - oy1

        results[color] = {
            "x": int(round(ox1)), "y": int(round(oy1)),
            "width": int(round(ow)), "height": int(round(oh)),
            "center_x": int(round(ox1 + ow / 2)), "center_y": int(round(oy1 + oh / 2)),
            "confidence": conf,
        }
    return results

def detect_blocks_onnx(frame: np.ndarray, session, input_name: str) -> tuple:
    tensor, scale, left, top = preprocess(frame, MODEL_INPUT_SIZE)
    outputs = session.run(None, {input_name: tensor})
    decoded = decode_onnx_output(outputs[0], scale, left, top, CONF_THRESHOLD)
    return decoded.get("red"), decoded.get("green")

# ---------------------------------------------------------------------------
# Waypoint: A = block, B = robot at stop, C = pass point beside A
# Robot frame at freeze: B = (0, 0), +X = right, +Y = forward (camera axis)
# ---------------------------------------------------------------------------
def block_to_robot_xy(box: dict, frame_size: int) -> tuple:
    """Map a detection to robot-frame centimetres (A relative to B)."""
    h = max(int(box["height"]), 1)
    cx = float(box["center_x"])
    ccx = frame_size / 2.0

    # 640x480 squeezed to a square: width pixels are stretched vs height pixels.
    x_aspect = CAPTURE_W / float(CAPTURE_H)

    # Depth from the taped AB at AB_CAL_HEIGHT_PX (not STOP_HEIGHT; 30 px is farther).
    y_a = AB_DISTANCE_CM * (AB_CAL_HEIGHT_PX / float(h))

    # Lateral from similar triangles, using real pillar height vs box height.
    x_a = (cx - ccx) * x_aspect * (REAL_BLOCK_HEIGHT_CM / float(h))
    return x_a, y_a


def pass_point_c(x_a: float, y_a: float, color: str) -> tuple:
    """C is AC_OFFSET_CM horizontally from A. Red = pass right, green = pass left."""
    side = 1.0 if color == "red" else -1.0
    return x_a + side * AC_OFFSET_CM, y_a


def arc_b_to_c(x_c: float, y_c: float) -> dict:
    """
    Constant-curvature forward arc from B=(0,0) heading +Y to C=(x_c, y_c).
    Signed radius: + = right turn, - = left. Infinite radius = drive straight.
    """
    dist_sq = x_c * x_c + y_c * y_c
    dist = math.sqrt(dist_sq)

    if abs(x_c) < 0.5:
        return {
            "radius_cm": float("inf"),
            "theta_deg": 0.0,
            "arc_len_cm": abs(y_c),
            "turn": "straight",
        }

    radius = dist_sq / (2.0 * x_c)
    theta_rad = 2.0 * math.atan2(x_c, y_c) if y_c != 0 else math.copysign(math.pi, x_c)
    arc_len = abs(radius * theta_rad)
    return {
        "radius_cm": radius,
        "theta_deg": math.degrees(theta_rad),
        "arc_len_cm": arc_len,
        "turn": "right" if radius > 0 else "left",
        "chord_cm": dist,
    }


def median_detection_box(hist):
    """Median box over the last MIN_VOTES hits so A/C is not one noisy frame."""
    boxes = [b for b in hist if b is not None]
    if not boxes:
        return None
    use = boxes[-min(len(boxes), MIN_VOTES) :]

    def med(key):
        xs = sorted(b[key] for b in use)
        return xs[len(xs) // 2]

    return {
        "x": int(med("x")),
        "y": int(med("y")),
        "width": int(med("width")),
        "height": int(med("height")),
        "center_x": int(med("center_x")),
        "center_y": int(med("center_y")),
        "confidence": max(float(b["confidence"]) for b in use),
    }


def compute_waypoint(box: dict, color: str, frame_size: int) -> dict:
    x_a, y_a = block_to_robot_xy(box, frame_size)
    x_c, y_c = pass_point_c(x_a, y_a, color)
    arc = arc_b_to_c(x_c, y_c)
    wp = {
        "color": color,
        "A_cm": (x_a, y_a),
        "B_cm": (0.0, 0.0),
        "C_cm": (x_c, y_c),
        "AC_cm": AC_OFFSET_CM,
        "AB_cm": math.hypot(x_a, y_a),
        "box": {
            "center_x": box["center_x"],
            "center_y": box["center_y"],
            "width": box["width"],
            "height": box["height"],
        },
        **arc,
    }
    return wp


def format_waypoint_line(wp: dict) -> str:
    xa, ya = wp["A_cm"]
    xc, yc = wp["C_cm"]
    r = wp["radius_cm"]
    r_str = "inf" if math.isinf(r) else f"{r:.1f}"
    return (
        f"WAYPOINT,{wp['color']},{xa:.1f},{ya:.1f},{xc:.1f},{yc:.1f},"
        f"{r_str},{wp['theta_deg']:.1f},{wp['arc_len_cm']:.1f}\n"
    )


def print_waypoint(wp: dict) -> None:
    xa, ya = wp["A_cm"]
    xc, yc = wp["C_cm"]
    r = wp["radius_cm"]
    r_str = "straight" if math.isinf(r) else f"{r:.1f} cm ({wp['turn']})"
    print(
        f"WAYPOINT lock color={wp['color']} "
        f"A=({xa:.1f},{ya:.1f}) cm  B=(0,0)  C=({xc:.1f},{yc:.1f}) cm  "
        f"arc R={r_str}  theta={wp['theta_deg']:.1f} deg  len={wp['arc_len_cm']:.1f} cm"
    )

# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------
def upscale_for_display(frame_bgr: np.ndarray, scale: int = 3) -> np.ndarray:
    h, w = frame_bgr.shape[:2]
    return cv2.resize(frame_bgr, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)

def draw_boxes(frame_bgr: np.ndarray, red_box: dict, green_box: dict) -> np.ndarray:
    out = frame_bgr.copy()
    for box, bgr_color, label in ((red_box, (0, 0, 255), "RED"), (green_box, (0, 255, 0), "GREEN")):
        if not box:
            continue
        x, y, w, h = box['x'], box['y'], box['width'], box['height']
        cx, cy = box['center_x'], box['center_y']
        cv2.rectangle(out, (x, y), (x + w, y + h), bgr_color, 2)
        cv2.circle(out, (cx, cy), 3, bgr_color, -1)
        conf = box.get('confidence')
        conf_str = f" conf={conf:.2f}" if conf is not None else ""
        cv2.putText(out, f"{label} {w}x{h}px{conf_str}", (x, max(0, y - 22)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, bgr_color, 1)
        cv2.putText(out, f"pos=({x},{y}) center=({cx},{cy})", (x, max(0, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, bgr_color, 1)
    return out

# ---------------------------------------------------------------------------
# Camera capture (OpenCV - more reliable on Pi USB webcams than PyAV)
# ---------------------------------------------------------------------------
def resize_frame(frame: np.ndarray, target_w: int = 240, target_h: int = 240) -> np.ndarray:
    if frame is None or frame.size == 0:
        return None
    return cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)

def open_opencv_camera(index: int):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None
    print(f"Camera opened: index {index}, frame {frame.shape[1]}x{frame.shape[0]}")
    return cap

def start_capture_thread(camera_id: int, frame_size=240):
    frame_q = queue.Queue(maxsize=1)
    stop_flag = threading.Event()
    holder = {"cap": None}

    def enqueue(img):
        if img is None:
            return
        if frame_q.full():
            try:
                frame_q.get_nowait()
            except queue.Empty:
                pass
        frame_q.put(img)

    def capture_loop():
        indexes = []
        for i in (camera_id,) + CAMERA_INDEXES:
            if i not in indexes:
                indexes.append(i)

        while not stop_flag.is_set():
            cap = None
            for idx in indexes:
                if stop_flag.is_set():
                    break
                cap = open_opencv_camera(idx)
                if cap is not None:
                    holder["cap"] = cap
                    set_manual_camera_controls(idx, CAMERA_EXPOSURE, CAMERA_WB_TEMP)
                    break
            if cap is None:
                print(
                    "No camera frames. Unplug the webcam, plug it into a USB 2 port "
                    "(not blue USB 3), then: pkill -f detect.py && python detect.py"
                )
                time.sleep(2.0)
                continue
            try:
                while not stop_flag.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        print("Camera read failed; reopening...")
                        break
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    enqueue(resize_frame(rgb, frame_size, frame_size))
            finally:
                try:
                    cap.release()
                except Exception:
                    pass
                holder["cap"] = None
            if not stop_flag.is_set():
                time.sleep(0.4)

    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()
    return t, frame_q, stop_flag, holder

def set_manual_camera_controls(camera_id: int, exposure_value: int, wb_temperature: int):
    dev = f'/dev/video{camera_id}'
    cmds = [
        ['v4l2-ctl', '-d', dev, '-c', 'auto_exposure=1'],
        ['v4l2-ctl', '-d', dev, '-c', f'exposure_time_absolute={exposure_value}'],
        ['v4l2-ctl', '-d', dev, '-c', 'white_balance_automatic=0'],
        ['v4l2-ctl', '-d', dev, '-c', f'white_balance_temperature={wb_temperature}'],
    ]
    for cmd in cmds:
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Warning: could not run {' '.join(cmd)} ({e})")
    print(f"Camera controls locked: exposure={exposure_value}, wb_temp={wb_temperature}")

class _Tee:
    """Copy stdout/stderr to a log file with timestamps."""

    def __init__(self, stream, log_file):
        self.stream = stream
        self.log_file = log_file
        self._buf = ""

    def write(self, data):
        self.stream.write(data)
        self.stream.flush()
        self._buf += data
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            ts = time.strftime("%H:%M:%S")
            self.log_file.write(f"{ts} {line}\n")
            self.log_file.flush()

    def flush(self):
        self.stream.flush()
        self.log_file.flush()


def setup_run_log():
    here = os.path.dirname(os.path.abspath(__file__)) or "."
    path = os.path.join(here, LOG_NAME)
    log_file = open(path, "a", encoding="utf-8", buffering=1)
    log_file.write(f"\n===== start {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n")
    log_file.flush()
    sys.stdout = _Tee(sys.stdout, log_file)
    sys.stderr = _Tee(sys.stderr, log_file)
    print(f"Log file: {path}", flush=True)
    return path, log_file


def open_pi_serial():
    """Open ESP USB. Fall back to a plain Serial() if extra flags fail."""
    try:
        import serial
    except Exception as e:
        print("pyserial not installed: %s" % e, flush=True)
        return None
    last_err = None
    extra = dict(timeout=0.2, write_timeout=0.5, dsrdtr=False, rtscts=False)
    plain = dict(timeout=1)
    for port in SERIAL_PORTS:
        for kwargs in (extra, plain):
            try:
                ser = serial.Serial(port, SERIAL_BAUD, **kwargs)
                time.sleep(0.3)
                try:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                except Exception:
                    pass
                print("Serial port opened: %s" % port, flush=True)
                return ser
            except Exception as e:
                last_err = e
    print("Could not open serial port: %s" % last_err, flush=True)
    return None


def ascii_only(text):
    return "".join(ch for ch in text if 32 <= ord(ch) <= 126)


def start_esp_log_thread(ser, stop_flag):
    """Read ESP telemetry (MODE: GOTO-C, PI: STOP, ...) into the same log."""

    def loop():
        buf = ""
        while not stop_flag.is_set():
            try:
                n = ser.in_waiting
                if not n:
                    time.sleep(0.02)
                    continue
                raw = ser.read(n).decode("ascii", errors="ignore")
                buf += raw
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = ascii_only(line.strip("\r"))
                    if line:
                        print(f"ESP {line}", flush=True)
                if len(buf) > 400:
                    buf = ascii_only(buf[-80:])
            except Exception:
                break

    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return t

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(camera_id: int = CAMERA_ID, frame_size: int = FRAME_SIZE):
    print("detect.py start python=%s" % sys.version.split()[0], flush=True)
    log_path, log_file = setup_run_log()
    set_manual_camera_controls(camera_id, CAMERA_EXPOSURE, CAMERA_WB_TEMP)

    ser = open_pi_serial()
    esp_log_thread = None

    try:
        session, input_name, _ = load_onnx_session(ONNX_MODEL_PATH)
    except Exception as e:
        print("ONNX load failed: %s" % e, flush=True)
        print("Put best_ncnn.onnx next to round2.py", flush=True)
        raise

    t, frame_q, stop_flag, cam = start_capture_thread(camera_id, frame_size)
    if ser is not None:
        esp_log_thread = start_esp_log_thread(ser, stop_flag)

    def get_frame(timeout=1.0):
        try:
            return frame_q.get(timeout=timeout)
        except queue.Empty:
            return None

    print("Testing camera...")
    test_frames = 0
    for _ in range(15):
        frame = get_frame(timeout=1.0)
        if frame is not None:
            test_frames += 1
            print(f"Got test frame {test_frames}, shape: {frame.shape}")
            break

    if test_frames == 0:
        print("No frames received from camera!")
        print("Run:  pkill -f detect.py ; sudo fuser -k /dev/video0")
        print("Unplug the Lenovo cam, plug it into a USB 2 port (not blue USB 3).")
        print("Do not set CAMERA_ID = 1 - that is UVC metadata, not the picture.")
        stop_flag.set()
        t.join(timeout=2.0)
        if cam.get("cap") is not None:
            cam["cap"].release()
        return

    window_name = "WRO Block Detector (ONNX)"
    preview = True
    try:
        cv2.namedWindow(window_name)
    except Exception as e:
        preview = False
        print("No preview window (%s) - running headless" % e, flush=True)

    red_hist = deque(maxlen=VOTE_HISTORY)
    green_hist = deque(maxlen=VOTE_HISTORY)

    last_sent = None
    frame_count = 0
    clear_counter = 0
    waypoint_lock = None   # frozen A/B/C until the block is gone (CLEAR)
    waypoint_lock_time = 0.0
    last_wp_send_time = 0.0
    sent_stop_for_lock = False
    stop_streak = 0

    def serial_write(line: str) -> None:
        ser.write(line.encode("ascii"))
        ser.flush()
        print(f">>> Sent {line.strip()}", flush=True)

    try:
        while True:
            frame = get_frame(timeout=0.5)
            if frame is None:
                continue

            red_box, green_box = detect_blocks_onnx(frame, session, input_name)
            red_hist.append(red_box)
            green_hist.append(green_box)

            def confirmed(hist):
                return sum(1 for b in hist if b is not None) >= MIN_VOTES

            red_confirmed = confirmed(red_hist)
            green_confirmed = confirmed(green_hist)
            red_stable = median_detection_box(red_hist) if red_confirmed else None
            green_stable = median_detection_box(green_hist) if green_confirmed else None

            primary_box = None
            primary_color = None
            if red_confirmed and green_confirmed:
                if red_stable is not None and green_stable is not None:
                    primary_box, primary_color = (
                        (red_stable, "red") if red_stable["height"] >= green_stable["height"]
                        else (green_stable, "green")
                    )
                elif red_stable is not None:
                    primary_box, primary_color = red_stable, "red"
                elif green_stable is not None:
                    primary_box, primary_color = green_stable, "green"
            elif red_confirmed:
                primary_box, primary_color = red_stable, "red"
            elif green_confirmed:
                primary_box, primary_color = green_stable, "green"

            # ---- Decision: ignore until stop height, then freeze C; reverse if too close ----
            decision = "CLEAR"
            active_box = None

            if primary_box is not None:
                block_height = int(primary_box["height"])
                if block_height > REVERSE_HEIGHT_PX:
                    decision = "REVERSE"
                    active_box = primary_box
                    stop_streak = 0
                elif block_height >= STOP_HEIGHT_PX:
                    stop_streak += 1
                    if stop_streak >= LOCK_HOLD_FRAMES:
                        decision = "STOP"
                        active_box = primary_box
                else:
                    stop_streak = 0
            else:
                stop_streak = 0

            # Freeze A/B/C from the median box, not a one-frame height spike.
            # Stay locked even if height dips (that used to CLEAR and abort the ESP).
            if decision == "STOP" and waypoint_lock is None and active_box is not None and primary_color:
                waypoint_lock = compute_waypoint(active_box, primary_color, frame_size)
                print_waypoint(waypoint_lock)
                sent_stop_for_lock = False
                last_wp_send_time = 0.0
                waypoint_lock_time = time.time()
            elif decision == "REVERSE":
                waypoint_lock = None
                sent_stop_for_lock = False
                stop_streak = 0
            elif waypoint_lock is not None and (red_confirmed or green_confirmed):
                decision = "STOP"
                active_box = active_box or waypoint_lock.get("box")

            clear_counter = clear_counter + 1 if decision == "CLEAR" else 0
            if decision == "CLEAR" and clear_counter >= CLEAR_HISTORY:
                waypoint_lock = None
                sent_stop_for_lock = False
                stop_streak = 0

            now = time.time()
            if ser is None and frame_count % 30 == 0:
                print("Serial is NOT open - ESP will never STOP/WAYPOINT. Check /dev/ttyUSB*", flush=True)

            try:
                if ser is not None and decision == "REVERSE" and active_box is not None:
                    cmd_str = (
                        f"REVERSE,{active_box['center_x']},{active_box['center_y']},"
                        f"{active_box['width']},{active_box['height']}\n"
                    )
                    if cmd_str != last_sent:
                        serial_write(cmd_str)
                        last_sent = cmd_str
                elif ser is not None and decision == "STOP" and waypoint_lock is not None:
                    still_retrying = (now - waypoint_lock_time) <= WAYPOINT_RESEND_WINDOW_S
                    due = (now - last_wp_send_time) >= WAYPOINT_RESEND_S
                    if still_retrying and ((not sent_stop_for_lock) or due):
                        if not sent_stop_for_lock:
                            stop_box = waypoint_lock["box"]
                            serial_write(
                                f"STOP,{stop_box['center_x']},{stop_box['center_y']},"
                                f"{stop_box['width']},{stop_box['height']}\n"
                            )
                            time.sleep(0.03)
                            sent_stop_for_lock = True
                        wp_line = format_waypoint_line(waypoint_lock)
                        serial_write(wp_line)
                        last_sent = wp_line
                        last_wp_send_time = now
                elif ser is not None and decision == "CLEAR" and clear_counter >= CLEAR_HISTORY:
                    if last_sent != "CLEAR\n":
                        serial_write("CLEAR\n")
                        last_sent = "CLEAR\n"
            except Exception as e:
                print(f"Serial write failed: {e}", flush=True)

            h_now = primary_box["height"] if primary_box else 0
            if preview:
                try:
                    display_red = red_box if red_confirmed else None
                    display_green = green_box if green_confirmed else None
                    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    display = draw_boxes(bgr, display_red, display_green)
                    ser_txt = ser.port if ser is not None else "NO-SERIAL"
                    cv2.putText(
                        display,
                        f"{decision} h={h_now} stop={STOP_HEIGHT_PX} {ser_txt}",
                        (2, 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                        (0, 255, 255) if decision == "STOP" else (255, 255, 255), 1,
                    )
                    if waypoint_lock is not None:
                        xa, ya = waypoint_lock["A_cm"]
                        xc, yc = waypoint_lock["C_cm"]
                        cv2.putText(
                            display,
                            f"{waypoint_lock['color']} A=({xa:.0f},{ya:.0f}) C=({xc:.0f},{yc:.0f}) th={waypoint_lock['theta_deg']:.0f}",
                            (2, frame_size - 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1,
                        )
                    display = upscale_for_display(display, scale=3)
                    cv2.imshow(window_name, display)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        break
                except Exception as e:
                    preview = False
                    print("Preview failed, headless: %s" % e, flush=True)

            frame_count += 1
            print(f"Frame {frame_count} | {decision} | RED:{red_box} | GREEN:{green_box}", flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        stop_flag.set()
        t.join(timeout=2.0)
        if cam.get("cap") is not None:
            try:
                cam["cap"].release()
            except Exception:
                pass
        if ser is not None:
            ser.close()
        if log_file is not None:
            try:
                log_file.write(f"===== stop {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n")
                log_file.close()
            except Exception:
                pass
        if preview:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        raise
