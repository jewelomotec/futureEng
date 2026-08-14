# WRO Future Engineers robot

Two programs run together:

| Piece | File | Runs on | Job |
|---|---|---|---|
| Drive / steering / walls | `ESP32_Robot/ESP32_Robot.ino` | ESP32 | Motors, servo, LiDAR, IMU, serial from the Pi |
| Blocks | `detect.py` (same code as `wro_block_detector.py`) | Raspberry Pi | Camera + ONNX, sends `STOP` / `WAYPOINT` / `REVERSE` / `CLEAR` |

The ESP drives the open-challenge **walls** by itself. The Pi only speaks up when it sees a **red or green block**. USB serial is 115200 baud (Pi `/dev/ttyUSB0` or `/dev/ttyUSB1`).

```
Pi camera  -->  detect.py  -->  USB serial  -->  ESP32
LiDARs + IMU  ------------------------------>  ESP32  -->  motor + servo
```

---

## What a run looks like

### Walls (LiDAR + IMU, no Pi needed)

1. ESP waits for BNO055 gyro calibration (up to 10 s).
2. Drives straight at PWM **80**, servo center **117**, PID holds heading. Speed ramps 30 → 80 over 2 s.
3. Front LiDAR **&lt; 10 cm for 150 ms** → pause **500 ms** (wheels centered) → **reverse-arc** with servo offset `ARC_SERVO_ANGLE` (20°).
4. When heading is within **8°** of the next cardinal (**0 / 90 / 180 / 270**), wheels center and it drives straight on that heading.
5. **First** corner: turn toward the side with more space (left vs right LiDAR). After that, **always that same direction**.
6. After **12** turns, if left and right are both under 100 cm and front under 150 cm for 1 s → race stop.

Bluetooth telemetry: `MODE: STRAIGHT`, then `PAUSE`, then `ARC`.

### Blocks (Pi + ESP)

1. Pi must see the same colour in **5 of the last 7** frames (`CONF_THRESHOLD` 0.55).
2. Box height **≥ 45 px** (try **30** if the pass is too sharp):
   - Robot pose = **B**, block = **A**, pass point **C** = A shifted **25 cm** sideways (red → right, green → left).
   - Pi sends `STOP,...` then `WAYPOINT,...`.
3. ESP: motors off ~**400 ms** (`MODE: PI-HOLD`), then drives **forward** to C (`MODE: GOTO-C`) with servo set from radius **R**. IMU exits when heading is within **8°** of (heading at stop + theta).
4. Then PID holds the **heading from before the stop** (same lane).
5. When the block is gone for **10** frames, Pi sends `CLEAR`.
6. Height **&gt; 80 px**: Pi sends `REVERSE` → ESP backs up at −80 until `CLEAR` or 5 s.

Front LiDAR wall-turns only run in `DRIVING_STRAIGHT`. At **10 cm** they should not steal a block the camera already stopped for (~45 px is much farther). After `GOTO-C`, if you are still aimed at a wall under 10 cm, a reverse-arc can still start.

---

## Raspberry Pi — `detect.py`

### What you need

- Raspberry Pi, USB webcam, USB serial to the ESP (`/dev/ttyUSB0` is tried first)
- Python packages: `opencv-python`, `numpy`, `onnxruntime`, `pyserial`
- Model file **`best_ncnn.onnx`** next to the script (input **224×224**, classes `green=0`, `red=1`)
- Optional: `v4l2-ctl` for exposure / white balance

`detect.py` and `wro_block_detector.py` are the same program. On the Pi, run **one** copy only.

```bash
pkill -f detect.py
python detect.py
```

You should see `Camera opened: index N` and `Serial port opened: /dev/ttyUSBx`.  
`Failed to detect devices under /sys/class/drm/...` from ONNX is harmless (CPU is used).

Quit with `q` in the preview window or Ctrl+C.

### Capture

- OpenCV V4L2, 640×480 MJPEG, then resized to a **240×240** square for display/boxes.
- Inference letterboxes that frame to **224** — do not raise `MODEL_INPUT_SIZE` if you care about lag.
- Tries camera indexes **0, 1, 2** (on a Pi the webcam is often `video1`).
- If a USB glitch happens (camera cable moved), it prints `Camera read failed; reopening...` and continues.

### Geometry (measure these)

Robot frame at the freeze: **B = (0, 0)**, **+X right**, **+Y forward**.

| Variable | Default | Meaning |
|---|---|---|
| `STOP_HEIGHT_PX` | 45 | Stop / freeze A,B,C. **30** = farther = gentler arc |
| `AB_DISTANCE_CM` | 40 | Tape **forward** distance to the pillar **at that same pixel height** |
| `AC_OFFSET_CM` | 25 | How far beside the block to pass |
| `REAL_BLOCK_HEIGHT_CM` | 10 | Real pillar height (cm), for left/right position |
| `REVERSE_HEIGHT_PX` | 80 | Too close → `REVERSE` |
| `CONF_THRESHOLD` | 0.55 | ONNX score gate |
| `MIN_VOTES` / `VOTE_HISTORY` | 5 / 7 | Confirmation |
| `CLEAR_HISTORY` | 10 | Frames of no-block before `CLEAR` |
| `CAMERA_ID` | 0 | Force 1 if index 0 is not the webcam |
| `SERIAL_PORTS` | USB0, USB1, AMA0 | First port that opens wins |

If you change 45 → 30, **remeasure `AB_DISTANCE_CM`**. Keeping 40 cm at 30 px makes C too close and the arc stays tight.

640×480 squeezed to 240×240 stretches width vs height; the script corrects lateral cm with `CAPTURE_W / CAPTURE_H`.

### Serial lines the Pi sends

Only when the command **changes** (CLEAR is delayed until `CLEAR_HISTORY` frames).

```text
CLEAR
STOP,<center_x>,<center_y>,<width>,<height>
WAYPOINT,<color>,<xa>,<ya>,<xc>,<yc>,<R>,<theta_deg>,<arc_len>
REVERSE,<center_x>,<center_y>,<width>,<height>
```

Example:

```text
STOP,132,110,28,47
WAYPOINT,red,8.2,38.3,33.2,38.3,42.1,52.0,38.5
```

- `color`: `red` or `green`
- `xa,ya` / `xc,yc`: centimetres in the robot frame
- `R`: signed radius, cm (`inf` = straight). **+ = right**
- `theta_deg`: heading change, degrees (**+ = right**)
- `arc_len`: path length, cm

The Pi **does not** send `RED` / `GREEN` any more. The ESP still accepts them if you type them.

### Pi troubleshooting

| Symptom | What to do |
|---|---|
| `[Errno 16] Device or resource busy` | Another `detect.py` still has the camera. `pkill -f detect.py` then `fuser -v /dev/video0` |
| `No frames received from camera!` | `ls /dev/video*`, set `CAMERA_ID = 1`, only one script running |
| `avcodec_send_packet` / `av.AVError` | Old PyAV script. Use the current `detect.py` (OpenCV) |
| Always `CLEAR \| RED:None \| GREEN:None` | Nothing in view, or model/conf too strict. Check the preview boxes |
| Serial never opens | ESP not on USB, or wrong port. Unplug/replug; `ls /dev/ttyUSB*` |

Dataset helpers (not used at race time): `capture.py` (save red/green photos), `prepare.py` (XML → YOLO).

---

## ESP32 — `ESP32_Robot/ESP32_Robot.ino`

### Hardware (as wired in the sketch)

| Item | Pins / notes |
|---|---|
| Steering servo | GPIO **13** |
| Drive motor | IN1 **25**, IN2 **26**, PWM **33** |
| I2C | SDA **21**, SCL **22** |
| Mux (HW-617 @ `0x70`) | LiDAR L/C/R channels **0 / 1 / 2**, BNO055 channel **4** |
| TF-Luna | `0x10` |
| BNO055 | `0x28` |
| USB serial from Pi | 115200 |
| Bluetooth | name `ESP32_Robot_Telemetry` |

`INVERT_STEERING` is **true**. If GOTO-C or wall arcs steer the wrong physical way, flip that or check servo direction.

### Flash

Arduino IDE / ESP32, libraries: Adafruit BNO055, Adafruit Unified Sensor, ESP32Servo, (WiFi/WebServer/Preferences/ESPmDNS already in the core).

WiFi tuner (`RobotTuner` / `tunemybot`) is **commented out** in `setup()`. Uncomment `setupWifiTuner()` if you want the slider page at `http://192.168.4.1`. Bluetooth `NAME=VALUE` still works without WiFi (not saved to flash unless you use the web `/set`).

### States

| `MODE` (telemetry) | Meaning |
|---|---|
| `STRAIGHT` | PID heading hold, cruise 80 |
| `PAUSE` / `ARC` | Wall reverse-arc |
| `PI-HOLD` | Pi `STOP` — motors off |
| `GOTO-C` | Pi waypoint, forward arc |
| `REVERSE` | Pi too-close backup |
| `AVOID` | Legacy `RED`/`GREEN` full-lock swerve |

Race finish (`ROBOT_STOPPED`) is not a Pi command; it is the 12-turn + boxed-in LiDAR condition.

### ESP tunables

| Variable | Default | Role |
|---|---|---|
| `STRAIGHT_SPEED` / `BACKWARD_SPEED` | 80 / −80 | Cruise / reverse PWM |
| `SERVO_CENTER` / `DIFF` | 117 / 25 | Center and max steer |
| `FRONT_TURN_DISTANCE` | **10 cm** | Wall reverse-arc trigger (held 150 ms) |
| `ARC_PAUSE_MS` | 500 | Stand still before wall reverse |
| `ARC_SERVO_ANGLE` | 20° | How sharp the **wall** reverse is |
| `ARC_EXIT_THRESHOLD` | 8° | Wall arc done |
| `ARC_MIN_MS` / `ARC_MAX_MS` | 400 / 4000 | Wall arc timing |
| `WAYPOINT_PAUSE_MS` | 400 | Stand still after Pi `STOP` |
| `WAYPOINT_EXIT_DEG` | 8° | Arrived at C |
| `WHEELBASE_CM` | 18 | Maps Pi **R** → servo. Lower = more steer for the same R |
| `MAX_TURNS` | 12 | Then allow race stop |
| `OBSTACLE_TIMEOUT_MS` | 5000 | Auto-clear reverse / old avoid |

Bluetooth examples: `FRONT=10`, `CENTER=117`, `STRAIGHT=80`, `ARCANGLE=20`.

### How ESP uses a `WAYPOINT`

1. Parse `R`, `theta`, `arc_len`.
2. Target heading = heading at `STOP` **+ theta** (BNO heading increases on a right turn).
3. Servo from `atan(WHEELBASE_CM / |R|)`, clamped to `DIFF`, inverted if `INVERT_STEERING`.
4. Drive forward at 80 until heading is close, or `WAYPOINT_MAX_MS` (4 s).
5. Center wheels; `straightTargetHeading` = heading from before the stop.

If the pass is too wide, lower `WHEELBASE_CM`. Too tight: raise it, or on the Pi use 30 px + new `AB_DISTANCE_CM`.

---

## Tuning order

1. Straight: `SERVO_CENTER` so it does not drift; then PID if needed.
2. Walls: `FRONT_TURN_DISTANCE` 10 cm, then `ARC_SERVO_ANGLE` / pause / exit.
3. Blocks: tape `AB_DISTANCE_CM` at 45 px. Run one pillar. If the arc is sharp, `STOP_HEIGHT_PX = 30` and tape AB again. Then `AC_OFFSET_CM`.
4. ESP `WHEELBASE_CM` last, only if C is right but the curve is too soft/hard.

---

## Other files

| File | Use |
|---|---|
| `capture.py` | Webcam tool: `r` / `g` to save cuboid photos |
| `prepare.py` | Convert Pascal VOC XML to YOLO and zip a dataset |
| `Final_FutureEnginner.ipynb` | Training notebook |

Keep **one** detector process and **one** ESP sketch that understands `STOP`/`WAYPOINT`. Mixing an old PyAV `detect.py` with this ESP, or the new Pi script with an ESP that only knows `RED`/`GREEN`, will look like “nothing happens” on blocks.
