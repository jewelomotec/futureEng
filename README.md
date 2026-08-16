# WRO Future Engineers robot

Named inventory of **every file and function** in this repo:
[`docs/CODE_CATALOG.md`](docs/CODE_CATALOG.md).

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
2. Drives straight at PWM **120**, servo center **117** (set **106** on this car), PID holds heading. Speed ramps 30 → 120 over 2 s.
3. Front LiDAR **&lt; 15 cm for 150 ms** → pause **500 ms** (wheels centered) → **reverse-arc** with servo offset `ARC_SERVO_ANGLE` (20°).
4. When heading is within **8°** of the next cardinal (**0 / 90 / 180 / 270**), wheels center and it drives straight on that heading.
5. **First** corner: turn toward the side with more space (left vs right LiDAR). After that, **always that same direction**.
6. After **12** turns, if left and right are both under 100 cm and front under 150 cm for 1 s → race stop.

USB serial (115200) is for the Pi only. Debug `MODE:` lines also go to USB — do not open Serial Monitor while `detect.py` has the port.

### Blocks (Pi + ESP)

1. Pi must see the same colour in **5 of the last 7** frames (`CONF_THRESHOLD` 0.55).
2. Box height **≥ 30 px** for **2 frames** (median of last 5 hits, not a one-frame spike):
   - Robot pose = **B**, block = **A**, pass point **C** = A shifted **10 cm** sideways (red → right, green → left).
   - Pi sends `STOP,...` then `WAYPOINT,...`.
3. ESP sits **400 ms**, then drives **forward** to C (`MODE: GOTO-C`) with servo from radius **R**. Exit after **~3/4 of the arc time** if heading is within **12°**, or after the full `arc_len / 30 cm/s` time (4 s cap). Do not quit at 12° after only 250 ms.
4. Then PID on the **heading from before the stop**. No S-curve. Extra `STOP` can restart the hold; `CLEAR` can abort an arc still running.
5. When the block is gone for **10** frames, Pi sends `CLEAR`.
6. Height **&gt; 80 px**: Pi sends `REVERSE` → ESP backs up at −80 until `CLEAR` or 5 s.

Front LiDAR wall-turns only run in `DRIVING_STRAIGHT`. After `STOP` / `GOTO-C` the ESP ignores the front LiDAR for **`WAYPOINT_LIDAR_IGNORE_MS` (2.5 s)** so a wall behind the block does not steal the pass.

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

Pi prints and ESP serial (`MODE: GOTO-C`, `PI: STOP`, …) are appended to **`wro_detect.log`** next to `detect.py`. ESP lines are prefixed with `ESP`. With `round2`:

```bash
ls -l ~/Documents/Test2_Round2/wro_detect.log
tail -f ~/Documents/Test2_Round2/wro_detect.log
```

Do not open Serial Monitor while the detector has `/dev/ttyUSB0`.

### Capture

- OpenCV V4L2, 640×480 MJPEG, then resized to a **240×240** square for display/boxes.
- Inference letterboxes that frame to **224** — do not raise `MODEL_INPUT_SIZE` if you care about lag.
- Tries camera indexes **0, 1, 2** (on a Pi the webcam is often `video1`).
- If a USB glitch happens (camera cable moved), it prints `Camera read failed; reopening...` and continues.

### Geometry (measure these)

Robot frame at the freeze: **B = (0, 0)**, **+X right**, **+Y forward**.

| Variable | Default | Meaning |
|---|---|---|
| `STOP_HEIGHT_PX` | 30 | Freeze A,B,C and send WAYPOINT |
| `AB_DISTANCE_CM` | 40 | Tape **forward** distance to the pillar **at that same pixel height** |
| `AC_OFFSET_CM` | 10 | Pass-side nudge of robot center (field: 10 worked). Not “sit 20 cm beside the pillar” |
| `REAL_BLOCK_HEIGHT_CM` | 10 | Real pillar height (cm), for left/right position |
| `REVERSE_HEIGHT_PX` | 80 | Too close → `REVERSE` |
| `CONF_THRESHOLD` | 0.55 | ONNX score gate |
| `MIN_VOTES` / `VOTE_HISTORY` | 5 / 7 | Confirmation |
| `CLEAR_HISTORY` | 10 | Frames of no-block before `CLEAR` |
| `CAMERA_ID` | 0 | Lenovo picture node. **Do not use 1** (UVC metadata) |
| `CAMERA_FPS` | 15 | Lower than 30 to avoid Pi 5 USB3 xHCI overruns |
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
| `xhci buffer overrun` / UVC probe `-32` / `-71` | USB 3 bandwidth. Plug the Lenovo webcam into a **USB 2** port (not the blue USB 3). Put the ESP (`ttyUSB0`) on a different port. Unplug the cam 5 s. Use this OpenCV `detect.py` at 15 fps — do not open `/dev/video1` (metadata). |
| `No frames received from camera!` | `pkill -f detect.py`, unplug/replug on USB 2, `v4l2-ctl --list-devices`. Lenovo picture is `/dev/video0`; keep `CAMERA_ID = 0` |
| `avcodec_send_packet` / `av.AVError` | Old PyAV script. Copy the current `detect.py` (OpenCV) over `~/Documents/Test2_Round2/detect.py` |
| Overlay `h=` never hits 45 | Height is on the **240×240** working frame, not the 3× preview window. A box that looks ~135 px tall on screen is only ~45 px to the script |
| Overlay says `STOP` but the car does not | Need `Serial port opened` and `>>> Sent STOP` then `>>> Sent WAYPOINT`. If you see `NO-SERIAL` / `Serial is NOT open`, the ESP never gets the command |
| `>>> Sent STOP` but car never holds / GOTO-C | Re-flash this `.ino`. Duplicate STOP/CLEAR no longer abort the arc |
| Always `CLEAR \| RED:None \| GREEN:None` | Nothing in view, or model/conf too strict. Check the preview boxes |
| `cp210x ttyUSB0: failed set request ... -110` | Same USB stress as the camera. Separate ports; unplug/replug the ESP |
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

`INVERT_STEERING` is **true**. If GOTO-C or wall arcs steer the wrong physical way, flip that or check servo direction.

### Flash

Arduino IDE / ESP32, libraries: Adafruit BNO055, Adafruit Unified Sensor, ESP32Servo, (WiFi/WebServer/Preferences/ESPmDNS already in the core).

WiFi tuner (`RobotTuner` / `tunemybot`) is **commented out** in `setup()`. Leave it off for the competition. There is **no Bluetooth**.

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
| `STRAIGHT_SPEED` / `BACKWARD_SPEED` | **80** / −80 | Cruise / reverse PWM |
| `SERVO_CENTER` / `DIFF` | **106** / **45** | Center and max steer on this car |
| `FRONT_TURN_DISTANCE` | **20 cm** | Wall reverse-arc trigger (held 150 ms) |
| `ARC_PAUSE_MS` | 500 | Stand still before wall reverse |
| `ARC_SERVO_ANGLE` | **45°** | Wall reverse lock (full DIFF) |
| `ARC_EXIT_THRESHOLD` | 8° | Wall arc done |
| `ARC_MIN_MS` / `ARC_MAX_MS` | 400 / 4000 | Wall arc timing |
| `WAYPOINT_PAUSE_MS` | **400** | Sit after Pi `STOP` before GOTO-C |
| `WAYPOINT_LIDAR_IGNORE_MS` | **2500** | Do not start a wall reverse-arc during/after GOTO-C |
| `SIDE_AVOID_CM` | **12** | During GOTO-C, if L or R closer than this, steer away from that wall |
| `AFTER_C_PAUSE_MS` | **500** | Sit after arriving at C, then steer back to mid-lane |
| `RECENTER_SERVO` | **22°** | After C: red → **left**, green → **right** (opposite the pass) |
| `RECENTER_MAX_MS` | **1500** | Then original heading hold |
| `WAYPOINT_EXIT_DEG` | 12° | Arrived at C (looser than 8 → lock ends earlier) |
| `WHEELBASE_CM` | **13.0** | This bot axle-to-axle (cm). Maps Pi **R** → servo. 600 rpm N20 |
| `MAX_TURNS` | 12 | Then allow race stop |
| `OBSTACLE_TIMEOUT_MS` | 5000 | Auto-clear reverse / old avoid |

### How ESP uses a `WAYPOINT`

1. Parse `R`, `theta`, `arc_len`.
2. Target heading = heading at `STOP` **+ theta** (BNO heading increases on a right turn).
3. Servo from `atan(WHEELBASE_CM / |R|)`, clamped to `DIFF`, inverted if `INVERT_STEERING`.
4. Sit 400 ms, then drive the arc. Exit at 12° **only after ~75% of expected arc time**, or when that time is fully up, or at 4 s.
5. Center wheels; `straightTargetHeading` = heading from before the stop.

This sketch is **round1_2026 with Bluetooth removed**. There is no `SerialBT`. Extra `STOP` restarts the hold. `CLEAR` can abort `GOTO-C`.

If the pass is too wide (not enough curve), **raise** `WHEELBASE_CM`. Too tight: **lower** it, or on the Pi retape `AB_DISTANCE_CM`.

---

## Tuning order

1. Straight: `SERVO_CENTER` so it does not drift; then PID if needed.
2. Walls: `FRONT_TURN_DISTANCE` 15 cm, then `ARC_SERVO_ANGLE` / pause / exit.
3. Blocks: tape `AB_DISTANCE_CM` at **30 px** (the lock height). Then `AC_OFFSET_CM`.
4. ESP `WHEELBASE_CM` last, only if C is right but the curve is too soft/hard.

---

## Other files

Every program is named and described in [`docs/CODE_CATALOG.md`](docs/CODE_CATALOG.md).

| Official name | File | Use |
|---|---|---|
| Photo capture tool | `capture.py` | `r` / `g` to save cuboid photos |
| Dataset converter | `prepare.py` | Pascal VOC XML → YOLO zip |
| Training notebook | `Final_FutureEnginner.ipynb` | Colab training / ONNX export |
| Round 2 service | `deploy/round2.service` | systemd unit that starts `round2.py` |

Keep **one** detector process and **one** ESP sketch that understands `STOP`/`WAYPOINT`. Mixing an old PyAV `detect.py` with this ESP, or the new Pi script with an ESP that only knows `RED`/`GREEN`, will look like “nothing happens” on blocks.
