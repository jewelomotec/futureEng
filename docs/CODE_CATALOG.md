# Code catalog — names and descriptions

This file is the named inventory of **every program in this repository**.
Race-day how-to stays in the root `README.md`. This catalog is what each
file is called, where it runs, and what every major function does.

GitHub branch with the current race stack:
`cursor/waypoint-stop-latch-e87a`
(PR https://github.com/jewelomotec/futureEng/pull/5)

---

## Official names (what to call each piece)

| Official name | Filename in the repo | Where it runs | Race? |
|---|---|---|---|
| **Drive firmware** | `ESP32_Robot/ESP32_Robot.ino` | ESP32 | Yes — motors, servo, walls, waypoint consumer |
| **Block detector** | `detect.py` | Raspberry Pi | Yes — camera + ONNX + serial |
| **Block detector (GitHub name)** | `wro_block_detector.py` | Raspberry Pi (copy of `detect.py`) | Same program; do not run both |
| **Pi race copy** | `round2.py` | Pi disk only (`~/Documents/Test2_Round2/`) | Yes — **not stored in git**; paste `detect.py` into this name |
| **Round 2 service** | `deploy/round2.service` | Pi systemd (`round2`) | Yes — starts `round2.py` at boot |
| **Service notes** | `deploy/README.md` | Human | How to install the unit |
| **Photo capture tool** | `capture.py` | Laptop / Pi | No — dataset photos |
| **VOC→YOLO converter** | `prepare.py` | Training PC | No — builds `dataset.zip` |
| **Training notebook** | `Final_FutureEnginner.ipynb` | Google Colab | No — trains `best.pt` / export ONNX |
| **Race README** | `README.md` | Human | Wiring, tunables, troubleshooting |
| **This catalog** | `docs/CODE_CATALOG.md` | Human | Names and function lists |

Runtime files **not in git** (created on the Pi):

| Name | Typical path | Role |
|---|---|---|
| `best_ncnn.onnx` | next to `round2.py` | ONNX model (green=0, red=1, input 224) |
| `wro_detect.log` | next to `round2.py` | Pi stdout + ESP `MODE:` lines |
| `venv/` | `Test2_Round2/venv` | Python with opencv / onnxruntime / pyserial |
| `/etc/systemd/system/round2.service` | Pi | Installed copy of the unit |

Do not confuse:

- **Wall reverse-arc** (LiDAR, pause 500 ms, **reverse**) ≠ **block waypoint** (camera, **forward** `GOTO-C`).
- **`round2`** = systemd unit name. **`round2.py`** = Python file that unit starts.
- **`detect.py` / `wro_block_detector.py` / `round2.py`** = the same detector if you pasted the latest code. Only **one** process.
- Repo `SERVO_CENTER` default is **117**. This car uses **106**.

---

## 1. Drive firmware — `ESP32_Robot/ESP32_Robot.ino`

**Name:** Drive firmware (ESP32 robot sketch).

**Job:** Drive straight on a BNO055 heading, reverse-arc at walls, consume Pi
serial (`STOP` / `WAYPOINT` / `REVERSE` / `CLEAR`), stop after 12 boxed-in turns.

**Does not:** Open the camera or run ONNX. There is no Bluetooth. WiFi tuner
exists in the file but is commented out in `setup()`.

**Hardware named in the sketch**

| Name | Value |
|---|---|
| Steering servo | GPIO 13 |
| Motor IN1 / IN2 / PWM | 25 / 26 / 33 |
| I2C SDA / SCL | 21 / 22 |
| Mux HW-617 | 0x70, LiDAR L/C/R = ch 0/1/2, BNO = ch 4 |
| TF-Luna | 0x10 |
| BNO055 | 0x28 |
| USB serial | 115200 |

**States (`RobotState`)**

| Enum | Telemetry `MODE` | What the car does |
|---|---|---|
| `DRIVING_STRAIGHT` | `STRAIGHT` | PID heading hold, cruise PWM 120 |
| `TURNING` + `PHASE_PAUSE` | `PAUSE` | Wall: sit 500 ms, wheels centered |
| `TURNING` + `PHASE_REVERSE` | `ARC` | Wall: reverse with servo ±20° to next cardinal |
| `PI_HOLD` | `PI-HOLD` | Pi `STOP`: motors off until `WAYPOINT` |
| `WAYPOINT_ARC` | `GOTO-C` | Forward arc using Pi `R` and IMU `theta` |
| `PI_REVERSE` | `REVERSE` | Pi too-close backup |
| `OBSTACLE_AVOIDING` | `AVOID` | Legacy `RED`/`GREEN` full-lock swerve |
| `ROBOT_STOPPED` | (no drive) | 12 turns + L/R &lt; 100 cm, front &lt; 150 cm |

There is **no** `PI_RECENTER` / S-curve and **no Bluetooth**. After `GOTO-C` it
returns to `STRAIGHT` on the heading from before `STOP`. This is round1_2026
logic: extra `STOP` restarts the 400 ms hold; `CLEAR` can abort an arc.

**Functions (named)**

| Function | Description |
|---|---|
| `selectMuxChannel` | PCA9548-style mux: talk to one LiDAR or the BNO |
| `getLunaDistance` | Read TF-Luna cm on the selected mux channel |
| `getCurrentHeading` / `getSmoothedHeading` | Raw and filtered BNO heading |
| `snapToCardinal` | Nearest of 0 / 90 / 180 / 270 |
| `computeTurnTarget` | Next cardinal left or right of current heading |
| `shortestAngleDiff` / `wrapHeading` | Heading arithmetic |
| `setMotorOutput` | Signed PWM: +forward, −reverse |
| `getRampedSpeed` | 30→120 over 2 s after a standstill |
| `checkFrontObstacle` | Front &lt; 15 cm for 150 ms → start wall turn |
| `driveStraightMode` | PID on `straightTargetHeading` |
| `reversingArcServoAngle` | Wall reverse: crank opposite the intended turn |
| `executeTurnMode` | Pause then reverse until heading close or timeout |
| `finishArcTurn` | Count a wall turn, resume straight on new cardinal |
| `servoAngleFromRadius` | `atan(WHEELBASE_CM / \|R\|)` → servo, respects invert |
| `handlePiLine` | Split first CSV field: STOP / WAYPOINT / REVERSE / CLEAR / RED / GREEN |
| `handlePiStop` | Enter `PI_HOLD` (restarts if STOP arrives again) |
| `handlePiWaypoint` | Parse R, theta, arc_len; arm `waypointReady` |
| `handlePiReverse` / `handlePiClear` | Backup; CLEAR resumes even mid-arc |
| `executePiHold` | Wait `WAYPOINT_PAUSE_MS` (400) then start `GOTO-C` |
| `executeWaypointArc` | Forward at 120 until IMU or timer; then straight |
| `executePiReverse` | Reverse until timeout |
| `avoidObstacle` | Legacy full-lock swerve |
| `printTelemetry` | USB `MODE:` line (Pi logs these as `ESP …`) |
| `loadTunables` / `saveTunables` / `handleRoot` | WiFi tuner (disabled in `setup`) |
| `setup` / `loop` | Calibrate BNO, then 20 ms cycle: serial, LiDAR, state machine |

**Flash notes:** keep `INVERT_STEERING = true`, `WHEELBASE_CM = 13.0` (this bot,
600 rpm N20), set `SERVO_CENTER = 106` if that is still this car’s centre. Do not
open Serial Monitor while the Pi owns `/dev/ttyUSB0`.

---

## 2. Block detector — `detect.py`

**Name:** Block detector (canonical race Python in git).

**Aliases:** `wro_block_detector.py` (byte-for-byte same). On the Pi the service
runs a pasted copy named **`round2.py`**.

**Job:** Open Lenovo webcam (`/dev/video0` picture node), run `best_ncnn.onnx`,
vote 5 of 7 frames, median box, 2 frames at `STOP_HEIGHT_PX`, freeze A/B/C, send `STOP` then
`WAYPOINT` on USB 115200. Log Pi prints and ESP lines to `wro_detect.log`.

**Does not:** Steer the servo or read LiDAR. Does not send `RED`/`GREEN`
(those are legacy; ESP still accepts them).

**Constants (named)**

| Name | Default | Meaning |
|---|---|---|
| `ONNX_MODEL_PATH` | `best_ncnn.onnx` | Model file next to the script |
| `MODEL_INPUT_SIZE` | 224 | Letterbox size for ONNX |
| `CLASS_NAMES` | 0 green, 1 red | Class ids |
| `CONF_THRESHOLD` | 0.55 | Score gate |
| `FRAME_SIZE` | 240 | Working square after 640×480 capture |
| `CAMERA_ID` | 0 | Picture node; **never 1** (UVC metadata) |
| `CAMERA_INDEXES` | (0, 2) | Probe list |
| `CAMERA_FPS` | 15 | Avoid Pi 5 USB3 overruns |
| `SERIAL_PORTS` | USB0, USB1, AMA0 | First that opens |
| `STOP_HEIGHT_PX` | 30 | Freeze waypoint |
| `LOCK_HOLD_FRAMES` | 2 | Consecutive STOP-height frames before freeze |
| `AB_DISTANCE_CM` | 40 | Taped depth at `AB_CAL_HEIGHT_PX` |
| `AB_CAL_HEIGHT_PX` | 45 | Scale for Y; 30 px lock → ~60 cm |
| `AC_OFFSET_CM` | 10 | Pass-side nudge (field-tested). C often earlier than the block |
| `REVERSE_HEIGHT_PX` | 80 | Too close → `REVERSE` |
| `WAYPOINT_RESEND_S` | 0.4 | Retry interval |
| `WAYPOINT_RESEND_WINDOW_S` | 1.2 | Then stop sending until CLEAR |
| `LOG_NAME` | `wro_detect.log` | Combined log |

**Functions (named)**

| Function | Description |
|---|---|
| `load_onnx_session` | CPU ONNX Runtime session |
| `preprocess` | Letterbox to 224, NCHW float32 |
| `decode_onnx_output` | Ultralytics (1,300,6) → pixel boxes on the 240 frame |
| `detect_blocks_onnx` | Best red box and best green box, or None |
| `block_to_robot_xy` | Box → A in robot cm (B at origin, +X right, +Y forward) |
| `pass_point_c` | C = A ± `AC_OFFSET_CM` |
| `arc_b_to_c` | Circle through B and C, tangent to +Y: `R`, `theta`, `arc_len` |
| `compute_waypoint` | Freeze A, B, C, arc, and the pixel box |
| `median_detection_box` | Median of last 5 hits — freeze A from this, not a spike |
| `format_waypoint_line` | `WAYPOINT,color,xa,ya,xc,yc,R,theta,arclen` |
| `print_waypoint` | Human A/B/C dump |
| `open_opencv_camera` | V4L2 MJPG 640×480 15 fps |
| `start_capture_thread` | Background grabber; skip video1; reopen on USB glitch |
| `set_manual_camera_controls` | Optional `v4l2-ctl` exposure / WB |
| `_Tee` / `setup_run_log` | Mirror stdout to `wro_detect.log` |
| `start_esp_log_thread` | Read ESP USB, prefix `ESP` |
| `draw_boxes` / `upscale_for_display` / `resize_frame` | Preview overlay |
| `main` | Votes, lock, serial, OpenCV window |

**Serial lines this program sends**

```text
STOP,<cx>,<cy>,<w>,<h>
WAYPOINT,<color>,<xa>,<ya>,<xc>,<yc>,<R>,<theta_deg>,<arc_len>
REVERSE,<cx>,<cy>,<w>,<h>
CLEAR
```

---

## 3. Block detector (GitHub name) — `wro_block_detector.py`

**Name:** Block detector, GitHub filename.

**Description:** Identical to `detect.py`. Exists so downloads from GitHub are
obviously the ONNX waypoint script, not an old colour-threshold `detect.py`.
If you save it on the Pi as `round2.py`, paste from either file.

---

## 4. Pi race copy — `round2.py` (not in git)

**Name:** Pi race copy.

**Description:** The file systemd actually starts. Same source as `detect.py`.
Keep it in `/home/pi/Documents/Test2_Round2/round2.py` next to `best_ncnn.onnx`
and `venv/`. After updating git, paste again and `sudo systemctl restart round2`.

---

## 5. Round 2 service — `deploy/round2.service`

**Name:** Round 2 systemd unit.

**Unit name on the Pi:** `round2` (`systemctl status round2`).

**Description:** Runs as user `pi`, working directory
`/home/pi/Documents/Test2_Round2`, `ExecStart` … `python -u` … `round2.py`,
`DISPLAY=:0`, restart always. `-u` is required so logs are not buffered.

Install: copy to `/etc/systemd/system/round2.service`, then `daemon-reload`
and `restart round2`. Details: `deploy/README.md`.

---

## 6. Photo capture tool — `capture.py`

**Name:** Photo capture tool.

**Job:** Open camera 0, live window. `r` saves `red/red_NNNN.jpg`, `g` saves
`green/green_NNNN.jpg`, `q` quits. Used only to collect training pictures.
Not started by `round2`.

---

## 7. VOC→YOLO converter — `prepare.py`

**Name:** Dataset converter.

**Job:** Read Pascal VOC XML from `label_red` / `label_green`, map class names
(`red`/`red_b` → 1, `green`/`green_b` → 0), 80/20 train/val split, write YOLO
txt + `dataset.yaml` (`names: ['green', 'red']`), zip `dataset.zip` for Colab.

**Note:** `BASE` is a Windows path from the original training PC. Change `BASE`
before running on another machine.

---

## 8. Training notebook — `Final_FutureEnginner.ipynb`

**Name:** Training notebook (filename spelling is historical).

**Job:** Google Colab (T4) pipeline: unpack `dataset.zip`, train YOLO, export
weights. Race cars need the exported **`best_ncnn.onnx`**, not this notebook.

---

## 9. Race README — `README.md`

**Name:** Race README.

**Job:** How a run looks (walls vs blocks), Pi capture settings, serial format,
ESP pinout, tunables, troubleshooting. Points here for the named file list.

---

## 10. Service notes — `deploy/README.md`

**Name:** Round 2 service install notes.

**Job:** Copy/edit the unit, `daemon-reload`, `restart round2`, `journalctl`.
Expect `Loaded ONNX model` and `>>> Sent WAYPOINT`.
