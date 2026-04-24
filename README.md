# 🤖 BeachBot — Autonomous Beach Can Collector

Autonomous robot built for a robotics competition. It navigates a beach strip, collects black cans using computer vision, avoids the ocean, and deposits the cans into a red bin — all without human intervention.

---

## How it works

The system runs on a **Raspberry Pi** and communicates with an **ESP32** motor controller over serial. Two Python processes run in parallel and talk through Linux named pipes (`FIFO`):

```
[Camera] → vision.py ──/tmp/error_tracking──→ maestro.py → ESP32 → Motors
                    └──/tmp/water_alert──────→ maestro.py
```

**`vision.py`** — Reads frames from a USB camera and detects:
- **Black cans** via HSV masking + strict shape filters (aspect ratio, circularity, convexity)
- **Red deposit bin** via dual HSV range (wraps around 0°/180° hue)
- **Ocean water** via blue HSV mask over the full frame

**`maestro.py`** — Finite state machine that reads pipe messages and drives the robot:

```
SEARCHING → APPROACHING → REACHED_CAN → (repeat N times)
         → SEARCHING_RED → APPROACHING_RED → TURNING_TO_DEPOSIT → DEPOSITING
         → AVOIDING_WATER → WATER_CLEAR_BACK → SEARCHING
```

---

## Architecture

| Component | Role |
|---|---|
| Raspberry Pi | Vision processing + FSM (Python) |
| ESP32 | Motor controller (receives string commands over UART) |
| USB Camera (640×480) | Input for OpenCV pipeline |
| Named pipes (FIFO) | IPC between vision and master processes |

---

## Key techniques

- **Auto-calibration** — brightness threshold computed from median frame luminance at startup, solved inconsistent detection across different lighting conditions at the venue
- **1D Kalman filter** — smoothed lateral error before PD control, eliminated jitter commands to the ESP32
- **PD control** — proportional + derivative correction for can/bin approach (`Kp=24`, `Kd=5`)
- **S-pattern sweep** — timed forward + curve sequence that covered the competition lane without GPS
- **Centroid tracking** — kept the robot locked on the same can across frames, prevented switching targets mid-approach
- **Water side detection** — largest blue contour centroid determines L/R side, used for a directed curved reverse instead of a blind backstep
- **Stuck detector** — same-command timeout triggers a rock-back-and-turn sequence, recovered from wheel spin in loose sand
- **Serial watchdog** — background thread reconnects the ESP32 if the USB cable is jostled

---

## Serial command reference

| Command | Action |
|---|---|
| `F` | Forward |
| `B` | Backward |
| `CR` / `CL` | Curve right / left |
| `BK` / `BA` / `BL` | Back-right / back-left diagonals |
| `S` | Stop |
| `M` | Enable motors |
| `J` | Short forward jolt |
| `DO` / `DC` | Open / close deposit hopper |
| `{float}` | PD correction value (signed, range ~−50 to +50) |

---

## Setup

**Requirements**
```
Python 3.9+
opencv-python
numpy
pyserial
```

**Install**
```bash
pip install opencv-python numpy pyserial
```

**Run**
```bash
# Terminal 1 — vision process
python vision.py

# Terminal 2 — master FSM (starts after pipe is open)
python maestro.py
```

The vision process blocks on the FIFO until maestro connects — this is expected behavior for named pipes.

---

## Tunable parameters

| Parameter | File | Description |
|---|---|---|
| `DANGER_ON / DANGER_OFF` | `vision.py` | Blue pixel ratio thresholds for water detection |
| `CANS_PER_DEPOSIT` | `maestro.py` | Cans collected before triggering deposit cycle |
| `Kp, Kd` | `maestro.py` | PD controller gains for approach |
| `S_TRAMO_RECTO / CURVA` | `maestro.py` | S-pattern segment durations (seconds) |
| `STUCK_TIME` | `maestro.py` | Seconds of same command before triggering recovery |

---

## Competition results

The robot successfully ran fully autonomous collection and deposit cycles during the competition. Water avoidance triggered correctly on all shoreline contacts across multiple runs, and the stuck recovery prevented any manual interventions during judged heats.

---

## License

MIT
