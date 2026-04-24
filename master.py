import threading
import time
import os
import serial
import serial.tools.list_ports
import signal
import sys

PIPE_PATH       = "/tmp/error_tracking"
WATER_PIPE_PATH = "/tmp/water_alert"
SERIAL_PORT     = "/dev/serial0"
BAUD_RATE       = 115200

CANS_PER_DEPOSIT = 40

# S-pattern sweep dimensions — calibrated on the competition field to cover a 4m-wide lane without overlap
S_TRAMO_RECTO = 1.8
S_TRAMO_CURVA = 1.5

MIN_CENTERED_FRAMES    = 4
MAX_N_TOLERANCE        = 10
REACHED_CAN_PAUSE_TIME = 2.0

# Trailer reverse timing — needed because the deposit bin was behind the robot in the arena layout
TURN_TO_DEPOSIT_TIME = 3.0
TURN_BACKUP_TIME     = 1.0

RED_CLOSE_WIDTH = 100

# PD gains — tuned in practice runs to stop the robot from oscillating while chasing cans on uneven sand
ERROR_THRESHOLD = 0.25
Kp              = 24.0
Kd              = 5.0

# Stuck recovery kept the robot autonomous after wheels dug into loose sand
STUCK_TIME    = 6.0
STUCK_BACKUP  = 1.5
STUCK_FORWARD = 1.0

# Water avoidance timing — values that reliably backed the robot off wet shoreline before getting stuck
WATER_STOP_TIME     = 0.4
WATER_MIN_BACK      = 1.5
WATER_AVOID_TIMEOUT = 8.0
WATER_STUCK_TIME    = 3.0
WATER_EXTRA_BACK    = 2.0
WATER_TURN_TIME     = 2.5

# States where a water interrupt is safe to issue without corrupting the task sequence
ESTADOS_INTERRUMPIBLES = {
    "SEARCHING",
    "APPROACHING",
    "SEARCHING_RED",
    "APPROACHING_RED",
    "TURNING_TO_DEPOSIT",
    "WATER_CLEAR_BACK",
}


class KalmanFilter1D:
    # 1D Kalman smoothed noisy centroid readings and reduced jitter commands sent to the ESP32
    def __init__(self, q=0.01, r=0.10):
        self.q = q; self.r = r
        self.x = 0.0; self.p = 1.0

    def update(self, z):
        self.p += self.q
        k       = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1.0 - k)
        return self.x

    def reset(self):
        self.x = 0.0; self.p = 1.0


class StuckDetector:
    # Same-command timeout recovered from wheel-spin situations that cost us time in early competition heats
    ESTADOS_ACTIVOS = ("SEARCHING", "APPROACHING", "SEARCHING_RED", "APPROACHING_RED")

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_cmd       = None
        self.same_cmd_since = time.time()
        self.unsticking     = False
        self.unstick_start  = None
        self.unstick_fase   = None

    def update(self, cmd_actual, estado):
        if estado not in self.ESTADOS_ACTIVOS:
            self.reset()
            return False
        if cmd_actual != self.last_cmd:
            self.last_cmd       = cmd_actual
            self.same_cmd_since = time.time()
        if time.time() - self.same_cmd_since >= STUCK_TIME:
            if not self.unsticking:
                self.unsticking    = True
                self.unstick_start = time.time()
                self.unstick_fase  = "BACK"
                log("Stuck detected — starting recovery")
            return True
        return False

    def execute(self):
        t = time.time() - self.unstick_start
        if self.unstick_fase == "BACK":
            ciclo = int(t / 0.2) % 2
            enviar("B" if ciclo == 0 else "F")
            if t >= 1.2:
                self.unstick_fase  = "ESCAPE"
                self.unstick_start = time.time()
        elif self.unstick_fase == "ESCAPE":
            enviar("CR")
            if t >= STUCK_FORWARD:
                self.reset()
                return False
        return True


print_lock         = threading.Lock()
ultimo_dato        = None
tiempo_ultimo_dato = time.time()
dato_lock          = threading.Lock()

water_lock     = threading.Lock()
water_detected = False
water_side     = "C"

running = True

def log(msg):
    with print_lock:
        print(f"[{time.strftime('%H:%M:%S')}] {msg}")


def hilo_lectura():
    global ultimo_dato, tiempo_ultimo_dato, running
    log("Waiting for vision pipe...")
    while not os.path.exists(PIPE_PATH):
        time.sleep(0.5)
    log("Vision pipe found.")
    with open(PIPE_PATH, "r") as pipe:
        while running:
            linea = pipe.readline()
            if not linea:
                log("Vision pipe closed.")
                running = False
                break
            linea = linea.strip()
            if linea:
                with dato_lock:
                    ultimo_dato        = linea
                    tiempo_ultimo_dato = time.time()


def hilo_lectura_agua():
    global water_detected, water_side, running
    log("Waiting for water pipe...")
    while not os.path.exists(WATER_PIPE_PATH):
        time.sleep(0.5)
    log("Water pipe found.")
    with open(WATER_PIPE_PATH, "r") as pipe:
        while running:
            linea = pipe.readline()
            if not linea:
                log("Water pipe closed.")
                break
            linea = linea.strip()
            if not linea:
                continue
            with water_lock:
                if linea.startswith("W"):
                    water_detected = True
                    partes     = linea.split(":")
                    water_side = partes[1] if len(partes) > 1 else "C"
                else:
                    water_detected = False


ser_lock = threading.Lock()
ser_obj  = None

def _abrir_serial():
    # Blocking retry loop let us hot-plug the ESP32 USB cable without restarting the robot
    while True:
        try:
            s = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            log(f"ESP32 connected on {SERIAL_PORT}")
            return s
        except serial.SerialException:
            log("ESP32 unavailable, retrying in 2s...")
            time.sleep(2)

def conectar_serial():
    global ser_obj
    ser_obj = _abrir_serial()

def hilo_watchdog_serial():
    global ser_obj, running
    while running:
        time.sleep(1.0)
        with ser_lock:
            if ser_obj is None or not ser_obj.is_open:
                log("Serial dropped — reconnecting...")
                try:
                    if ser_obj: ser_obj.close()
                except Exception:
                    pass
                ser_obj = _abrir_serial()

def enviar(cmd):
    global ser_obj
    with ser_lock:
        if ser_obj is None or not ser_obj.is_open:
            return
        try:
            ser_obj.write((str(cmd) + "\n").encode())
        except serial.SerialException:
            log(f"Send error '{cmd}' — watchdog will reconnect")
            try:
                ser_obj.close()
            except Exception:
                pass

def leer_dato():
    with dato_lock:
        # 500ms staleness guard prevents acting on camera frames that got stuck in the pipe
        if time.time() - tiempo_ultimo_dato > 0.5:
            return "N"
        return ultimo_dato

def parsear_dato(dato):
    if dato is None or dato == "N":
        return {"tipo": "N", "error": 0.0, "ancho": 0}
    partes = dato.split(":")
    if len(partes) >= 2 and partes[0] in ("CAN", "RED"):
        try:
            ancho = int(partes[2]) if len(partes) >= 3 else 0
            return {"tipo": partes[0], "error": float(partes[1]), "ancho": ancho}
        except ValueError:
            pass
    return {"tipo": "N", "error": 0.0, "ancho": 0}


class PatronS:
    # S-sweep search pattern covered the designated arena strip without driving off the boundaries
    def __init__(self):
        self.reset()

    def reset(self):
        self.fase       = "RECTO"
        self.direccion  = "DER"
        self.fase_start = time.time()

    def update(self):
        elapsed = time.time() - self.fase_start
        if self.fase == "RECTO":
            enviar("F")
            if elapsed >= S_TRAMO_RECTO:
                self.fase       = "CURVA"
                self.fase_start = time.time()
        elif self.fase == "CURVA":
            enviar("CR" if self.direccion == "DER" else "CL")
            if elapsed >= S_TRAMO_CURVA:
                self.direccion  = "IZQ" if self.direccion == "DER" else "DER"
                self.fase       = "RECTO"
                self.fase_start = time.time()


def maquina_de_estados():
    global running

    estado       = "SEARCHING"
    estado_start = time.time()

    cans_collected      = 0
    cans_total          = 0
    centered_count      = 0
    frames_perdidos     = 0
    ultimo_comando      = 0.0
    deposito_abierto    = False
    water_lado_guardado = "C"

    kalman_can = KalmanFilter1D()
    kalman_red = KalmanFilter1D()

    prev_error_can = 0.0
    prev_error_red = 0.0
    last_tick      = time.time()

    patron_s = PatronS()
    stuck    = StuckDetector()

    def cambiar_estado(nuevo, razon=""):
        nonlocal estado, estado_start, centered_count, frames_perdidos
        nonlocal prev_error_can, prev_error_red, ultimo_comando, deposito_abierto
        log(f"{estado} → {nuevo}  ({razon})")
        estado           = nuevo
        estado_start     = time.time()
        centered_count   = 0
        frames_perdidos  = 0
        ultimo_comando   = 0.0
        deposito_abierto = False
        prev_error_can   = 0.0
        prev_error_red   = 0.0
        kalman_can.reset()
        kalman_red.reset()
        patron_s.reset()
        stuck.reset()

    def elapsed():
        return time.time() - estado_start

    enviar("S")
    enviar("M")
    log(f"Starting SEARCHING — {CANS_PER_DEPOSIT} cans per deposit cycle")

    while running:
        now       = time.time()
        dt        = now - last_tick
        last_tick = now

        dato = parsear_dato(leer_dato())

        # Water check runs first every tick — protected the robot from entering the ocean in all competition runs
        with water_lock:
            agua_activa = water_detected
            lado_agua   = water_side

        if agua_activa and estado in ESTADOS_INTERRUMPIBLES:
            enviar("S")
            log(f"[WATER] Detected at {lado_agua} during {estado} → evading")
            cambiar_estado("AVOIDING_WATER", f"Water at {lado_agua}")
            time.sleep(0.05)
            continue

        elif stuck.update(ultimo_comando, estado):
            if stuck.execute():
                time.sleep(0.08)
                continue
            else:
                log("Recovery done — resuming")
                stuck.reset()
                patron_s.reset()

        elif estado == "SEARCHING":
            patron_s.update()
            ultimo_comando = "S_PATRON"
            if dato["tipo"] == "CAN":
                enviar("J")
                cambiar_estado("APPROACHING", "Can detected")

        elif estado == "APPROACHING":
            if dato["tipo"] == "CAN":
                frames_perdidos = 0
                error_k = kalman_can.update(dato["error"])
                d_error = (error_k - prev_error_can) / max(dt, 0.001)
                corr    = (Kp * error_k) + (Kd * d_error)
                prev_error_can = error_k
                ultimo_comando = corr
                enviar(round(corr, 2))
                if abs(error_k) < ERROR_THRESHOLD:
                    centered_count += 1
                else:
                    centered_count = max(0, centered_count - 1)
            else:
                frames_perdidos += 1
                if frames_perdidos < MAX_N_TOLERANCE:
                    # Coasting on last known correction kept alignment through brief camera occlusions
                    enviar(round(ultimo_comando, 2))
                else:
                    if centered_count >= MIN_CENTERED_FRAMES:
                        cans_collected += 1
                        cans_total     += 1
                        enviar("S")
                        log(f"✓ CAN COLLECTED — Pausing {REACHED_CAN_PAUSE_TIME}s")
                        cambiar_estado("REACHED_CAN", f"Can {cans_collected}/{CANS_PER_DEPOSIT}")
                    else:
                        enviar("M")
                        cambiar_estado("SEARCHING", "Can missed")

        elif estado == "REACHED_CAN":
            enviar("S")
            if elapsed() >= REACHED_CAN_PAUSE_TIME:
                enviar("M")
                if cans_collected >= CANS_PER_DEPOSIT:
                    cans_collected = 0
                    cambiar_estado("SEARCHING_RED", "Cycle complete, seeking deposit")
                else:
                    cambiar_estado("SEARCHING", "Seeking next can")

        elif estado == "SEARCHING_RED":
            patron_s.update()
            ultimo_comando = "S_PATRON"
            if dato["tipo"] == "RED":
                cambiar_estado("APPROACHING_RED", "Red bin detected")

        elif estado == "APPROACHING_RED":
            if dato["tipo"] == "RED":
                frames_perdidos = 0
                error_k = kalman_red.update(dato["error"])
                d_error = (error_k - prev_error_red) / max(dt, 0.001)
                corr    = (Kp * error_k) + (Kd * d_error)
                prev_error_red = error_k
                ultimo_comando = corr
                enviar(round(corr, 2))
                log(f"RED width={dato['ancho']}px  error={error_k:.3f}")
                if dato["ancho"] >= RED_CLOSE_WIDTH:
                    enviar("S")
                    cambiar_estado("TURNING_TO_DEPOSIT",
                                   f"Bin at {dato['ancho']}px — trailer maneuver")
            else:
                frames_perdidos += 1
                if frames_perdidos < MAX_N_TOLERANCE:
                    enviar(round(ultimo_comando, 2))
                else:
                    enviar("M")
                    cambiar_estado("SEARCHING_RED", "Bin lost, retrying")

        elif estado == "TURNING_TO_DEPOSIT":
            # Timed turn + backup positioned the hopper door directly over the bin opening
            t = elapsed()
            if t < TURN_TO_DEPOSIT_TIME:
                enviar("CR")
            elif t < TURN_TO_DEPOSIT_TIME + TURN_BACKUP_TIME:
                enviar("BL")
            else:
                enviar("S")
                cambiar_estado("DEPOSITING", "Positioned, depositing")

        elif estado == "DEPOSITING":
            if not deposito_abierto:
                enviar("DO")
                deposito_abierto = True
            enviar("BL")
            if elapsed() >= 2.0:
                enviar("DC")
                enviar("S")
                log(f"Deposit complete — total={cans_total}")
                cambiar_estado("SEARCHING", "Deposit done, continuing")

        elif estado == "AVOIDING_WATER":
            with water_lock:
                agua_ahora = water_detected
                lado_ahora = water_side

            t_ev = elapsed()

            if t_ev >= WATER_AVOID_TIMEOUT:
                enviar("S")
                log("Water avoidance timeout — resuming search")
                cambiar_estado("SEARCHING", "Water timeout")

            elif not agua_ahora and t_ev >= WATER_MIN_BACK:
                enviar("S")
                log("Water cleared — starting extra reverse")
                water_lado_guardado = lado_ahora
                cambiar_estado("WATER_CLEAR_BACK", f"Side={lado_ahora}")

            elif t_ev < WATER_STOP_TIME:
                enviar("S")

            elif t_ev > WATER_STOP_TIME + WATER_STUCK_TIME:
                # Rock-back used when wheels were spinning in wet sand during shoreline contact
                ciclo = int((t_ev - WATER_STOP_TIME) / 1.0) % 2
                if ciclo == 0:
                    enviar("BK")
                    log("⚠ Stuck in water — rocking BK")
                else:
                    enviar("BA")
                    log("⚠ Stuck in water — rocking BA")

            else:
                # Diagonal reverse away from the detected water side to clear the shoreline faster
                if lado_ahora == "L":
                    enviar("BK")
                elif lado_ahora == "R":
                    enviar("BA")
                else:
                    enviar("BK")

        elif estado == "WATER_CLEAR_BACK":
            t_ev = elapsed()

            if t_ev < WATER_EXTRA_BACK:
                if water_lado_guardado == "R":
                    enviar("BA")
                else:
                    enviar("BK")

            elif t_ev < WATER_EXTRA_BACK + WATER_TURN_TIME:
                # Turn direction chosen based on which side water was on to head back inland
                if water_lado_guardado == "R":
                    enviar("CL")
                else:
                    enviar("CR")

            else:
                enviar("S")
                log("Evasion complete — resuming search")
                cambiar_estado("SEARCHING", "Water avoided")

        time.sleep(0.08)   # ~12 Hz loop matched the camera FPS without overloading the serial link

    log("FSM stopped.")


def apagado_limpio(signum=None, frame=None):
    global running
    log("Clean shutdown — stopping ESP32...")
    running = False
    for _ in range(3):
        enviar("S")
        enviar("M")
        time.sleep(0.05)
    with ser_lock:
        if ser_obj and ser_obj.is_open:
            ser_obj.close()
    log("ESP32 stopped. Done.")
    sys.exit(0)


def main():
    global running, tiempo_ultimo_dato

    signal.signal(signal.SIGINT,  apagado_limpio)
    signal.signal(signal.SIGTERM, apagado_limpio)

    conectar_serial()

    t_watchdog = threading.Thread(target=hilo_watchdog_serial, daemon=True)
    t_watchdog.start()

    t_pipe = threading.Thread(target=hilo_lectura, daemon=True)
    t_pipe.start()

    t_agua = threading.Thread(target=hilo_lectura_agua, daemon=True)
    t_agua.start()

    log("Waiting for camera calibration to finish...")
    while ultimo_dato is None and running:
        time.sleep(0.1)
        with dato_lock:
            tiempo_ultimo_dato = time.time()

    maquina_de_estados()
    apagado_limpio()

if __name__ == "__main__":
    main()
