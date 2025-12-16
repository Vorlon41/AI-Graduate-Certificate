# ReactivePython.py — Webots e-puck controller
#
# Devices:
#   Motors:   "left wheel motor", "right wheel motor"
#   Encoders: "left wheel sensor", "right wheel sensor"
#   IR:       ps0..ps7
#
# Behavior:
#   1) Drive forward until strong front IR hit (robust threshold with debounce).
#   2) Encoder-driven 180° spin (two-phase: fast then fine).
#   3) Go straight briefly (dead-zone) to clear the obstacle.
#   4) Continue forward; on the next hit: stop, encoder-driven ~90° spin,
#      then drive parallel for ~1.5 wheel turns and stop.

from controller import Robot
import math
import sys

# e-puck kinematics (meters).
WHEEL_RADIUS   = 0.0205   # m
AXLE_LENGTH    = 0.0630   # m  (tuned by you for perfect 180°)

# Device name fallbacks
LEFT_MOTOR_CANDIDATES   = ["left wheel motor", "left_motor", "left motor"]
RIGHT_MOTOR_CANDIDATES  = ["right wheel motor", "right_motor", "right motor"]
LEFT_ENCODER_CANDIDATES = ["left wheel sensor", "left_encoder", "left position sensor"]
RIGHT_ENCODER_CANDIDATES= ["right wheel sensor", "right_encoder", "right position sensor"]

IR_NAMES = [f"ps{i}" for i in range(8)]
FRONT_IDX = [0, 1, 6, 7]

# Motion tuning (e-puck max velocity ≈ 6.28 rad/s)
FWD_SPEED         = 3.5
TURN_SPEED_FAST   = 3.2
TURN_SPEED_SLOW   = 1.2
DEAD_ZONE_TIME    = 0.8
DEAD_ZONE_SPEED   = 4.0

# First turn (after first obstacle)
TURN_TARGET_DEG   = 180.0
SLOWDOWN_DEG      = 150.0
TURN_TOL_DEG      = 2.0

# Second behavior (after second obstacle)
SECOND_TURN_TARGET_DEG = 90.0         
SECOND_SLOWDOWN_DEG    = 70.0
SECOND_TURN_TOL_DEG    = 3.0
SECOND_TURN_DIR        = +1           
PARALLEL_TURNS         = 1.5          
PARALLEL_SPEED         = 3.2
PARALLEL_TIMEOUT       = 6.0         

# IR detection (robust trigger)
EMA_ALPHA          = 0.01            
ABS_MIN_HIT        = 78.0
REL_MIN_HIT        = 9.0
ROC_MIN_HIT        = 3.5             
REL_MIN_EARLY      = 6.0
DEBOUNCE_ON        = 3
DEBOUNCE_OFF       = 2

# Safety bounds
MAX_TURN_TIME     = 5.0
MAX_TURN_DEG      = 220.0
SECOND_MAX_TURN_TIME = 3.0
SECOND_MAX_TURN_DEG  = 130.0

# State IDs
TO_FIRST, STOP1, TURN180, STRAIGHT1, TO_SECOND, STOP2, TURN90, PARALLEL, DONE = range(9)
PRINT_EVERY_STEP = True
TAG = "ReactivePython"

# UTILS 
def deg(rad): return rad * 180.0 / math.pi
def clamp(v, m): return max(-m, min(m, v))
def mean(lst): return sum(lst) / max(1, len(lst))
def ema(prev, x, a): return x if prev is None else (1-a)*prev + a*x

# Robot setup
robot = Robot()
timestep = int(robot.getBasicTimeStep())

def get_device_or_none(name):
    try: return robot.getDevice(name)
    except Exception: return None

def get_first_device(candidates):
    for name in candidates:
        dev = get_device_or_none(name)
        if dev is not None:
            return dev
    return None

left_motor  = get_first_device(LEFT_MOTOR_CANDIDATES)
right_motor = get_first_device(RIGHT_MOTOR_CANDIDATES)
if left_motor is None or right_motor is None:
    print('ERROR: Could not find e-puck wheel motors. Expected "left wheel motor" & "right wheel motor".', flush=True)
    sys.exit(1)

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

try:  L_MAX = left_motor.getMaxVelocity()
except: L_MAX = 6.28
try:  R_MAX = right_motor.getMaxVelocity()
except: R_MAX = 6.28

def set_wheels(l, r):
    left_motor.setVelocity(clamp(l, L_MAX))
    right_motor.setVelocity(clamp(r, R_MAX))

# Encoders
left_ps  = get_first_device(LEFT_ENCODER_CANDIDATES)
right_ps = get_first_device(RIGHT_ENCODER_CANDIDATES)
if left_ps:  left_ps.enable(timestep)
if right_ps: right_ps.enable(timestep)

def get_encoders():
    l = left_ps.getValue() if left_ps else None
    r = right_ps.getValue() if right_ps else None
    return l, r

def yaw_from_encoders(start_l, start_r, cur_l, cur_r):
    if None in (start_l, start_r, cur_l, cur_r): return None
    d_l = (cur_l - start_l) * WHEEL_RADIUS
    d_r = (cur_r - start_r) * WHEEL_RADIUS
    return (d_r - d_l) / AXLE_LENGTH

def dist_from_encoders(start_l, start_r, cur_l, cur_r):
    if None in (start_l, start_r, cur_l, cur_r): return None
    d_l = (cur_l - start_l) * WHEEL_RADIUS
    d_r = (cur_r - start_r) * WHEEL_RADIUS
    return 0.5 * (d_l + d_r)

# IR sensors
ir = []
for name in IR_NAMES:
    ds = get_device_or_none(name)
    if ds is None:
        print(f"[WARN] IR '{name}' not found.", flush=True)
        ir.append(None)
    else:
        ds.enable(timestep)
        ir.append(ds)

def read_ir():
    vals = []
    for ds in ir:
        vals.append(ds.getValue() if ds else 0.0)
    return vals

def print_line(vals, baseline, state, note=None, extra=None):
    if not PRINT_EVERY_STEP: return
    front_vals = [vals[i] for i in FRONT_IDX]
    msg = (
        f"IR: {['{:.2f}'.format(v) for v in vals]} | "
        f"front={[float(f'{x:.2f}') for x in front_vals]} | "
        f"baseline={None if baseline is None else float(f'{baseline:.2f}')} | "
        f"state={state}"
    )
    if note:  msg += f" | {note}"
    if extra: msg += f" | {extra}"
    print(msg, flush=True)

#Hit logic (shared by first & second detection)
def front_hit_update(front_vals, baseline, prev_front_mean, debounce_on, debounce_off):
    front_mean = mean(front_vals)
    base_new   = ema(baseline, front_mean, EMA_ALPHA)
    roc        = 0.0 if prev_front_mean is None else (front_mean - prev_front_mean)
    peak       = max(front_vals)

    # Two ways to "hit"
    clause1 = (peak >= base_new + REL_MIN_HIT) and (peak >= ABS_MIN_HIT)
    clause2 = (roc  >= ROC_MIN_HIT) and (peak >= base_new + REL_MIN_EARLY)

    hit_on  = clause1 or clause2
    hit_off = (peak < base_new + REL_MIN_EARLY)

    # Keep a small debounce counter in the function scope by returning it to caller
    return hit_on, hit_off, base_new, front_mean, roc, peak

# State machine 
state = TO_FIRST

# shared rolling stats
baseline = None
prev_front_mean = None
deb = 0

# turn bookkeeping
turn_start_time = None
start_l = start_r = None

# second leg movement bookkeeping
parallel_start_l = parallel_start_r = None
parallel_deadline = None
PARALLEL_TARGET_DIST = PARALLEL_TURNS * (2.0 * math.pi * WHEEL_RADIUS)  # meters

print(f"INFO: {TAG}: Starting controller", flush=True)
print("Starting: TO_FIRST", flush=True)

while robot.step(timestep) != -1:
    vals = read_ir()
    front_vals = [vals[i] for i in FRONT_IDX]

    # Update detection stats (used in TO_FIRST / TO_SECOND only)
    hit_on, hit_off, baseline, front_mean, roc, peak = front_hit_update(
        front_vals, baseline, prev_front_mean, DEBOUNCE_ON, DEBOUNCE_OFF
    )
    prev_front_mean = front_mean

    # TO_FIRST: drive until strong IR hit
    if state == TO_FIRST:
        set_wheels(FWD_SPEED, FWD_SPEED)
        if hit_on: deb += 1
        elif hit_off: deb = max(0, deb-1)
        print_line(vals, baseline, state, note=f"peak={peak:.1f} base={baseline:.1f} roc={roc:.2f} deb={deb}")
        if deb >= DEBOUNCE_ON:
            print(">>> FIRST object detected -> STOP1", flush=True)
            set_wheels(0.0, 0.0)
            deb = 0
            state = STOP1
            continue

    # STOP1: begin 180 turn 
    elif state == STOP1:
        # clockwise spin (left fwd, right bwd)
        set_wheels(TURN_SPEED_FAST, -TURN_SPEED_FAST)
        turn_start_time = robot.getTime()
        start_l, start_r = get_encoders()
        print(">>> TURN180 (encoder-accurate)", flush=True)
        state = TURN180

    # TURN180: encoder-accurate 180°
    elif state == TURN180:
        now = robot.getTime()
        cur_l, cur_r = get_encoders()
        yaw = yaw_from_encoders(start_l, start_r, cur_l, cur_r)
        rot_deg = None if yaw is None else abs(deg(yaw))

        # phase switch
        if rot_deg is not None and rot_deg >= SLOWDOWN_DEG and abs(left_motor.getVelocity()) > TURN_SPEED_SLOW + 1e-6:
            set_wheels(TURN_SPEED_SLOW, -TURN_SPEED_SLOW)

        print_line(vals, baseline, state, note=f"rot_deg={None if rot_deg is None else f'{rot_deg:.1f}'}")

        if rot_deg is not None and rot_deg >= (TURN_TARGET_DEG - TURN_TOL_DEG):
            # center the band
            if rot_deg < TURN_TARGET_DEG:
                set_wheels(TURN_SPEED_SLOW * 0.6, -TURN_SPEED_SLOW * 0.6)
            elif rot_deg > (TURN_TARGET_DEG + TURN_TOL_DEG):
                set_wheels(-TURN_SPEED_SLOW * 0.6, TURN_SPEED_SLOW * 0.6)
            else:
                print(">>> STRAIGHT1 (dead-zone)", flush=True)
                set_wheels(DEAD_ZONE_SPEED, DEAD_ZONE_SPEED)
                state = STRAIGHT1
                dead_zone_end_time = robot.getTime() + DEAD_ZONE_TIME
                continue

        if (now - turn_start_time) > MAX_TURN_TIME or (rot_deg is not None and rot_deg > MAX_TURN_DEG):
            print(">>> TURN180 safety exit -> STRAIGHT1", flush=True)
            set_wheels(DEAD_ZONE_SPEED, DEAD_ZONE_SPEED)
            state = STRAIGHT1
            dead_zone_end_time = robot.getTime() + DEAD_ZONE_TIME
            continue

    # STRAIGHT1: obstacle dead-zone
    elif state == STRAIGHT1:
        print_line(vals, baseline, state, note="dead-zone")
        if robot.getTime() >= dead_zone_end_time:
            set_wheels(FWD_SPEED, FWD_SPEED)
            baseline = None
            prev_front_mean = None
            deb = 0
            print(">>> TO_SECOND (obstacle re-enabled)", flush=True)
            state = TO_SECOND
            continue

    # TO_SECOND: continue forward & watch for next object
    elif state == TO_SECOND:
        set_wheels(FWD_SPEED, FWD_SPEED)
        if hit_on: deb += 1
        elif hit_off: deb = max(0, deb-1)
        print_line(vals, baseline, state, note=f"peak={peak:.1f} base={baseline:.1f} roc={roc:.2f} deb={deb}")
        if deb >= DEBOUNCE_ON:
            print(">>> SECOND object detected -> STOP2", flush=True)
            set_wheels(0.0, 0.0)
            deb = 0
            state = STOP2
            continue

    # STOP2: prepare 90° turn 
    elif state == STOP2:
        # Choose direction via SECOND_TURN_DIR
        l =  SECOND_TURN_DIR * TURN_SPEED_FAST
        r = -SECOND_TURN_DIR * TURN_SPEED_FAST
        set_wheels(l, r)
        turn_start_time = robot.getTime()
        start_l, start_r = get_encoders()
        print(">>> TURN90 (encoder-accurate)", flush=True)
        state = TURN90

    # TURN90: encoder-accurate ~90° 
    elif state == TURN90:
        now = robot.getTime()
        cur_l, cur_r = get_encoders()
        yaw = yaw_from_encoders(start_l, start_r, cur_l, cur_r)
        rot_deg = None if yaw is None else abs(deg(yaw))

        # phase switch for 90°
        if rot_deg is not None and rot_deg >= SECOND_SLOWDOWN_DEG:
            set_wheels(SECOND_TURN_DIR * TURN_SPEED_SLOW, -SECOND_TURN_DIR * TURN_SPEED_SLOW)

        print_line(vals, baseline, state, note=f"rot_deg={None if rot_deg is None else f'{rot_deg:.1f}'}")

        if rot_deg is not None and rot_deg >= (SECOND_TURN_TARGET_DEG - SECOND_TURN_TOL_DEG):
            # center band
            inside = (abs(rot_deg - SECOND_TURN_TARGET_DEG) <= SECOND_TURN_TOL_DEG)
            if inside:
                # done turning -> start parallel run
                set_wheels(0.0, 0.0)
                parallel_start_l, parallel_start_r = get_encoders()
                parallel_deadline = robot.getTime() + PARALLEL_TIMEOUT
                set_wheels(PARALLEL_SPEED, PARALLEL_SPEED)
                print(">>> PARALLEL (distance ~1.5 wheel turns)", flush=True)
                state = PARALLEL
                continue
            elif rot_deg < SECOND_TURN_TARGET_DEG:
                set_wheels(SECOND_TURN_DIR * TURN_SPEED_SLOW * 0.6, -SECOND_TURN_DIR * TURN_SPEED_SLOW * 0.6)
            else:
                set_wheels(-SECOND_TURN_DIR * TURN_SPEED_SLOW * 0.6, SECOND_TURN_DIR * TURN_SPEED_SLOW * 0.6)

        if (now - turn_start_time) > SECOND_MAX_TURN_TIME or (rot_deg is not None and rot_deg > SECOND_MAX_TURN_DEG):
            print(">>> TURN90 safety exit -> PARALLEL", flush=True)
            set_wheels(0.0, 0.0)
            parallel_start_l, parallel_start_r = get_encoders()
            parallel_deadline = robot.getTime() + PARALLEL_TIMEOUT
            set_wheels(PARALLEL_SPEED, PARALLEL_SPEED)
            state = PARALLEL
            continue

    # PARALLEL: drive straight by encoder distance 
    elif state == PARALLEL:
        cur_l, cur_r = get_encoders()
        dist = dist_from_encoders(parallel_start_l, parallel_start_r, cur_l, cur_r)
        note = f"dist={None if dist is None else f'{dist:.3f}'} target={PARALLEL_TARGET_DIST:.3f}"
        print_line(vals, baseline, state, note=note)

        done = (dist is not None and dist >= PARALLEL_TARGET_DIST)
        timeout = (robot.getTime() >= parallel_deadline)
        if done or timeout:
            set_wheels(0.0, 0.0)
            print(">>> DONE (parallel segment complete)", flush=True)
            state = DONE
            continue

    # DONE 
    elif state == DONE:
        set_wheels(0.0, 0.0)
        # You can add any end-of-run behavior here (blink LEDs, etc.)
        # Keep stepping to allow console prints in Webots
        print_line(vals, baseline, state, note="idle")

# End of controller loop
