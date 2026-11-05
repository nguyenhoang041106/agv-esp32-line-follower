# main.py -- ESP32 AGV, MicroPython, TB6612FNG driver
#
# Save this file on the ESP32 as  main.py  

# (weighted-average line error with PD steering). 
#
# v3.2: identical to v3 except (1) AVOID_LEG1_CM 18 -> 23, (2) AVOID_RETURN_MAX_CM 35 -> 45,
#       (3) NEW LOST_STRAIGHT_MS: after losing a centred line the robot drives straight for
#       at most this long, then stops (v3 drove straight for up to 3000 ms).
# v3: first-run settings applied (faster line following, compact detour,
# measured calibration). TEST_MODE = None means it runs autonomously.
# For any re-test, change TEST_MODE to one of the modes below, then back to None.

from machine import Pin, PWM, disable_irq, enable_irq
from time import ticks_ms, ticks_us, ticks_diff, ticks_add, sleep_ms, sleep_us
import micropython

micropython.alloc_emergency_exception_buf(100)

# ======================================================================
# 0. MODE SELECT
# ======================================================================
# None       : autonomous line following + obstacle avoidance
# 'sensors'  : print the 5 line sensors (stage 1)
# 'motors'   : spin each wheel forward then backward, wheels LIFTED (stage 2)
# 'sonar'    : print ultrasonic distance (stage 3)
# 'turn'     : spin left 1 s then right 1 s -> measure the angles (calibration)
# 'forward'  : drive straight 1 s -> measure the distance (calibration)
TEST_MODE = None

START_DELAY_MS = 2000      # time to put the robot down after power-up (0 = none)

# ======================================================================
# 1. PINS  
# ======================================================================
# TB6612FNG channel A = LEFT motor, channel B = RIGHT motor 
PWMA_PIN = 13   # TB6612 PWMA   
AIN1_PIN = 12   # TB6612 AIN1   
AIN2_PIN = 14   # TB6612 AIN2   
PWMB_PIN = 15   # TB6612 PWMB   
BIN1_PIN = 4    # TB6612 BIN1   
BIN2_PIN = 2    # TB6612 BIN2   
# STBY is wired to 3.3 V (always enabled) - no GPIO used.

# Line sensors, listed LEFT -> RIGHT 
#   OUT1=GPIO34 (outer-left), OUT2=32, OUT3=33 (middle), OUT4=25, OUT5=27 (outer-right)
SENSOR_PINS = (34, 32, 33, 25, 27)
SENSOR_WEIGHTS = (-2, -1, 0, 1, 2)      # negative = line on the LEFT
# On-line = pin reads LOW (same as original onLine()).

TRIG_PIN = 21
ECHO_PIN = 19   # 3.3 V sensor supply -> connect directly 

# ======================================================================
# 2. MOTOR / SPEED SETTINGS (fixed constants - no phone speed control)
# ======================================================================
PWM_FREQ = 1000          # Hz 
MAX_PWM = 200            # hard cap, protects small motors on a 2-cell pack (0..255)
MIN_RUN_PWM = 70         # below this a motor may not start 
L_SCALE = 1.00           # wheel balance
R_SCALE = 1.05

BASE_PWM = 150           # cruising PWM on the line. If it leaves the line on curves: 135, then 120
JUNCTION_FACTOR = 0.8    # >=3 sensors on -> slow down 
RECOVERY_PWM = 135       # spin speed when line is lost (0.9 x BASE_PWM)
ALLOW_REVERSE = False    # False: steering never reverses a wheel (it just stops it)

# ======================================================================
# 3. LINE-FOLLOWING PD(I) SETTINGS  
# ======================================================================
LOOP_MS = 10
KP = 30.0                # PWM per unit of error (error range -2..+2)
KI = 0.0                 # leave 0 until KP/KD are good
KD = 0.7                 # PWM per (error unit / second)
D_SMOOTH = 0.6          # 0..1, higher = smoother derivative
I_LIMIT = 40.0           # max PWM contribution of the I term

RECOV_TIME_MS = 3000     # spin-search time after the line is lost 
LOST_STRAIGHT_MS = 1000  # NEW (v3.2): line vanished while centred -> go straight at most this
                         # long, then stop. v3 allowed the full RECOV_TIME_MS (3000 ms).
BAD_HOLD_MS = 3000       # all-off/all-on for this long -> stop 

# ======================================================================
# 4. ULTRASONIC 
# ======================================================================
OBSTACLE_ON_CM = 20.0
OBSTACLE_OFF_CM = 20.0
OBS_HIT_N = 2
US_TIMEOUT_US = 8000
SONAR_PERIOD_MS = 40     # min time between pings (module needs >= ~40-60 ms)

# ======================================================================
# 5. TIMED AVOIDANCE (replaces encoder-based turns/distances)
# ======================================================================
# Without encoders the manoeuvre is TIME based. It stays DISABLED (robot just
# stops in front of the obstacle) until you calibrate and set CALIBRATED = True:
#   1. TEST_MODE='turn'    -> measure degrees turned in 1 s  -> TURN_RATE_DEG_S
#   2. TEST_MODE='forward' -> measure cm travelled in 1 s    -> FWD_SPEED_CM_S
CALIBRATED = True    # you measured TURN_RATE_DEG_S and FWD_SPEED_CM_S below
TURN_PWM = 120
FWD_PWM = 110
TURN_RATE_DEG_S = 225.0   # measured at TURN_PWM (redo test 'turn' if TURN_PWM changes)
FWD_SPEED_CM_S = 18.0     # measured at FWD_PWM (redo test 'forward' if FWD_PWM changes)

ANGLE_FACTOR = 1.0
DIST_FACTOR = 1.0
STEP_PAUSE_MS = 200

# Detour shape (cm). About 21 cm sideways clearance with these values.
AVOID_LEG1_CM = 23.0          # sideways leg  -> larger = wider detour (v3: 18.0, +5 cm for bigger obstacles)
AVOID_LEG2_CM = 28.0          # leg that passes the obstacle's length
AVOID_RETURN_MAX_CM = 45.0    # longest the robot drives looking for the line (v3: 35.0, raised to match leg 1)
SPIN_TIMEOUT_MS = 5000

# ======================================================================
# Hardware objects
# ======================================================================
a_in1 = a_in2 = b_in1 = b_in2 = None
a_pwm = b_pwm = None
sensors = []
trig = echo = None

NONE = 0
LEFT = 1
RIGHT = 2


# ======================================================================
# Motor driver (TB6612FNG)
# ======================================================================
def _set_motor(in1, in2, pwm, v, scale):
    d = int(abs(v) * scale + 0.5)
    if d > MAX_PWM:
        d = MAX_PWM
    if v > 0:
        in1.value(1)
        in2.value(0)
    elif v < 0:
        in1.value(0)
        in2.value(1)
    else:
        in1.value(0)         # both low = coast
        in2.value(0)
        d = 0
    pwm.duty_u16(d * 257)    # 0..255 -> 0..65535


def _shape(v):
    # dead-band: never command a tiny PWM that cannot move the motor
    if v > 0 and v < MIN_RUN_PWM:
        return MIN_RUN_PWM
    if v < 0 and v > -MIN_RUN_PWM:
        return -MIN_RUN_PWM
    return v


def drive(left, right):
    """left/right: signed PWM, +forward -backward."""
    left = _shape(int(left))
    right = _shape(int(right))
    _set_motor(a_in1, a_in2, a_pwm, left, L_SCALE)
    _set_motor(b_in1, b_in2, b_pwm, right, R_SCALE)


def stop():
    a_in1.value(0)
    a_in2.value(0)
    b_in1.value(0)
    b_in2.value(0)
    a_pwm.duty_u16(0)
    b_pwm.duty_u16(0)


# ======================================================================
# Line sensors
# ======================================================================
def read_sensors():
    return [p.value() == 0 for p in sensors]          # True = on line


def line_error(on):
    n = 0
    s = 0
    for i in range(5):
        if on[i]:
            n += 1
            s += SENSOR_WEIGHTS[i]
    if n == 0:
        return 0, 0.0
    return n, s / n                                   # + = line on the right


# ======================================================================
# Non-blocking HC-SR04 (echo timed by pin interrupt)
# ======================================================================
echo_t0 = -1
echo_dur = 0
echo_new = False
sonar_waiting = False
sonar_ping_t = 0
sonar_hist = [999.0, 999.0, 999.0]
us_dist_cm = 999.0
obs_hit = 0
obs_latched = False


def echo_isr(pin):
    global echo_t0, echo_dur, echo_new
    t = ticks_us()
    if pin.value():
        echo_t0 = t
    elif echo_t0 != -1:
        echo_dur = ticks_diff(t, echo_t0)
        echo_t0 = -1
        echo_new = True


def _sonar_ping():
    global echo_t0, echo_new, sonar_waiting, sonar_ping_t
    echo_t0 = -1
    echo_new = False
    trig.value(1)
    sleep_us(10)
    trig.value(0)
    sonar_ping_t = ticks_ms()
    sonar_waiting = True


def _dur_to_cm(dur):
    if dur <= 0 or dur > US_TIMEOUT_US:
        return 999.0
    d = dur * 0.0343 / 2.0
    if d < 2.0 or d > 200.0:                          # reject glitches
        return 999.0
    return d


def _sonar_push(d):
    global us_dist_cm, obs_hit, obs_latched
    h = sonar_hist
    h[0] = h[1]
    h[1] = h[2]
    h[2] = d
    us_dist_cm = h[0] + h[1] + h[2] - min(h) - max(h)   # median of 3
    if not obs_latched:
        if us_dist_cm > 0 and us_dist_cm <= OBSTACLE_ON_CM:
            obs_hit += 1
            if obs_hit >= OBS_HIT_N:
                obs_latched = True
        else:
            obs_hit = 0
    else:
        if us_dist_cm >= OBSTACLE_OFF_CM:
            obs_latched = False
            obs_hit = 0


def sonar_update():
    """Call often. Never blocks (except the 10 us trigger pulse)."""
    global sonar_waiting, echo_new
    now = ticks_ms()
    if sonar_waiting:
        if echo_new:
            s = disable_irq()
            dur = echo_dur
            echo_new = False
            enable_irq(s)
            sonar_waiting = False
            _sonar_push(_dur_to_cm(dur))
        elif ticks_diff(now, sonar_ping_t) > (US_TIMEOUT_US // 1000) + 1:
            sonar_waiting = False                      # no echo in time
            _sonar_push(999.0)
    else:
        # wait until echo line is low again (module idle) before the next ping
        if ticks_diff(now, sonar_ping_t) >= SONAR_PERIOD_MS and echo.value() == 0:
            _sonar_ping()


def sonar_reset():
    global obs_hit, obs_latched, us_dist_cm
    sonar_hist[0] = sonar_hist[1] = sonar_hist[2] = 999.0
    us_dist_cm = 999.0
    obs_hit = 0
    obs_latched = False


# ======================================================================
# Timed manoeuvres (used by avoidance and calibration)
# ======================================================================
def _spin_timed(deg, left):
    ms = int(abs(deg) * ANGLE_FACTOR / TURN_RATE_DEG_S * 1000)
    if ms > SPIN_TIMEOUT_MS:
        ms = SPIN_TIMEOUT_MS
    if left:
        drive(-TURN_PWM, +TURN_PWM)
    else:
        drive(+TURN_PWM, -TURN_PWM)
    sleep_ms(ms)
    stop()


def _forward_timed(cm, until_line=False):
    """Drive forward for the time that corresponds to cm. Returns True if the
    middle sensor hit the line first (only when until_line)."""
    ms = int(cm * DIST_FACTOR / FWD_SPEED_CM_S * 1000)
    t0 = ticks_ms()
    drive(FWD_PWM, FWD_PWM)
    hit = False
    while ticks_diff(ticks_ms(), t0) < ms:
        if until_line and sensors[2].value() == 0:     # middle sensor on line
            hit = True
            break
        sleep_ms(2)
    stop()
    return hit


def avoid_obstacle():
    stop()
    if not CALIBRATED:
        print("OBSTACLE: avoidance not calibrated (CALIBRATED=False) - holding.")
        while obs_latched:
            sonar_update()
            sleep_ms(5)
        return

    # same 7 steps as original, timed instead of encoder-based
    _spin_timed(70.0, True)
    stop(); sleep_ms(STEP_PAUSE_MS)

    _forward_timed(AVOID_LEG1_CM)
    stop(); sleep_ms(STEP_PAUSE_MS)

    _spin_timed(60.0, False)
    stop(); sleep_ms(STEP_PAUSE_MS)

    _forward_timed(AVOID_LEG2_CM)
    stop(); sleep_ms(STEP_PAUSE_MS)

    _spin_timed(60.0, False)
    stop(); sleep_ms(STEP_PAUSE_MS)

    _forward_timed(AVOID_RETURN_MAX_CM, until_line=True)
    stop(); sleep_ms(STEP_PAUSE_MS)

    _spin_timed(30.0, True)
    stop(); sleep_ms(STEP_PAUSE_MS)

    on = read_sensors()
    n = sum(1 for x in on if x)
    if n == 0 or n == 5:                               
        _spin_timed(20.0, False)
        stop(); sleep_ms(STEP_PAUSE_MS)


# ======================================================================
# Setup
# ======================================================================
def setup():
    global a_in1, a_in2, b_in1, b_in2, a_pwm, b_pwm, sensors, trig, echo
    a_in1 = Pin(AIN1_PIN, Pin.OUT, value=0)
    a_in2 = Pin(AIN2_PIN, Pin.OUT, value=0)
    b_in1 = Pin(BIN1_PIN, Pin.OUT, value=0)
    b_in2 = Pin(BIN2_PIN, Pin.OUT, value=0)
    a_pwm = PWM(Pin(PWMA_PIN), freq=PWM_FREQ, duty_u16=0)
    b_pwm = PWM(Pin(PWMB_PIN), freq=PWM_FREQ, duty_u16=0)
    sensors = [Pin(p, Pin.IN) for p in SENSOR_PINS]    # no pull, like INPUT
    trig = Pin(TRIG_PIN, Pin.OUT, value=0)
    echo = Pin(ECHO_PIN, Pin.IN)
    echo.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=echo_isr)
    stop()
    sonar_reset()


# ======================================================================
# Autonomous loop
# ======================================================================
def autonomous():
    last_seen = NONE
    seen_line_ever = False
    lost_t0 = None                       # when the current line loss began
    bad_t = ticks_add(ticks_ms(), -(BAD_HOLD_MS + 1))   
    t_prev = ticks_ms()
    next_t = ticks_ms()
    last_err = 0.0
    integ = 0.0
    d_filt = 0.0
    pid_fresh = True

    while True:
        now = ticks_ms()
        on = read_sensors()
        n, err = line_error(on)
        sonar_update()

        if n > 0:
            seen_line_ever = True
        valid = 1 <= n <= 4

        # --- reject all-off / all-on samples for too long -> stop  ---
        if valid:
            bad_t = now
        elif ticks_diff(now, bad_t) > BAD_HOLD_MS:
            stop()
            pid_fresh = True
            lost_t0 = None
            t_prev = now
            sleep_ms(LOOP_MS)
            continue

        # --- obstacle (only while following a valid line) ---
        if valid and obs_latched:
            avoid_obstacle()
            sonar_reset()                # drop stale readings from before the manoeuvre
            pid_fresh = True
            lost_t0 = None
            t_prev = ticks_ms()
            next_t = t_prev
            continue

        if n == 0:
            # ---- line lost ----
            pid_fresh = True
            if not seen_line_ever:
                stop()
            else:
                if lost_t0 is None:
                    lost_t0 = now
                if ticks_diff(now, lost_t0) >= RECOV_TIME_MS:
                    stop()               # give up spinning; waits here until a line appears
                elif last_seen == LEFT:
                    drive(-RECOVERY_PWM, +RECOVERY_PWM)
                elif last_seen == RIGHT:
                    drive(+RECOVERY_PWM, -RECOVERY_PWM)
                elif ticks_diff(now, lost_t0) < LOST_STRAIGHT_MS:
                    drive(BASE_PWM, BASE_PWM)
                else:
                    stop()               
            t_prev = now
        else:
            # ---- line visible: PD(I) steering ----
            lost_t0 = None
            dt = ticks_diff(now, t_prev) / 1000.0
            if dt < 0.001:
                dt = 0.001
            t_prev = now

            if pid_fresh:
                last_err = err
                integ = 0.0
                d_filt = 0.0
                pid_fresh = False

            integ += err * dt
            if KI > 0:
                lim = I_LIMIT / KI
                if integ > lim:
                    integ = lim
                elif integ < -lim:
                    integ = -lim
            d_raw = (err - last_err) / dt
            d_filt = D_SMOOTH * d_filt + (1.0 - D_SMOOTH) * d_raw
            last_err = err

            u = KP * err + KI * integ + KD * d_filt

            speed = BASE_PWM * (JUNCTION_FACTOR if n >= 3 else 1.0)
            left = speed + u             # line on the right (err>0) -> left faster
            right = speed - u
            if not ALLOW_REVERSE:
                if left < 0:
                    left = 0
                if right < 0:
                    right = 0
            drive(left, right)

            if err < -0.25:
                last_seen = LEFT
            elif err > 0.25:
                last_seen = RIGHT
            else:
                last_seen = NONE

        # --- fixed loop period ---
        next_t = ticks_add(next_t, LOOP_MS)
        wait = ticks_diff(next_t, ticks_ms())
        if wait > 0:
            sleep_ms(wait)
        else:
            next_t = ticks_ms()


# ======================================================================
# Test / calibration modes
# ======================================================================
def run_test():
    if TEST_MODE == 'sensors':
        print("Order shown: OUT1(34) OUT2(32) OUT3(33) OUT4(25) OUT5(27); 1 = on line")
        while True:
            on = read_sensors()
            n, err = line_error(on)
            print(''.join('1' if x else '0' for x in on), 'n=%d err=%.2f' % (n, err))
            sleep_ms(200)

    elif TEST_MODE == 'motors':
        for name, pins in (('LEFT (A)', (a_in1, a_in2, a_pwm, L_SCALE)),
                           ('RIGHT (B)', (b_in1, b_in2, b_pwm, R_SCALE))):
            for v in (+100, -100):
                print(name, 'FORWARD' if v > 0 else 'BACKWARD')
                _set_motor(pins[0], pins[1], pins[2], v, pins[3])
                sleep_ms(1500)
                stop()
                sleep_ms(700)
        print('motor test done')

    elif TEST_MODE == 'sonar':
        t = ticks_ms()
        while True:
            sonar_update()
            if ticks_diff(ticks_ms(), t) >= 200:
                t = ticks_ms()
                print('last readings', sonar_hist, 'median %.1f cm' % us_dist_cm,
                      'OBSTACLE' if obs_latched else '')
            sleep_ms(2)

    elif TEST_MODE == 'turn':
        print('spinning LEFT 1 s at TURN_PWM')
        drive(-TURN_PWM, +TURN_PWM)
        sleep_ms(1000)
        stop()
        sleep_ms(3000)
        print('spinning RIGHT 1 s at TURN_PWM')
        drive(+TURN_PWM, -TURN_PWM)
        sleep_ms(1000)
        stop()
        print('measure the angles -> TURN_RATE_DEG_S (deg per second)')

    elif TEST_MODE == 'forward':
        print('driving forward 1 s at FWD_PWM')
        drive(FWD_PWM, FWD_PWM)
        sleep_ms(1000)
        stop()
        print('measure the distance -> FWD_SPEED_CM_S (cm per second)')

    else:
        print('unknown TEST_MODE', TEST_MODE)


# ======================================================================
# Entry point
# ======================================================================
def main():
    setup()
    if START_DELAY_MS > 0:
        sleep_ms(START_DELAY_MS)
    try:
        if TEST_MODE is None:
            autonomous()
        else:
            run_test()
    except KeyboardInterrupt:
        pass
    finally:
        stop()           # never leave motors driven after Ctrl-C or an error


main()
