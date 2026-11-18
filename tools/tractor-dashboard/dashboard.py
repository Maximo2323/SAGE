#!/usr/bin/env python3
"""Local STM32 dashboard. Python standard library only; serial supports macOS/Linux."""
import argparse
import ctypes
import functools
import hashlib
import json
import math
import os
from pathlib import Path
import re
import secrets
import select
import platform
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

PATTERN = re.compile(r'tgt (-?\d+) rpm (-?\d+) (-?\d+) cmd (-?\d+) (-?\d+) steer (-?\d+) g (-?\d+) r (-?\d+) rst (\d+) ov (\d+)((?: (?:STALL|IMU-FAIL|IMU-STALE))*)')
FIELDS = 'target rpm_left rpm_right cmd_left cmd_right steer game_yaw rotation_yaw resets overruns'.split()


def parse_line(line):
    match = PATTERN.fullmatch(line.strip())
    if not match:
        return None
    result = dict(zip(FIELDS, map(int, match.groups()[:10])))
    result['flags'] = match[11].split()
    return result


class Serial:
    def __init__(self, path):
        import termios
        import tty
        import fcntl
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            tty.setraw(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[4] = attrs[5] = termios.B115200
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except Exception:
            os.close(self.fd)
            raise
        self.buffer = b''

    def write(self, text):
        data = text.encode('ascii')
        if os.write(self.fd, data) != len(data):
            raise OSError('Incomplete serial write')

    def lines(self):
        if not select.select([self.fd], [], [], 0)[0]:
            return []
        chunk = os.read(self.fd, 4096)
        if not chunk:
            raise OSError('Serial device disconnected')
        self.buffer += chunk
        lines = self.buffer.split(b'\n')
        self.buffer = lines.pop()
        if len(self.buffer) > 4096:
            self.buffer = b''
        return [line.decode('ascii', errors='replace') for line in lines]


FIRMWARE = Path(__file__).resolve().parents[2] / 'John_Deere_Tractor'
CORE_SOURCES = [FIRMWARE / 'App/Control' / n for n in ('control.c', 'pid.c', 'math_utils.c')]
CORE_FILES = CORE_SOURCES + [FIRMWARE / 'App/Control' / n for n in ('control.h', 'pid.h', 'math_utils.h')] + [
    FIRMWARE / 'App/Config/constants.h', Path(__file__).with_name('control_binding.c')]
BUILD = Path(__file__).with_name('build')


class ControlLib:
    """The firmware control core (control.c) compiled for this PC with the same constants.h."""

    def __init__(self, path):
        lib = self.lib = ctypes.CDLL(str(path))
        lib.control_size.restype = ctypes.c_size_t
        lib.control_init.argtypes = [ctypes.c_void_p, ctypes.c_bool, ctypes.c_uint32]
        lib.control_key.argtypes = [ctypes.c_void_p, ctypes.c_char, ctypes.c_uint32]
        lib.control_tick.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        lib.control_wheels.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint16, ctypes.c_uint16, ctypes.c_uint32]
        lib.sim_steering.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_int]
        lib.control_view.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double)]
        for name in ('wrap180', 'wrap360'):
            getattr(lib, name).argtypes, getattr(lib, name).restype = [ctypes.c_float], ctypes.c_float
        lib.deadband_map.argtypes, lib.deadband_map.restype = [ctypes.c_float] * 3, ctypes.c_float
        names = ctypes.cast(lib.config_names, ctypes.c_char_p).value.decode().split()
        values = ctypes.cast(lib.config_values, ctypes.POINTER(ctypes.c_double))
        self.config = {n: values[i] for i, n in enumerate(names)}
        self.view_names = ctypes.cast(lib.view_names, ctypes.c_char_p).value.decode().split()
        self.size = lib.control_size()

    def __getattr__(self, name):
        return getattr(self.lib, name)

    def view(self, ctl):
        out = (ctypes.c_double * len(self.view_names))()
        self.lib.control_view(ctl, out)
        return dict(zip(self.view_names, out))

    @staticmethod
    def build():
        """Compile the core into build/control-<hash>.so; reuse it when the sources did not change.
        Raises RuntimeError with the compiler message when the code (e.g. constants.h) does not compile."""
        digest = hashlib.sha256(b''.join(f.read_bytes() for f in CORE_FILES)).hexdigest()[:12]
        out = BUILD / f'control-{sys.platform}-{platform.machine()}-{digest}.so'  # one build per OS and CPU
        if not out.exists():
            BUILD.mkdir(exist_ok=True)
            cmd = [os.environ.get('CC', 'cc'), '-std=c11', '-O2', '-shared', '-fPIC',
                   f'-I{FIRMWARE}/App/Control', f'-I{FIRMWARE}/App/Config', *map(str, CORE_SOURCES),
                   str(CORE_FILES[-1]), '-lm', '-o', str(out) + '.tmp']
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError('Control core does not compile: ' + (result.stderr.strip().splitlines() or ['?'])[0])
            os.replace(str(out) + '.tmp', out)
            for old in BUILD.glob('control-*.so'):
                if old != out:
                    try:
                        old.unlink()
                    except OSError:
                        pass  # an old build left in place is harmless
        return out


@functools.lru_cache(maxsize=1)
def compiled_config(mtime):
    """constants.h of this PC, as compiled (the cache key is the newest source time)."""
    try:
        return ControlLib(ControlLib.build()).config, ''
    except (RuntimeError, OSError) as exc:
        return None, str(exc)


WORLD_FILE = Path(__file__).with_name('track.json')
OBJECT_FIELDS = {  # type: numeric fields (mm, m for h, deg for yaw); 'label' and 'color' are optional text
    'wall': ('h',), 'box': ('x', 'y', 'w', 'd', 'h'), 'circle': ('x', 'y', 'r', 'h'),
    'checkpoint': ('x', 'y'), 'start': ('x', 'y', 'yaw')}
OPTIONAL_FIELDS = {'checkpoint': {'yaw': 90.0}}  # older track files have checkpoints without a heading
PX_PER_M = 23.8  # satellite measurements of the real track: 10 m ticks are 238 px apart. Origin: outer bottom-left corner.


def _map(points):
    """Outer screenshot pixels -> mm. Outer lengths 46.3 / 45.6 / 24.6 / 14.7 / 17.6 / 22.9 m (171.7 m; map: 171.79 m)."""
    return [[round((x - 219) * 1000 / PX_PER_M), round((1178 - y) * 1000 / PX_PER_M)] for x, y in points]


def _inner(points):
    """Inner screenshot pixels: same zoom, lined up on the monument building (37 px left, 44 px down).
    Lengths 31.1 / 32.7 / 7.5 / 11.1 / 9.4 / 13.8 / 10.6 m (116.2 m; map: 116.25 m)."""
    return _map([(x - 37, y + 44) for x, y in points])


GATE = (893, 991)  # ESTIMATED from the drawing: the upper 45 % of the steep outer diagonal is the gray segment
DEFAULT_WORLD = {'objects': [
    {'type': 'wall', 'pts': _map([(765, 1183), (219, 1178), (211, 77), (1297, 77), (1302, 663), (997, 834), GATE]), 'h': 0.2,
     'color': '#3fb950', 'label': 'outer'},
    {'type': 'wall', 'pts': _map([GATE, (765, 1183)]), 'h': 0.2, 'color': '#3fb950', 'label': 'outer'},
    {'type': 'wall', 'pts': _map([(997, 834), GATE]), 'h': 0.0, 'color': '#555c66', 'label': 'gate'},
    {'type': 'wall', 'pts': _inner([(384, 917), (405, 176), (1183, 176), (1180, 355), (970, 517), (774, 624), (636, 922), (384, 917)]),
     'h': 0.01, 'color': '#3fb950', 'label': 'inner'},  # 1 cm: below every sensor, drawn solid for reference
    # ESTIMATED from the drawing: red line, X and start area. Drag them to their real places.
    {'type': 'wall', 'pts': [[8700, 43300], [34200, 43300]], 'h': 0.2, 'color': '#e5534b'},
    {'type': 'box', **dict(zip('xy', _map([(990, 940)])[0])), 'w': 2000, 'd': 2000, 'h': 0.0, 'label': 'start area'},
    {'type': 'checkpoint', **dict(zip('xy', _map([(867, 751)])[0])), 'yaw': 235.0, 'label': 'X'},
    {'type': 'start', **dict(zip('xy', _map([(1032, 915)])[0])), 'yaw': 135.0}]}


def validate_world(world):
    """The track file and POST /world: known object types, finite numbers, short text only."""
    if not isinstance(world, dict) or not isinstance(world.get('objects'), list) or len(world['objects']) > 500:
        raise ValueError('Invalid track')
    clean = []
    for o in world['objects']:
        if not isinstance(o, dict) or o.get('type') not in OBJECT_FIELDS:
            raise ValueError('Invalid track object')
        out = {'type': o['type']}
        for k in OBJECT_FIELDS[o['type']] + tuple(OPTIONAL_FIELDS.get(o['type'], ())):
            v = o.get(k, OPTIONAL_FIELDS.get(o['type'], {}).get(k))
            if type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 1e6:
                raise ValueError(f'Invalid {o["type"]} {k}')
            out[k] = v
        if o['type'] == 'wall':
            pts = o.get('pts')
            if (not isinstance(pts, list) or not 2 <= len(pts) <= 200 or not all(
                    isinstance(p, list) and len(p) == 2 and all(type(v) in (int, float) and math.isfinite(v) and abs(v) < 1e6 for v in p)
                    for p in pts)):
                raise ValueError('Invalid wall points')
            out['pts'] = pts
        for k in ('label', 'color'):
            if k in o:
                if not isinstance(o[k], str) or len(o[k]) > 24:
                    raise ValueError(f'Invalid {k}')
                out[k] = o[k]
        if out.get('h', 0) < 0 or out.get('w', 1) <= 0 or out.get('d', 1) <= 0 or out.get('r', 1) <= 0:
            raise ValueError('Sizes must be positive')
        clean.append(out)
    if sum(o['type'] == 'start' for o in clean) > 1:
        raise ValueError('Only one start')
    return {'objects': clean}


def load_world(path=None):
    path = path or WORLD_FILE
    try:
        return validate_world(json.loads(Path(path).read_text()))
    except FileNotFoundError:
        return json.loads(json.dumps(DEFAULT_WORLD))


def save_world(world, path=None):
    world = validate_world(world)
    path = Path(path or WORLD_FILE)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(world, indent=1))
    os.replace(tmp, path)  # never a half-written track file
    return world


def segments(world, min_height_m):
    """Outline segments of every object at least min_height_m high."""
    for o in world['objects']:
        if o['type'] in ('checkpoint', 'start') or o['h'] < min_height_m:
            continue
        if o['type'] == 'wall':
            yield from zip(o['pts'], o['pts'][1:])
        elif o['type'] == 'box':
            x, y, w, d = o['x'], o['y'], o['w'], o['d']
            c = [(x, y), (x + w, y), (x + w, y + d), (x, y + d)]
            yield from zip(c, c[1:] + c[:1])


def ray_distance(world, ox, oy, angle_deg, min_height_m, max_mm):
    """Distance in mm to the first object at least min_height_m high, or None. 2D only: no beam width."""
    dx, dy = math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))
    best = max_mm
    hit = False
    for (ax, ay), (bx, by) in segments(world, min_height_m):
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((ax - ox) * ey - (ay - oy) * ex) / den
        u = ((ax - ox) * dy - (ay - oy) * dx) / den
        if 0 <= t <= best and 0 <= u <= 1:
            best, hit = t, True
    for o in world['objects']:
        if o['type'] == 'circle' and o['h'] >= min_height_m:
            fx, fy = ox - o['x'], oy - o['y']
            b = fx * dx + fy * dy
            disc = b * b - (fx * fx + fy * fy - o['r'] ** 2)
            if disc >= 0:
                for t in (-b - math.sqrt(disc), -b + math.sqrt(disc)):
                    if 0 <= t <= best:
                        best, hit = t, True
                        break
    return best if hit else None


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def sources_mtime():
    return max(f.stat().st_mtime for f in CORE_FILES)


class SimCar:
    """Simulated sensors, servo, motors and vehicle around the firmware control core.
    Every control decision is in C (control.c). This class has only the plant and the fake sensors,
    and it calls the core in the order of the main loop of main.c, one loop each simulated ms.
    Plant values marked SIM_ in constants.h are assumptions, not measurements: the sim does not
    predict the stability of the real tractor."""

    def __init__(self, lib=None, world=None):
        self.fixed_lib = lib is not None
        self.world = world or {'objects': []}
        self.lib, self.mtime, self.error = lib, None, ''
        self.now = 0                    # ms, simulated time
        self.leftover = 0.
        self.yaw = self.x = self.y = 0.  # true vehicle state: deg (counterclockwise), mm
        self.steer_actual = 0.           # simulated wheel angle, deg; the command is in the controller
        self.rpm = [0., 0.]              # true wheel speeds
        self.wheel_pos = [0., 0.]        # wheel revolutions, for the encoder counters
        self.seq = 0
        self.keys = []
        # Fake BNO085 (Game Rotation Vector): raw heading -180..180, report count, handling time
        self.imu_raw = self.imu_first = self.imu_raw_next = 0.
        self.imu_count = self.imu_resets = self.imu_t = self.imu_pending = 0
        self.imu_reset_zero = 0
        start = next((o for o in self.world['objects'] if o['type'] == 'start'), None)
        if start:  # before boot(): the heading zero is the start heading, as after a calibration there
            self.x, self.y, self.yaw = start['x'], start['y'], start['yaw']
            self.imu_raw = self.imu_raw_next = self.imu_first = ((self.yaw + 180) % 360) - 180
        self.load()

    def load(self):
        """Build the core when a source file changed. A new build = a new firmware flash: the controller
        restarts from its boot state; the simulated vehicle keeps its pose and its wheel speeds."""
        if self.fixed_lib:
            if self.mtime is None:
                self.mtime = 0
                self.boot()
            return
        mtime = sources_mtime()
        if mtime == self.mtime:
            return
        self.mtime = mtime
        try:
            self.lib = ControlLib(ControlLib.build())
        except (RuntimeError, OSError) as exc:
            self.error = str(exc)       # the old core keeps running; the dashboard shows the error
            if self.lib is None:
                raise
            return
        self.error = ''
        self.boot()
        print('sim: control core built from constants.h; controller restarted', flush=True)

    def boot(self):
        """Power-on of the firmware: the heading zero is the present heading (calibration, without the 3 s)."""
        self.c = self.lib.config
        self.ctl = ctypes.create_string_buffer(self.lib.size)
        self.zero = self.lib.wrap360(self.imu_raw)
        self.imu_reset_zero = self.imu_resets
        self.lib.control_init(self.ctl, True, self.now)
        self.t_steer = self.t_imu_read = self.now
        self.next_sample = self.now + int(self.c['WHEEL_CONTROL_PERIOD_MS'])
        self.keys = []

    @property
    def wheelbase(self):
        return self.c['WHEELBASE_MM'] if self.c['WHEELBASE_MM'] > 0 else self.c['SIM_WHEELBASE_MM']

    @property
    def track(self):
        return self.c['TRACK_WIDTH_MM'] if self.c['TRACK_WIDTH_MM'] > 0 else self.c['SIM_TRACK_WIDTH_MM']

    @property
    def geometry(self):
        return [self.c['WHEEL_DIAMETER_MM'], self.wheelbase, self.track]

    @property
    def placeholders(self):
        return [k for k in ('WHEELBASE_MM', 'TRACK_WIDTH_MM') if self.c[k] <= 0]

    def view(self):
        return self.lib.view(self.ctl)

    def imu_valid(self, now):
        """bno085_fresh() and the reset check of heading_hold_sample()."""
        age = ((now - self.imu_t + 2**31) % 2**32) - 2**31
        return self.imu_count != 0 and age < self.c['BNO_STALE_TIMEOUT_MS'] and self.imu_resets == self.imu_reset_zero

    def steering(self, now):
        self.lib.sim_steering(self.ctl, self.imu_raw, self.zero, self.imu_count % 2**32, self.imu_t % 2**32,
                              int(self.imu_valid(now)))

    def counter(self, side):
        sign = self.c['ENC_DIR_LEFT' if side == 0 else 'ENC_DIR_RIGHT']
        return int(math.floor(sign * self.wheel_pos[side] * self.c['COUNTS_PER_WHEEL_REV'])) & 0xFFFF

    def loop(self):
        """One main-loop pass of main.c at self.now, then 1 ms of plant."""
        lib, ctl, now, c = self.lib, self.ctl, self.now % 2**32, self.c
        if self.keys:                    # the firmware reads one key per loop
            lib.control_key(ctl, self.keys.pop(0).encode(), now)
        lib.control_tick(ctl, now)
        if self.now >= self.next_sample:  # TIM6 encoder sample
            self.next_sample += int(c['WHEEL_CONTROL_PERIOD_MS'])
            self.seq += 1
            lib.control_wheels(ctl, self.seq % 2**32, self.counter(0), self.counter(1), now)
        period = int(c['BNO_GAME_RV_INTERVAL_US'] // 1000)
        if self.now >= c['SIM_IMU_PHASE_MS'] and (self.now - int(c['SIM_IMU_PHASE_MS'])) % period == 0:
            self.imu_pending += 1        # the sensor made a report; it is read at the next I2C read
            self.imu_raw_next = float(lib.wrap180(self.yaw))
        if self.now - self.t_imu_read >= c['BNO_READ_PERIOD_MS']:
            self.t_imu_read = self.now
            if self.imu_pending:
                if self.imu_count == 0:
                    self.imu_first = self.imu_raw_next
                self.imu_raw, self.imu_t = self.imu_raw_next, self.now
                self.imu_count += self.imu_pending
                self.imu_pending = 0
        if self.now - self.t_steer >= c['STEERING_CONTROL_PERIOD_MS']:
            self.t_steer = self.now
            self.steering(now)
        self.plant(self.view(), .001)
        self.now += 1

    def plant(self, v, dt):
        """Assumed plant. Servo: first-order lag, then a rate limit (SIM_SERVO_*). Motors: first-order
        (SIM_MOTOR_*). Vehicle: kinematic bicycle, no slip. Positive steering = left."""
        c = self.c
        command = clamp(v['steer_deg'], -c['SERVO_MAX_STEER_DEG'], c['SERVO_MAX_STEER_DEG'])  # servo_set_deg()
        change = (command - self.steer_actual) * min(1., dt / c['SIM_SERVO_TAU_S']) if c['SIM_SERVO_TAU_S'] > 0 else command - self.steer_actual
        rate = c['SIM_SERVO_RATE_DEG_S'] * dt
        self.steer_actual += clamp(change, -rate, rate)
        for side, cmd in enumerate((v['cmd_l'], v['cmd_r'])):
            duty = self.lib.deadband_map(cmd, c['MOTOR_DEADBAND_PCT'], c['MOTOR_DUTY_MAX']) if v['motor_enable'] else 0.
            speed = duty / 100 * c['MOTOR_NO_LOAD_RPM_12V'] * c['SIM_MOTOR_GAIN']
            self.rpm[side] += (speed - self.rpm[side]) * min(1., dt / c['SIM_MOTOR_TAU_S'])
            self.wheel_pos[side] += self.rpm[side] / 60 * dt
        speed = sum(self.rpm) / 2 * math.pi * c['WHEEL_DIAMETER_MM'] / 60  # mm/s
        self.yaw += math.degrees(speed * math.tan(math.radians(self.steer_actual)) / self.wheelbase) * dt
        self.x += speed * math.cos(math.radians(self.yaw)) * dt
        self.y += speed * math.sin(math.radians(self.yaw)) * dt

    def advance(self, seconds):
        self.load()
        self.leftover += seconds * 1000
        steps = min(int(self.leftover), 1000)
        self.leftover = min(self.leftover - steps, 1.)
        for _ in range(steps):
            self.loop()

    def tof(self):
        """The three VL53L1X of distance.h: front, right low, right high. 2D ray, no beam width, no noise.
        ASSUMED mounts: front = front of the body; right = middle of the right side."""
        c, L = self.c, self.wheelbase
        mounts = (('front', 1.18 * L, 0., 0., c['SIM_TOF_FRONT_HEIGHT_MM']),
                  ('right_low', .5 * L, -.34 * self.track, -90., c['SIM_TOF_SIDE_LOW_HEIGHT_MM']),
                  ('right_high', .5 * L, -.34 * self.track, -90., c['SIM_TOF_SIDE_HIGH_HEIGHT_MM']))
        yaw, out = math.radians(self.yaw), []
        for name, fx, fy, angle, height in mounts:
            x = self.x + fx * math.cos(yaw) - fy * math.sin(yaw)
            y = self.y + fx * math.sin(yaw) + fy * math.cos(yaw)
            mm = ray_distance(self.world, x, y, self.yaw + angle, height / 1000, c['TOF_MAX_RANGE_MM'])
            out.append(dict(name=name, x=x, y=y, angle=self.yaw + angle, height_mm=height,
                            mm=-1 if mm is None else int(mm)))  # -1 = no target, as the firmware telemetry
        return out

    def telemetry(self):
        """The telemetry line of main.c, from the simulated state."""
        v = self.view()
        relative = int(self.lib.wrap180(self.imu_raw - self.imu_first))  # bno085_relative_yaw()
        data = dict(zip(FIELDS, [int(v['target_rpm']), int(v['rpm_l']), int(v['rpm_r']), int(v['cmd_l']),
                                 int(v['cmd_r']), int(v['steer_deg']), relative, relative, self.imu_resets, int(v['overruns'])]))
        data['flags'] = (['STALL'] if v['stalled'] else []) + ([] if self.imu_valid(self.now) else ['IMU-STALE'])
        data['steer_actual'] = round(self.steer_actual, 1)  # sim only: not in the firmware telemetry
        data['dist'] = [s['mm'] for s in self.tof()]       # front, right low, right high (firmware "dist" line)
        return data


class Robot:
    def __init__(self, serial=None, demo=False, geometry=None, placeholder_geometry=False, world=None):
        self.serial, self.demo, self.geometry = serial, demo, geometry
        self.placeholder_geometry = placeholder_geometry
        self.lock = threading.RLock()
        self.drive = self.turn = ''
        self.speed = None
        self.last_input = 0
        self.received = None
        self.data = None
        self.error = ''
        self.pose = [0., 0., 0.]
        self.trail = []
        self.trail_distance = 0.
        self.sim = SimCar(world=world) if demo else None
        if demo:
            self.pose = [self.sim.x, self.sim.y, math.radians(self.sim.yaw)]
        self.token = secrets.token_urlsafe(24)
        self.closed = False

    def send(self, keys):
        if self.serial and not self.demo:
            self.serial.write(keys)
        if self.demo:
            self.sim.keys.extend(keys)  # the same keys as the serial link, into control_key()

    def stop(self):
        self.send(' ')
        self.drive = self.turn = ''

    def move_car(self, x, y):
        """Sim only: put the car at x, y (mm). Heading, speeds and controller state stay."""
        with self.lock:
            self.sim.x, self.sim.y = x, y
            self.pose = [x, y, math.radians(self.sim.yaw)]
            self.trail, self.trail_distance = [], 0.

    def command(self, state):
        if (not isinstance(state, dict) or set(state) != {'drive', 'turn', 'speed', 'stop'}
                or state['drive'] not in ('', 'w', 's') or state['turn'] not in ('', 'a', 'd')
                or state['speed'] not in tuple('1234567890') or type(state['stop']) is not bool):
            raise ValueError('Invalid control state')
        with self.lock:
            if self.error:
                raise ValueError(self.error)
            self.last_input = time.monotonic()
            if state['stop']:
                self.stop()
                return
            if self.drive and state['drive'] != self.drive:
                self.stop()
            if self.speed != state['speed']:
                self.speed = state['speed']
                self.send(self.speed)
            if state['turn'] and state['turn'] != self.turn:
                self.send(state['turn'])
            self.turn = state['turn']
            self.drive = state['drive']
            if self.drive:
                self.send(self.drive)

    def ingest(self, data, now, pose=None):
        dt = 0 if self.received is None else now - self.received
        self.data, self.received = data, now
        if pose:  # sim: use the true sim pose, not an estimate from the rounded telemetry
            distance = math.dist(pose[:2], self.pose[:2])
            self.pose = pose
            self.trail_distance += distance
            if self.trail_distance >= 20:
                self.trail_distance = 0.
                self.trail.append(self.pose.copy())
                self.trail = self.trail[-400:]
        elif self.geometry and 0 < dt <= .3:
            diameter, wheelbase, track = self.geometry
            distance = (data['rpm_left'] + data['rpm_right']) / 2 * math.pi * diameter / 60 * dt
            delta = distance * math.tan(math.radians(data['steer'])) / wheelbase
            self.pose[0] += distance * math.cos(self.pose[2] + delta / 2)
            self.pose[1] += distance * math.sin(self.pose[2] + delta / 2)
            self.pose[2] += delta
            self.trail_distance += abs(distance)
            if self.trail_distance >= 20:  # one trail point for each 20 mm of travel
                self.trail_distance = 0.
                self.trail.append(self.pose.copy())
                self.trail = self.trail[-400:]

    def tick(self, now, dt):
        with self.lock:
            if self.drive and now - self.last_input > .2:
                self.stop()
            if self.demo:
                sim = self.sim
                sim.advance(dt)
                self.geometry = sim.geometry
                self.placeholder_geometry = bool(sim.placeholders)
                self.ingest(sim.telemetry(), now, [sim.x, sim.y, math.radians(sim.yaw)])
            elif self.serial:
                for line in self.serial.lines():
                    data = parse_line(line)
                    if data is not None:
                        self.ingest(data, now)

    def snapshot(self):
        with self.lock:
            age = None if self.received is None else time.monotonic() - self.received
            state = dict(mode='SIMULATION' if self.demo else 'SERIAL', data=self.data,
                         stale=age is None or age > .5, age=age, error=self.error,
                         geometry=self.geometry, placeholder_geometry=self.placeholder_geometry, pose=self.pose.copy(), trail=self.trail.copy())
            if self.demo:
                v = self.sim.view()
                state['config'] = self.sim.c
                state['sim'] = dict(error=self.sim.error, placeholders=self.sim.placeholders,
                                    steer_command=v['steer_deg'], steer_actual=self.sim.steer_actual,
                                    **{k: v[k] for k in ('heading_target', 'current_yaw', 'heading_error', 'heading_out')},
                                    heading_integral=v['pid_heading.integral'], heading_deriv=v['pid_heading.deriv'],
                                    tof=self.sim.tof())
            else:
                state['config'], state['config_error'] = compiled_config(sources_mtime())  # flashed firmware can differ
            return state

    def run(self):
        before = time.monotonic()
        while not self.closed:
            now = time.monotonic()
            try:
                self.tick(now, now-before)
            except OSError as exc:
                with self.lock:
                    self.error = str(exc)
                    self.drive = ''
                return
            before = now
            time.sleep(.1 if self.demo else .01)


class Dashboard:
    """One active mode; simulation never owns or writes the serial device."""
    def __init__(self, port=None, geometry=None, placeholder=False):
        self.port = port
        self.geometry = geometry
        self.placeholder = placeholder
        self.lock = threading.RLock()
        self.token = secrets.token_urlsafe(24)
        self.world = load_world()
        self.robot = Robot(demo=True, geometry=[100, 300, 220], world=self.world)
        self.generation = 0
        self.closed = False

    def switch(self, mode):
        if mode not in ('sim', 'live'):
            raise ValueError('Choose sim or live')
        with self.lock:
            if mode == 'live' and not self.port:
                raise ValueError('Start with --port /dev/cu.usbmodem… to enable Live')
            if self.robot.serial:
                try:
                    self.robot.stop()
                except OSError:
                    pass  # A lost cable cannot receive stop; firmware timeout remains.
                finally:
                    os.close(self.robot.serial.fd)
            self.generation += 1
            self.robot = Robot(demo=(mode == 'sim'),
                               geometry=[100, 300, 220] if mode == 'sim' else self.geometry,
                               placeholder_geometry=mode == 'live' and self.placeholder, world=self.world)
            if mode == 'live':
                try:
                    self.robot.serial = Serial(self.port)
                except OSError as exc:
                    self.robot.error = f'Cannot open {self.port}: {exc.strerror}. Close other serial apps, then click Live to retry.'

    def set_world(self, world):
        """Save the track (track.json) and give it to the running sim."""
        with self.lock:
            self.world = save_world(world)
            if self.robot.demo:
                self.robot.sim.world = self.world

    def move_car(self, payload):
        with self.lock:
            if not self.robot.demo:
                raise ValueError('The car can be moved in Sim only')
            if (not isinstance(payload, dict) or set(payload) != {'x', 'y'}
                    or not all(type(v) in (int, float) and math.isfinite(v) and abs(v) < 1e6 for v in payload.values())):
                raise ValueError('Invalid position')
            self.robot.move_car(payload['x'], payload['y'])

    def command(self, state):
        if not isinstance(state, dict):
            raise ValueError('Invalid control state')
        state = state.copy()
        generation = state.pop('generation', None)
        with self.lock:
            if type(generation) is not int or generation != self.generation:
                raise ValueError('Mode changed; reload controls')
            self.robot.command(state)

    def snapshot(self):
        with self.lock:
            return dict(self.robot.snapshot(), generation=self.generation,
                        live_available=bool(self.port))

    def run(self):
        before = time.monotonic()
        while not self.closed:
            now = time.monotonic()
            with self.lock:
                if not self.robot.error:
                    try:
                        self.robot.tick(now, min(now-before, .1))
                    except OSError as exc:
                        self.robot.error = str(exc)
                        self.robot.drive = ''
            before = now
            time.sleep(.1 if self.robot.demo else .01)

    def close(self):
        self.closed = True
        with self.lock:
            if self.robot.serial:
                try:
                    self.robot.stop()
                except OSError:
                    pass
                finally:
                    os.close(self.robot.serial.fd)
                    self.robot.serial = None


def handler(robot):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, data, kind='application/json'):
            body = data if isinstance(data, bytes) else json.dumps(data).encode()
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def valid_host(self):
            return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}')

        def do_GET(self):
            if not self.valid_host():
                self.reply(403, {'error': 'Local host required'})
                return
            if self.path == '/':
                page = Path(__file__).with_name('index.html').read_text().replace('__TOKEN__', robot.token)
                self.reply(200, page.encode(), 'text/html; charset=utf-8')
            elif self.path == '/state':
                self.reply(200, robot.snapshot())
            elif self.path == '/world':
                self.reply(200, robot.world)
            else:
                self.reply(404, {'error': 'Not found'})

        def do_POST(self):
            if not self.valid_host() or self.path not in ('/control', '/mode', '/world', '/pose') or self.headers.get('X-Control-Token') != robot.token:
                self.reply(403, {'error': 'Forbidden'})
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size < (262144 if self.path == '/world' else 512):
                    raise ValueError('Invalid body length')
                payload = json.loads(self.rfile.read(size))
                if self.path == '/mode':
                    if not isinstance(payload, dict) or set(payload) != {'mode'}:
                        raise ValueError('Invalid mode request')
                    robot.switch(payload['mode'])
                elif self.path == '/world':
                    robot.set_world(payload)
                elif self.path == '/pose':
                    robot.move_car(payload)
                else:
                    robot.command(payload)
                self.reply(200, {'ok': True})
            except (ValueError, OSError) as exc:
                self.reply(400, {'error': str(exc)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='Start in simulation, even when --port is supplied')
    parser.add_argument('--port', help='Explicit serial device, e.g. /dev/cu.usbmodem…')
    parser.add_argument('--http-port', type=int, default=8766)
    geometry_options = parser.add_mutually_exclusive_group()
    geometry_options.add_argument('--placeholder-geometry', action='store_true', help='Illustrative 100/300/220 mm geometry; not calibrated')
    geometry_options.add_argument('--geometry-mm', type=float, nargs=3, metavar=('DIAMETER', 'WHEELBASE', 'TRACK'), help='Measured dimensions; enables ESTIMATED odometry only')
    args = parser.parse_args()
    if args.geometry_mm and any(not math.isfinite(v) or v <= 0 for v in args.geometry_mm):
        parser.error('All dimensions must be finite and positive')
    robot = Dashboard(args.port, args.geometry_mm or ([100, 300, 220] if args.placeholder_geometry else None), args.placeholder_geometry)
    if args.port and not args.demo:
        robot.switch('live')
    server = HTTPServer(('127.0.0.1', args.http_port), handler(robot))
    server.timeout = .2
    thread = threading.Thread(target=robot.run, daemon=True)
    thread.start()
    print(f"http://127.0.0.1:{args.http_port} — {'SIMULATION (no robot)' if robot.robot.demo else args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()
        thread.join(timeout=1)
        server.server_close()


if __name__ == '__main__':
    main()
