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
        lib.sim_set_heading_goal.argtypes = [ctypes.c_void_p, ctypes.c_float]
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


ROUTE_FILE = Path(__file__).with_name('route.json')
SENSORS = ('front', 'right_low', 'right_high')  # the order of SimCar.tof()
STEP_FIELDS = {  # step type: numeric field -> (min, max)
    'drive': {'m': (-100, 100), 'speed': (10, 100)},
    'turn': {'deg': (-360, 360), 'speed': (10, 100)},
    'find': {'mm': (0, 4000), 'tol': (0, 4000), 'max': (.1, 100), 'speed': (10, 100), 'hold': (0, 4000), 'kp': (0, 1), 'after': (0, 100)},
    'checkpoint': {'s': (0, 60)},
    'align': {'m': (-100, 100), 'speed': (10, 100)}}
TOF_PERIOD_MS = 50  # TOF_INTER_MEASUREMENT_MS: a new ToF reading this often
ALIGN_PCT = 10      # speed of the back-and-forward align at the end of a find step
STEP_OPTIONAL = {'hold': 0, 'kp': .05, 'after': 0}  # route files from before wall following
HOLD_MAX_DEG = 20   # the most the wall-following P control turns away from the route heading
# Car body and ToF mounts, as fractions of the wheelbase L (forward from the rear axle) and the track T.
# ASSUMED: measure them on the car. index.html draws the same body.
FRONT_X = 1.18                  # front sensor, on the center line
SIDE_X, SIDE_Y = .5, .34        # the two right sensors: SIDE_Y T right of the center line
BODY_FRONT, BODY_REAR, HALF_W = 1.184, .183, .34  # body front / rear (behind the axle) / half width


def validate_route(steps):
    """route.json and POST /route: a list of known steps with numbers in range."""
    if not isinstance(steps, list) or len(steps) > 100:
        raise ValueError('Invalid route')
    clean = []
    for s in steps:
        if not isinstance(s, dict) or s.get('type') not in STEP_FIELDS:
            raise ValueError('Invalid route step')
        out = {'type': s['type']}
        for k, (lo, hi) in STEP_FIELDS[s['type']].items():
            v = s.get(k, STEP_OPTIONAL.get(k))
            if type(v) not in (int, float) or not lo <= v <= hi:  # NaN fails this too
                raise ValueError(f'Step {len(clean) + 1} {s["type"]}: {k} must be {lo} to {hi}')
            if k == 'speed' and v % 10:  # the firmware takes 10 % steps (keys 1..9, 0)
                raise ValueError(f'Step {len(clean) + 1} {s["type"]}: speed must be 10, 20 ... 100')
            out[k] = v
        if s['type'] == 'find':
            if s.get('sensor') not in SENSORS or s.get('until') not in ('seen', 'lost'):
                raise ValueError(f'Step {len(clean) + 1} find: invalid sensor or until')
            out['sensor'], out['until'] = s['sensor'], s['until']
            if out['hold'] and out['sensor'] == 'front':
                raise ValueError(f'Step {len(clean) + 1} find: hold works with a side sensor only')
        if s['type'] in ('checkpoint', 'align'):
            if not isinstance(s.get('label'), str) or len(s['label']) > 24:
                raise ValueError(f'Step {len(clean) + 1} {s["type"]}: invalid label')
            out['label'] = s['label']
        clean.append(out)
    return clean


def load_route(path=None):
    try:
        return validate_route(json.loads(Path(path or ROUTE_FILE).read_text()))
    except FileNotFoundError:
        return []


def save_route(steps, path=None):
    steps = validate_route(steps)
    path = Path(path or ROUTE_FILE)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(steps, indent=1))
    os.replace(tmp, path)
    return steps


def edge_to_checkpoint(sim, cp, find):
    """Metres from the edge where `find` stops to checkpoint cp, + = forward. Searches the line through the
    checkpoint along its heading, so it assumes the car comes to the checkpoint straight along that heading.
    None: no such edge within the max distance of the find step."""
    yaw, idx = math.radians(cp['yaw']), SENSORS.index(find['sensor'])
    want = find['until'] == 'seen'  # the state just after the edge, in the travel direction

    def inside(s):
        mm = sim.tof((cp['x'] + s * math.cos(yaw), cp['y'] + s * math.sin(yaw), cp['yaw']))[idx]['mm']
        return mm >= 0 and abs(mm - find['mm']) <= find['tol']
    limit, best = int(min(find['max'], 50) * 1000), None
    prev = inside(-limit)
    for s in range(-limit + 5, limit + 1, 5):  # 5 mm steps: about the align precision of a find step
        cur = inside(s)
        if cur == want and prev != want and (best is None or abs(s) < abs(best)):
            best = s
        prev = cur
    return None if best is None else -best / 1000


class Route:
    """Sim only: runs the route steps through the firmware interface: W/S, speed digits and Space as keys,
    and the heading goal (a firmware route runner would set it the same way). It decides from wheel distance,
    the controller's IMU heading and the ToF readings, never from the true pose. The true pose is used only
    to report the error at a checkpoint."""

    def __init__(self, steps, first=0, loop=False):
        self.steps, self.log, self.loop, self.lap = steps, [], loop and bool(steps), 1
        self.i, self.begun, self.running = first, False, True
        self.heading = self.t_start = None  # route heading goal, deg from the start heading
        self.dir = self.t_key = 0
        self.speed_key = self.edge = None
        self.offset = 0.  # deg added to the route heading by wall following
        self.avoid, self.avoid_off = None, 0.  # avoidance phase; deg it adds to the route heading

    def say(self, sim, text):
        self.log = (self.log + [f'{sim.now / 1000:6.1f} s  {text}'])[-100:]

    @staticmethod
    def odo(sim):
        """Distance, mm, + forward. The sim wheel turns: the encoders give the same to one count."""
        return sum(sim.wheel_pos) / 2 * math.pi * sim.c['WHEEL_DIAMETER_MM']

    def go(self, sim, direction, pct):
        key = str(int(pct) // 10 % 10)
        if key != self.speed_key:
            sim.keys.append(key)
            self.speed_key = key
        if direction != self.dir or sim.now - self.t_key >= 100:  # W/S again before COMMAND_TIMEOUT_MS
            sim.keys.append('w' if direction > 0 else 's')
            self.dir, self.t_key = direction, sim.now

    def finish(self, sim, text):
        self.say(sim, f'{self.i + 1} {self.steps[self.i]["type"]}: {text}')
        sim.keys.append(' ')  # Space: zero output now. It also sets the heading goal to the yaw; step() sets it again.
        self.dir, self.i, self.begun = 0, self.i + 1, False

    def fail(self, sim, text):
        self.finish(sim, 'FAILED, ' + text)
        self.running = False

    def report(self, sim, s):
        reads = ' / '.join(str(t['mm']) for t in sim.tof())
        cp = next((o for o in sim.world['objects'] if o['type'] == 'checkpoint' and o.get('label') == s['label']), None)
        if cp is None:
            return self.say(sim, f'{self.i + 1} checkpoint "{s["label"]}": not on the track. ToF F/RL/RH {reads} mm')
        expect = ' / '.join(str(t['mm']) for t in sim.tof((cp['x'], cp['y'], cp['yaw'])))
        self.say(sim, f'{self.i + 1} checkpoint {s["label"]}: off by {math.hypot(sim.x - cp["x"], sim.y - cp["y"]) / 10:.1f} cm, '
                      f'{sim.lib.wrap180(sim.yaw - cp["yaw"]):+.1f} deg. ToF F/RL/RH {reads} mm (at the checkpoint: {expect})')

    def step(self, sim):
        """Once per simulated ms, before the main loop reads its key."""
        if not self.running or sim.keys:
            return
        v = sim.view()
        if self.heading is None:
            self.heading, self.t_start, self.o0 = v['heading_goal'], sim.now, self.odo(sim)
        sim.lib.sim_set_heading_goal(sim.ctl, self.heading + self.offset + self.avoid_off)
        if not self.begun:
            if max(map(abs, sim.rpm)) > .5:  # each step starts with the car stopped
                return
            if self.i >= len(self.steps):
                if self.loop:  # again from step 1, from where the car is now
                    self.say(sim, f'lap {self.lap} done at {(sim.now - self.t_start) / 1000:.1f} s')
                    self.i, self.lap = 0, self.lap + 1
                    return
                self.running = False
                return self.say(sim, f'route done in {(sim.now - self.t_start) / 1000:.1f} s')
            s = self.steps[self.i]
            self.begun, self.s0, self.t0, self.offset = True, self.odo(sim), sim.now, 0.
            self.lat = self.fwd = 0.  # mm left of / along the line of this step, from wheel distance and IMU
            self.last, self.avoid, self.avoid_off = self.s0, None, 0.
            self.t_mm = sim.now - TOF_PERIOD_MS  # read the sensors at once
            if self.i == 0 or self.steps[self.i - 1]['type'] != 'find':
                self.edge = None  # Align uses the edge only right after its Find
            self.phase = 'arm' if s['type'] == 'find' and s['until'] == 'lost' else 'seek'
            if s['type'] == 'turn':
                self.heading = sim.lib.wrap180(self.heading + s['deg'])
            if s['type'] == 'checkpoint':
                self.report(sim, s)
        odo, angle = self.odo(sim), math.radians(sim.lib.wrap180(v['current_yaw'] - self.heading))
        self.lat += (odo - self.last) * math.sin(angle)
        self.fwd += (odo - self.last) * math.cos(angle)
        self.last = odo
        self.fresh = sim.now - self.t_mm >= TOF_PERIOD_MS  # a new ToF reading this ms
        if self.fresh:
            self.t_mm, self.mm = sim.now, [t['mm'] for t in sim.tof()]
        s = self.steps[self.i]
        if self.avoid or (self.dir > 0 and (s['type'] == 'drive' or s['type'] == 'find' and s['sensor'] != 'front')
                          and 0 <= self.mm[0] < sim.c['AVOID_STOP_MM']):
            return self.avoiding(sim, angle)
        moved = self.fwd if s['type'] in ('drive', 'find') else odo - self.s0  # drive/find: progress along the line
        if s['type'] == 'drive':
            if abs(moved) >= abs(s['m']) * 1000:
                return self.finish(sim, f'{moved / 1000:.2f} m')
            self.go(sim, 1 if s['m'] > 0 else -1, s['speed'])
        elif s['type'] == 'turn':
            if abs(sim.lib.wrap180(self.heading - v['current_yaw'])) < 2:
                return self.finish(sim, f'heading {v["current_yaw"]:.1f} deg after {moved / 1000:.2f} m')
            if abs(moved) > 20000:
                return self.fail(sim, 'no turn after 20 m (IMU?)')
            self.go(sim, 1, s['speed'])  # Ackermann: the car must roll to turn
        elif s['type'] == 'align':  # drive from the edge to the checkpoint, stop, then report the error
            moved = self.odo(sim) - (self.s0 if self.edge is None else self.edge)
            if self.phase == 'seek':
                left = abs(s['m']) * 1000 - abs(moved)
                if left > 0:  # the last 100 mm at ALIGN_PCT: less coasting after the stop
                    return self.go(sim, 1 if s['m'] > 0 else -1, s['speed'] if left > 100 else ALIGN_PCT)
                sim.keys.append(' ')
                self.dir, self.phase = 0, 'settle'
            elif max(map(abs, sim.rpm)) <= .5:
                self.report(sim, s)
                self.finish(sim, f'{moved / 1000:.2f} m from the edge')
        elif s['type'] == 'checkpoint':
            if sim.now - self.t0 >= s['s'] * 1000:
                self.finish(sim, f'waited {s["s"]} s')
        else:
            self.find(sim, s, moved, angle)

    def avoiding(self, sim, angle):
        """Obstacle ahead. Stop and wait AVOID_WAIT_S: gone = go on. Still there = go around on the left.
        All positions in the frame of the step line: fwd along it, off to its left, from wheel distance and IMU.
        reverse: straight back until the car front is AVOID_BACK_MM from the obstacle face (an Ackermann car
                 needs room to move sideways).
        out:  swerve +AVOID_SWERVE_DEG. Each front hit is a point of the obstacle: lateral = off + r sin(angle),
              forward = fwd + r cos(angle). Keep the leftmost (edge) and nearest (face). Go on until the front is
              clear and off >= edge + half car + margin. Right front corner within margin of the face = no room: fail.
        pass: parallel, until the right sensors lose it + rear overhang (sensor to car rear) + margin.
        back: swerve -AVOID_SWERVE_DEG to the line. straight: on until the heading is within 3 deg of the line.
        New obstacle in pass or back: stop, wait; still there = new face, reverse again; the edge keeps the
        leftmost point of both, the return is to the first line."""
        c = sim.c
        front, sample, L, W, a = self.mm[0], self.fresh, sim.wheelbase, HALF_W * sim.track, angle
        r = FRONT_X * L + front  # rear axle to the hit point, along the car axis
        if self.avoid is None:
            self.avoid, self.av_t, self.av_lat0, self.offset = 'wait', sim.now, self.lat, 0.
            self.av_face, self.av_edge = self.fwd + r * math.cos(a), 0.
            sim.keys.append(' ')
            self.dir = 0
            return self.say(sim, f'{self.i + 1} obstacle at {front} mm: stop and wait')
        off = self.lat - self.av_lat0  # mm left of the line where the obstacle was met
        corner_fwd = self.fwd + BODY_FRONT * L * math.cos(a) + W * math.sin(a)   # right front corner of the body
        corner_off = off + BODY_FRONT * L * math.sin(a) - W * math.cos(a)
        gap = self.av_face - corner_fwd
        if self.avoid == 'wait':
            if sample and not 0 <= front < c['AVOID_STOP_MM'] + 100:
                self.avoid = None
                return self.say(sim, f'{self.i + 1} obstacle gone')
            if sim.now - self.av_t >= c['AVOID_WAIT_S'] * 1000:
                self.avoid = 'reverse'
                self.say(sim, f'{self.i + 1} obstacle still there: back up, then go around on the left')
            return
        if self.avoid == 'reverse':
            if gap < c['AVOID_BACK_MM']:
                return self.go(sim, -1, c['AVOID_PCT'])
            self.avoid = 'out'
        if self.avoid == 'out':
            self.avoid_off = c['AVOID_SWERVE_DEG']
            hit = 0 <= front <= c['AVOID_LOOK_MM']
            if sample and hit:
                self.av_edge = max(self.av_edge, off + r * math.sin(a))
                self.av_face = min(self.av_face, self.fwd + r * math.cos(a))
            target = self.av_edge + W + c['AVOID_MARGIN_MM']
            if target > c['AVOID_MAX_OFFSET_MM']:
                return self.fail(sim, f'obstacle needs more than {c["AVOID_MAX_OFFSET_MM"] / 1000:g} m to the left')
            if not hit and off >= target:
                self.avoid, self.avoid_off, self.av_mark, self.av_seen = 'pass', 0., self.fwd, False
                return self.say(sim, f'{self.i + 1} {off:.0f} mm left of the line: passing')
            if gap < c['AVOID_MARGIN_MM'] and corner_off < self.av_edge + c['AVOID_MARGIN_MM']:
                return self.fail(sim, f'no room to go around: edge {self.av_edge:.0f} mm left of the line')
            return self.go(sim, 1, c['AVOID_PCT'])
        if self.avoid in ('pass', 'back', 'straight') and sample and 0 <= front < c['AVOID_STOP_MM']:
            self.av_resume, self.av_t, self.avoid = self.avoid, sim.now, 'wait2'
            sim.keys.append(' ')
            self.dir = 0
            return self.say(sim, f'{self.i + 1} new obstacle at {front} mm during the detour: stop and wait')
        if self.avoid == 'wait2':
            if sample and not 0 <= front < c['AVOID_STOP_MM'] + 100:
                self.avoid = self.av_resume
                return self.say(sim, f'{self.i + 1} it is gone: continue the detour')
            if sim.now - self.av_t >= c['AVOID_WAIT_S'] * 1000:  # part of the same obstacle: same detour again
                self.av_face, self.avoid = self.fwd + r * math.cos(a), 'reverse'
                self.say(sim, f'{self.i + 1} still there: back up and go around both')
            return
        if self.avoid == 'pass':
            if self.fwd + SIDE_X * L < self.av_face:  # side sensors not beside it yet
                self.av_mark = self.fwd
            if any(0 <= m <= off for m in self.mm[1:]):  # right sensors: something nearer than the line
                self.av_mark, self.av_seen = self.fwd, True
            # the car rear is behind the side sensors; never seen = 1 m past the face
            if self.fwd - self.av_mark < ((SIDE_X + BODY_REAR) * L + c['AVOID_MARGIN_MM'] if self.av_seen else 1000):
                return self.go(sim, 1, c['AVOID_PCT'])
            self.avoid = 'back'
            self.say(sim, f'{self.i + 1} passed: back to the line')
        if self.avoid == 'back':
            self.avoid_off = -c['AVOID_SWERVE_DEG']
            # straighten early: turning back from the swerve at full lock still moves R (1 - cos) sideways
            lead = L / math.tan(math.radians(c['SERVO_MAX_STEER_DEG'])) * (1 - math.cos(math.radians(c['AVOID_SWERVE_DEG'])))
            if off > lead:
                return self.go(sim, 1, c['AVOID_PCT'])
            self.avoid, self.avoid_off = 'straight', 0.
        if abs(math.degrees(a)) > 3:  # straight: heading back along the line before the step goes on
            return self.go(sim, 1, c['AVOID_PCT'])
        self.avoid = None
        self.say(sim, f'{self.i + 1} back on the line')

    def find(self, sim, s, moved, angle):
        """Drive until the reading enters (seen) or leaves (lost) mm +- tol. Then, when faster than ALIGN_PCT,
        back over that edge and come up to it again at ALIGN_PCT: the stop is at the edge, not at the coast.
        The reading is corrected to the distance square to the object: reading x cos(yaw - route heading).
        It assumes the object runs parallel to the route heading.
        hold > 0: while the object is seen, P control on the heading keeps the corrected reading at hold.
        after > 0: no back-and-forward; drive on `after` m from where the edge was read, then stop.
        max limits only the search for the edge."""
        if self.phase != 'after' and abs(moved) > s['max'] * 1000:
            return self.fail(sim, f'no "{s["until"]}" edge within {s["max"]} m')
        if self.fresh:
            raw = self.mm[SENSORS.index(s['sensor'])]
            self.reading = round(raw * math.cos(angle)) if raw >= 0 else -1
            self.inside = self.reading >= 0 and abs(self.reading - s['mm']) <= s['tol']
            following = s['hold'] and self.inside and self.phase in ('arm', 'seek', 'after')
            # right-side sensor: too far (reading > hold) = turn right = negative degrees
            self.offset = clamp(-s['kp'] * (self.reading - s['hold']), -HOLD_MAX_DEG, HOLD_MAX_DEG) if following else 0.
        edge = self.inside == (s['until'] == 'seen')
        if self.phase == 'arm':  # 'lost': see the object first
            if not self.inside:
                return self.go(sim, 1, s['speed'])
            self.phase = 'seek'
        if self.phase == 'seek':
            if not edge:
                return self.go(sim, 1, s['speed'])
            self.edge = self.odo(sim)  # shortcut: read every 50 ms, so up to speed x 50 ms late; measure the lag on the car
            self.phase = 'after' if s['after'] else 'back' if s['speed'] > ALIGN_PCT else 'creep'
        if self.phase == 'after':
            left = s['after'] * 1000 - (self.odo(sim) - self.edge)
            if left > 0:  # the last 100 mm at ALIGN_PCT: less coasting after the stop
                return self.go(sim, 1, s['speed'] if left > 100 else ALIGN_PCT)
            return self.finish(sim, f'{s["after"]} m after the edge, {s["sensor"]} {self.reading} mm')
        if self.phase == 'back':
            if edge:
                return self.go(sim, -1, ALIGN_PCT)
            self.phase = 'creep'
        if not edge:
            return self.go(sim, 1, ALIGN_PCT)
        self.edge = self.odo(sim)  # Align measures from here, not from where the car stops after it coasts
        self.finish(sim, f'{s["sensor"]} {self.reading} mm, {moved / 1000:.2f} m')


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
        self.route = None                # sim only: a running Route
        # Fake BNO085 (Game Rotation Vector): raw heading -180..180, report count, handling time
        self.imu_raw = self.imu_first = self.imu_raw_next = 0.
        self.imu_count = self.imu_resets = self.imu_t = self.imu_pending = 0
        self.imu_reset_zero = 0
        start = next((o for o in self.world['objects'] if o['type'] == 'start'), None)
        if start:  # before boot(): the heading zero is the start heading, as after a calibration there
            self.place(start['x'], start['y'], start['yaw'])
        self.load()

    def place(self, x, y, yaw):
        """Put the car at x, y (mm), yaw (deg) with the IMU reading that heading. boot() after it = power-on there."""
        self.x, self.y, self.yaw = x, y, yaw
        self.imu_raw = self.imu_raw_next = self.imu_first = ((yaw + 180) % 360) - 180

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
        if self.route:
            self.route.step(self)
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

    def tof(self, pose=None):
        """The three VL53L1X of distance.h: front, right low, right high. 2D ray, no beam width, no noise.
        ASSUMED mounts: front = front of the body; right = middle of the right side.
        pose = (x mm, y mm, yaw deg) of the rear axle; default: the car."""
        c, L = self.c, self.wheelbase
        px, py, pyaw = pose or (self.x, self.y, self.yaw)
        mounts = (('front', FRONT_X * L, 0., 0., c['SIM_TOF_FRONT_HEIGHT_MM']),
                  ('right_low', SIDE_X * L, -SIDE_Y * self.track, -90., c['SIM_TOF_SIDE_LOW_HEIGHT_MM']),
                  ('right_high', SIDE_X * L, -SIDE_Y * self.track, -90., c['SIM_TOF_SIDE_HIGH_HEIGHT_MM']))
        yaw, out = math.radians(pyaw), []
        for name, fx, fy, angle, height in mounts:
            x = px + fx * math.cos(yaw) - fy * math.sin(yaw)
            y = py + fx * math.sin(yaw) + fy * math.cos(yaw)
            mm = ray_distance(self.world, x, y, pyaw + angle, height / 1000, c['TOF_MAX_RANGE_MM'])
            out.append(dict(name=name, x=x, y=y, angle=pyaw + angle, height_mm=height,
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
                route = self.sim.route
                if route:
                    odo, v = Route.odo(self.sim), self.sim.view()
                    state['route'] = dict(step=route.i, running=route.running, log=route.log[-40:],
                                          total_m=(odo - getattr(route, 'o0', odo)) / 1000,
                                          step_m=(odo - route.s0) / 1000 if route.begun else 0,
                                          heading=v['current_yaw'], tof=[t['mm'] for t in self.sim.tof()],
                                          offset_m=route.lat / 1000 if route.begun else 0, avoid=route.avoid)
                else:
                    state['route'] = None
                state['checkpoints'] = [dict(x=o['x'], y=o['y'], tof=[t['mm'] for t in self.sim.tof((o['x'], o['y'], o['yaw']))])
                                        for o in self.sim.world['objects'] if o['type'] == 'checkpoint']
            else:
                state['config'], state['config_error'] = compiled_config(sources_mtime())  # flashed firmware can differ
            return state


class Dashboard:
    """One active mode; simulation never owns or writes the serial device."""
    def __init__(self, port=None, geometry=None, placeholder=False):
        self.port = port
        self.geometry = geometry
        self.placeholder = placeholder
        self.lock = threading.RLock()
        self.token = secrets.token_urlsafe(24)
        self.world = load_world()
        self.route = load_route()
        self.sim_speed = 1  # x faster than real time while a route runs
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

    def set_route(self, steps):
        with self.lock:
            self.route = save_route(steps)

    def run_route(self, payload):
        """run true: Reset sim (car at the start, controller restarted), then run route.json. false: stop it.
        from = index of a checkpoint step: power the car on at that checkpoint, run the steps after it."""
        if (not isinstance(payload, dict) or not {'run'} <= set(payload) <= {'run', 'from', 'loop'}
                or type(payload['run']) is not bool or type(payload.get('loop', False)) is not bool):
            raise ValueError('Invalid run request')
        with self.lock:
            if payload['run']:
                first, cp = 0, None
                if 'from' in payload:
                    i = payload['from']
                    if type(i) is not int or not 0 <= i < len(self.route) or self.route[i]['type'] != 'checkpoint':
                        raise ValueError('Not a checkpoint step')
                    cp = next((o for o in self.world['objects'] if o['type'] == 'checkpoint'
                               and o.get('label') == self.route[i]['label']), None)
                    if cp is None:
                        raise ValueError(f'No checkpoint "{self.route[i]["label"]}" on the track')
                    first = i + 1
                self.switch('sim')
                if cp:
                    sim = self.robot.sim
                    sim.place(cp['x'], cp['y'], cp['yaw'])
                    sim.boot()
                    self.robot.pose = [sim.x, sim.y, math.radians(sim.yaw)]
                self.robot.sim.route = Route(self.route, first, payload.get('loop', False))
            elif self.robot.demo and self.robot.sim.route:
                self.robot.sim.route.running = False
                self.robot.sim.keys.append(' ')

    def set_speed(self, payload):
        if not isinstance(payload, dict) or set(payload) != {'x'} or payload['x'] not in (1, 2, 4, 8):
            raise ValueError('Speed must be 1, 2, 4 or 8')
        with self.lock:
            self.sim_speed = payload['x']

    def align_distance(self, payload):
        """Align step i: the distance from the edge of the find step before it to its checkpoint, from the map."""
        with self.lock:
            i = payload.get('i') if isinstance(payload, dict) else None
            if type(i) is not int or not 0 <= i < len(self.route) or self.route[i]['type'] != 'align':
                raise ValueError('Not an align step')
            if not self.robot.demo:
                raise ValueError('Switch to Sim first')
            find = next((s for s in reversed(self.route[:i]) if s['type'] == 'find'), None)
            cp = next((o for o in self.world['objects'] if o['type'] == 'checkpoint' and o.get('label') == self.route[i]['label']), None)
            if find is None or cp is None:
                raise ValueError('Align needs a Find step before it and a checkpoint with its label')
            m = edge_to_checkpoint(self.robot.sim, cp, find)
            if m is None:
                raise ValueError(f'No {find["until"]} edge for {find["sensor"]} {find["mm"]} ± {find["tol"]} mm on the line through the checkpoint')
            return round(m, 3)

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
            route = self.robot.demo and self.robot.sim.route
            if route and route.running and state.get('stop') is not True:
                raise ValueError('Route running: press ■ Stop first')
            self.robot.command(state)

    def snapshot(self):
        with self.lock:
            return dict(self.robot.snapshot(), generation=self.generation, sim_speed=self.sim_speed,
                        live_available=bool(self.port))

    def run(self):
        before = time.monotonic()
        while not self.closed:
            now = time.monotonic()
            with self.lock:
                if not self.robot.error:
                    try:
                        route = self.robot.demo and self.robot.sim.route
                        # only while a route runs: manual W/S at x4 would miss COMMAND_TIMEOUT_MS in sim time
                        self.robot.tick(now, min(now-before, .1) * (self.sim_speed if route and route.running else 1))
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
            elif self.path == '/route':
                self.reply(200, robot.route)
            else:
                self.reply(404, {'error': 'Not found'})

        def do_POST(self):
            if not self.valid_host() or self.path not in ('/control', '/mode', '/world', '/pose', '/route', '/run', '/align', '/speed') or self.headers.get('X-Control-Token') != robot.token:
                self.reply(403, {'error': 'Forbidden'})
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size < (262144 if self.path in ('/world', '/route') else 512):
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
                elif self.path == '/route':
                    robot.set_route(payload)
                elif self.path == '/run':
                    robot.run_route(payload)
                elif self.path == '/speed':
                    robot.set_speed(payload)
                elif self.path == '/align':
                    self.reply(200, {'m': robot.align_distance(payload)})
                    return
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
