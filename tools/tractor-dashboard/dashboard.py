#!/usr/bin/env python3
"""Local STM32 dashboard. Python standard library only on macOS and Linux. Windows: Live mode needs pyserial."""
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
import shutil
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
        self.buffer, self.port = b'', None
        if os.name == 'nt':  # Windows has no termios: use pyserial there
            try:
                import serial
            except ImportError:
                raise OSError(0, 'pyserial is missing. Install it with: py -m pip install pyserial') from None
            try:
                self.port = serial.Serial(path, 115200, timeout=0, write_timeout=1)
            except serial.SerialException as exc:
                raise OSError(0, str(exc)) from None
            return
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

    def write(self, text):
        data = text.encode('ascii')
        n = self.port.write(data) if self.port else os.write(self.fd, data)
        if n != len(data):
            raise OSError('Incomplete serial write')

    def lines(self):
        if self.port:
            chunk = self.port.read(4096)  # pyserial raises SerialException (an OSError) on a lost cable
            if not chunk:
                return []
        else:
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

    def close(self):
        if self.port:
            self.port.close()
        else:
            os.close(self.fd)


def find_port(kind):
    """'auto': the ST-LINK USB serial port of the Nucleo board. 'bt': the paired HC-06 module.
    Returns the first match or None. The port name is different on each PC and USB socket."""
    if os.name == 'nt':
        try:
            from serial.tools import list_ports
        except ImportError:
            raise SystemExit('pyserial is missing. Install it with: py -m pip install pyserial') from None
        for p in list_ports.comports():
            if kind == 'auto' and p.vid == 0x0483 or kind == 'bt' and 'bluetooth' in (p.description or '').lower():
                return p.device
        return None
    import glob
    patterns = ['/dev/cu.usbmodem*', '/dev/ttyACM*'] if kind == 'auto' else ['/dev/cu.HC-0*', '/dev/rfcomm*']
    found = sorted(f for pattern in patterns for f in glob.glob(pattern))
    return found[0] if found else None


def c_compiler():
    """$CC, else the first of cc, gcc, clang on the PATH. Windows: MinGW-w64 gcc (see README)."""
    cc = os.environ.get('CC') or next(filter(None, map(shutil.which, ('cc', 'gcc', 'clang'))), None)
    if not cc:
        raise RuntimeError('No C compiler found. Install gcc or clang (Windows: MinGW-w64 gcc, see README), or set CC')
    return cc


FIRMWARE = Path(__file__).resolve().parents[2] / 'firmware' / 'H533RE'
CORE_SOURCES = [FIRMWARE / 'App/Control' / n for n in ('control.c', 'pid.c', 'math_utils.c', 'route.c')]
CORE_FILES = CORE_SOURCES + [FIRMWARE / 'App/Control' / n for n in ('control.h', 'pid.h', 'math_utils.h', 'route.h')] + [
    FIRMWARE / 'App/Config/constants.h', Path(__file__).with_name('control_binding.c')]
BUILD = Path(__file__).with_name('build')


STEP_TYPES = ('drive', 'turn', 'find', 'checkpoint', 'align')  # = StepType in route.h
AVOID_NAMES = (None, 'wait', 'reverse', 'out', 'pass', 'back', 'straight', 'wait2')  # = AvoidPhase
ROUTE_VIEW = 'i running begun lap s0 lat reading avoid set_goal goal_deg nkeys nmsg'.split()  # = control_binding.c


class RouteStepC(ctypes.Structure):  # = RouteStep in route.h
    _fields_ = ([('type', ctypes.c_uint8), ('sensor', ctypes.c_uint8), ('until_seen', ctypes.c_uint8)]
                + [(n, ctypes.c_float) for n in 'm speed deg mm tol max hold kp after s'.split()]
                + [('label', ctypes.c_char * 25)])


class RouteConfigC(ctypes.Structure):  # = RouteConfig
    _fields_ = [(n, ctypes.c_float) for n in ('wheelbase_mm track_mm servo_max_steer_deg avoid_stop_mm avoid_wait_s '
                                             'avoid_back_mm avoid_swerve_deg avoid_margin_mm avoid_max_offset_mm '
                                             'avoid_look_mm avoid_pct').split()]


class RouteInputC(ctypes.Structure):  # = RouteInput
    _fields_ = [('now_ms', ctypes.c_uint32)] + [(n, ctypes.c_float) for n in ('odo_mm', 'current_yaw', 'heading_goal', 'max_abs_rpm')]


READ_TOF = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int32))


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
        lib.route_size.restype = lib.route_step_size.restype = ctypes.c_size_t
        if lib.route_step_size() != ctypes.sizeof(RouteStepC):
            raise RuntimeError('RouteStep in route.h and RouteStepC in dashboard.py differ')
        lib.route_start.argtypes = [ctypes.c_void_p, ctypes.POINTER(RouteStepC), ctypes.c_int, ctypes.c_int, ctypes.c_bool,
                                    ctypes.POINTER(RouteConfigC), READ_TOF, ctypes.c_void_p]
        lib.route_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(RouteInputC)]
        lib.route_stop.argtypes = [ctypes.c_void_p]
        lib.route_view.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double)]
        for name, res in (('route_key', ctypes.c_char), ('route_msg', ctypes.c_char_p), ('route_report', ctypes.c_int)):
            getattr(lib, name).argtypes, getattr(lib, name).restype = [ctypes.c_void_p, ctypes.c_int], res
        self.body = tuple(ctypes.cast(lib.route_body, ctypes.POINTER(ctypes.c_double))[k] for k in range(6))
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
        ext = '.dll' if os.name == 'nt' else '.so'
        out = BUILD / f'control-{sys.platform}-{platform.machine()}-{digest}{ext}'  # one build per OS and CPU
        if not out.exists():
            BUILD.mkdir(exist_ok=True)
            pic = ['-static-libgcc'] if os.name == 'nt' else ['-fPIC']  # Windows DLL: no libgcc DLL needed
            cmd = [c_compiler(), '-std=c11', '-O2', '-shared', *pic,
                   f'-I{FIRMWARE}/App/Control', f'-I{FIRMWARE}/App/Config', *map(str, CORE_SOURCES),
                   str(CORE_FILES[-1]), '-lm', '-o', str(out) + '.tmp']
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError('Control core does not compile: ' + (result.stderr.strip().splitlines() or ['?'])[0])
            os.replace(str(out) + '.tmp', out)
            for old in BUILD.glob('control-*'):
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
        return validate_world(json.loads(Path(path).read_text(encoding='utf-8')))
    except FileNotFoundError:
        return json.loads(json.dumps(DEFAULT_WORLD))


def save_world(world, path=None):
    world = validate_world(world)
    path = Path(path or WORLD_FILE)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(world, indent=1), encoding='utf-8')
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
STEP_OPTIONAL = {'hold': 0, 'kp': .05, 'after': 0}  # route files from before wall following
# Car body and ToF mounts, as fractions of the wheelbase L (forward from the rear axle) and the track T.
# Same values as ROUTE_FRONT_X ... in route.h (a test checks it). index.html draws the same body.
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
            if not isinstance(s.get('label'), str) or len(s['label']) > 24 or not all(' ' <= ch <= '~' for ch in s['label']):
                raise ValueError(f'Step {len(clean) + 1} {s["type"]}: invalid label')
            out['label'] = s['label']
        clean.append(out)
    return clean


def load_route(path=None):
    try:
        return validate_route(json.loads(Path(path or ROUTE_FILE).read_text(encoding='utf-8')))
    except FileNotFoundError:
        return []


def save_route(steps, path=None):
    steps = validate_route(steps)
    path = Path(path or ROUTE_FILE)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(steps, indent=1), encoding='utf-8')
    os.replace(tmp, path)
    return steps


FIRMWARE_ROOT = Path(__file__).resolve().parents[2] / 'firmware'
ROUTE_HEADERS = [d / 'App/Config/route_steps.h' for d in sorted(FIRMWARE_ROOT.glob('*')) if (d / 'App/Config').is_dir()]


def route_id(steps):
    """Short id of a route. The firmware sends the id of its flashed route; the dashboard compares."""
    return hashlib.sha256(json.dumps(steps, sort_keys=True).encode()).hexdigest()[:8]


def route_header(steps):
    """The steps as C code for the firmware: App/Config/route_steps.h."""
    rows = []
    for st in steps:
        fields = [f'.type = STEP_{st["type"].upper()}']
        if st['type'] == 'find':
            fields += [f'.sensor = SENSOR_{st["sensor"].upper()}', f'.until_seen = {int(st["until"] == "seen")}']
        fields += [f'.{k} = {float(st[k])!r}f' for k in ('m', 'speed', 'deg', 'mm', 'tol', 'max', 'hold', 'kp', 'after', 's') if k in st]
        if 'label' in st:
            fields.append('.label = ' + json.dumps(st['label']))  # printable ASCII: also a valid C string
        rows.append('  { ' + ', '.join(fields) + ' },')
    return '\n'.join([
        '/* Made by the dashboard from tools/tractor-dashboard/route.json when you save the route.',
        ' * Do not edit this file: edit the route in the dashboard, then build and flash. */',
        '#ifndef ROUTE_STEPS_H', '#define ROUTE_STEPS_H', '', '#include "route.h"', '',
        f'#define ROUTE_ID          "{route_id(steps)}"  /* the dashboard compares it with the saved route */',
        f'#define ROUTE_STEP_COUNT  {len(steps)}', '',
        'static const RouteStep ROUTE_STEPS[] = {', *(rows or ['  { 0 },  /* no steps */']), '};', '',
        '#endif /* ROUTE_STEPS_H */', ''])


def write_route_headers(steps):
    for path in ROUTE_HEADERS:
        tmp = path.with_suffix('.tmp')
        with open(tmp, 'w', encoding='utf-8', newline='\n') as f:  # LF on Windows too; Path.write_text has no newline before 3.10
            f.write(route_header(steps))
        os.replace(tmp, path)


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


def c_steps(steps):
    """The steps as a RouteStep array for route.c."""
    out = (RouteStepC * max(len(steps), 1))()
    for c, st in zip(out, steps):
        c.type = STEP_TYPES.index(st['type'])
        for k in ('m', 'speed', 'deg', 'mm', 'tol', 'max', 'hold', 'kp', 'after', 's'):
            setattr(c, k, st.get(k, 0))
        if st['type'] == 'find':
            c.sensor, c.until_seen = SENSORS.index(st['sensor']), st['until'] == 'seen'
        c.label = st.get('label', '').encode()
    return out


class Route:
    """Sim: runs the C route runner (App/Control/route.c, the same code as the firmware) on a SimCar.
    The runner gives W/S, speed digits and Space as keys, and the heading goal. It decides from wheel
    distance, the controller's IMU heading and the ToF readings, never from the true pose. Only the
    checkpoint report here uses the true pose, to show the error."""

    def __init__(self, steps, first=0, loop=False):
        self.steps, self.first, self.loop = steps, first, loop
        self.log, self.r, self.sim, self.o0 = [], None, None, None
        self.v = dict(zip(ROUTE_VIEW, [first, 1, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0]))

    i = property(lambda self: int(self.v['i']))
    lap = property(lambda self: int(self.v['lap']))
    begun = property(lambda self: bool(self.v['begun']))
    s0 = property(lambda self: self.v['s0'])
    lat = property(lambda self: self.v['lat'])
    reading = property(lambda self: int(self.v['reading']))
    avoid = property(lambda self: AVOID_NAMES[int(self.v['avoid'])])

    @property
    def running(self):
        return bool(self.v['running'])

    @running.setter
    def running(self, on):
        if not on:
            self.v['running'] = 0
            if self.r is not None:
                self.lib.route_stop(self.r)

    def say(self, sim, text):
        self.log = (self.log + [f'{sim.now / 1000:6.1f} s  {text}'])[-100:]

    @staticmethod
    def odo(sim):
        """Distance, mm, + forward. The sim wheel turns: the encoders give the same to one count."""
        return sum(sim.wheel_pos) / 2 * math.pi * sim.c['WHEEL_DIAMETER_MM']

    def _tof(self, ctx, out):
        for k, t in enumerate(self.sim.tof()):
            out[k] = t['mm']

    def _start(self, sim):
        c, lib = sim.c, sim.lib.lib
        self.lib, self.cfg, self.c_steps = lib, RouteConfigC(sim.wheelbase, sim.track, c['SERVO_MAX_STEER_DEG'], *(c[k] for k in (
            'AVOID_STOP_MM', 'AVOID_WAIT_S', 'AVOID_BACK_MM', 'AVOID_SWERVE_DEG', 'AVOID_MARGIN_MM',
            'AVOID_MAX_OFFSET_MM', 'AVOID_LOOK_MM', 'AVOID_PCT'))), c_steps(self.steps)
        self.read_tof = READ_TOF(self._tof)  # keep a reference: C calls it
        self.r = ctypes.create_string_buffer(lib.route_size())
        lib.route_start(self.r, self.c_steps, len(self.steps), self.first, self.loop, self.cfg, self.read_tof, None)

    def report(self, sim, i):
        s = self.steps[i]
        reads = ' / '.join(str(t['mm']) for t in sim.tof())
        cp = next((o for o in sim.world['objects'] if o['type'] == 'checkpoint' and o.get('label') == s['label']), None)
        if cp is None:
            return self.say(sim, f'{i + 1} checkpoint "{s["label"]}": not on the track. ToF F/RL/RH {reads} mm')
        expect = ' / '.join(str(t['mm']) for t in sim.tof((cp['x'], cp['y'], cp['yaw'])))
        self.say(sim, f'{i + 1} checkpoint {s["label"]}: off by {math.hypot(sim.x - cp["x"], sim.y - cp["y"]) / 10:.1f} cm, '
                      f'{sim.lib.wrap180(sim.yaw - cp["yaw"]):+.1f} deg. ToF F/RL/RH {reads} mm (at the checkpoint: {expect})')

    def step(self, sim):
        """Once per simulated ms, before the main loop reads its key: route_step() as in main.c."""
        if not self.running or sim.keys:
            return
        if self.r is None:
            self._start(sim)
        self.sim, v, odo, lib = sim, sim.view(), self.odo(sim), self.lib
        if self.o0 is None:
            self.o0 = odo
        lib.route_step(self.r, RouteInputC(sim.now % 2**32, odo, v['current_yaw'], v['heading_goal'], max(map(abs, sim.rpm))))
        out = (ctypes.c_double * len(ROUTE_VIEW))()
        lib.route_view(self.r, out)
        self.v = dict(zip(ROUTE_VIEW, out))
        if self.v['set_goal']:
            sim.lib.sim_set_heading_goal(sim.ctl, self.v['goal_deg'])
        sim.keys.extend(lib.route_key(self.r, k).decode() for k in range(int(self.v['nkeys'])))
        for k in range(int(self.v['nmsg'])):
            i = lib.route_report(self.r, k)
            if i >= 0:
                self.report(sim, i)
            else:
                self.say(sim, lib.route_msg(self.r, k).decode())


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
        self.live_route = dict(running=False, step=0, log=[], id=None)  # Live: from the "route ..." lines of the car
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
                    if line.startswith('route '):
                        self.route_line(line[6:].strip())
                        continue
                    data = parse_line(line)
                    if data is not None:
                        self.ingest(data, now)

    def route_line(self, text):
        """Route lines of the firmware (main.c): id ID N, run, stop, step I, log T TEXT."""
        r, words = self.live_route, text.split(' ', 2)
        if words[0] == 'id' and len(words) > 1:
            r['id'] = words[1]
        elif words[0] in ('run', 'stop'):
            r['running'] = words[0] == 'run'
        elif words[0] == 'step' and len(words) > 1 and words[1].isdigit():
            r['step'] = int(words[1])
        elif words[0] == 'log' and len(words) > 2:
            r['log'] = (r['log'] + [f'{words[1]:>6} s  {words[2]}'])[-100:]

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
                                          total_m=(odo - (odo if route.o0 is None else route.o0)) / 1000,
                                          step_m=(odo - route.s0) / 1000 if route.begun else 0,
                                          heading=v['current_yaw'], tof=[t['mm'] for t in self.sim.tof()],
                                          offset_m=route.lat / 1000 if route.begun else 0, avoid=route.avoid)
                else:
                    state['route'] = None
                state['checkpoints'] = [dict(x=o['x'], y=o['y'], tof=[t['mm'] for t in self.sim.tof((o['x'], o['y'], o['yaw']))])
                                        for o in self.sim.world['objects'] if o['type'] == 'checkpoint']
            else:
                state['config'], state['config_error'] = compiled_config(sources_mtime())  # flashed firmware can differ
                r = self.live_route
                state['route'] = dict(step=r['step'], running=r['running'], log=r['log'][-40:], flashed=r['id'], live=True,
                                      total_m=0, step_m=0, offset_m=0, avoid=None, tof=[-1] * 3,
                                      heading=(self.data or {}).get('game_yaw', 0))
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
                raise ValueError('Start with --port auto (or the port name) to enable Live')
            if self.robot.serial:
                try:
                    self.robot.stop()
                except OSError:
                    pass  # A lost cable cannot receive stop; firmware timeout remains.
                finally:
                    self.robot.serial.close()
            self.generation += 1
            self.robot = Robot(demo=(mode == 'sim'),
                               geometry=[100, 300, 220] if mode == 'sim' else self.geometry,
                               placeholder_geometry=mode == 'live' and self.placeholder, world=self.world)
            if mode == 'live':
                try:
                    self.robot.serial = Serial(self.port)
                    self.robot.send('?')  # the car answers with the id of its flashed route
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
            write_route_headers(self.route)  # build and flash to put it on the car

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
            if not self.robot.demo and self.robot.live_route['running']:
                if state.get('stop') is not True:
                    raise ValueError('Auto mode: click Manual first')
                self.robot.live_route['running'] = False  # the car stops the route at any key
            self.robot.command(state)

    def set_auto(self, payload):
        """Live. Auto: the car runs the route that is flashed in its firmware (key R).
        Manual: Space, the car stops the route and the motors."""
        if not isinstance(payload, dict) or set(payload) != {'on'} or type(payload['on']) is not bool:
            raise ValueError('Invalid auto request')
        with self.lock:
            if self.robot.demo or not self.robot.serial:
                raise ValueError('Auto needs Live and a connected car')
            if payload['on']:
                self.robot.send('r')
                self.robot.live_route['running'] = True  # the car confirms with "route run"
            else:
                self.robot.stop()
                self.robot.live_route['running'] = False

    def snapshot(self):
        with self.lock:
            state = dict(self.robot.snapshot(), generation=self.generation, sim_speed=self.sim_speed,
                         live_available=bool(self.port), route_id=route_id(self.route))
            r = state.get('route')
            if r and r.get('live') and r['flashed'] and r['flashed'] != state['route_id']:
                r['warning'] = 'The car has another route than the saved one: build and flash the firmware.'
            return state

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
                    self.robot.serial.close()
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
                page = Path(__file__).with_name('index.html').read_text(encoding='utf-8').replace('__TOKEN__', robot.token)
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
            if not self.valid_host() or self.path not in ('/control', '/mode', '/world', '/pose', '/route', '/run', '/auto', '/align', '/speed') or self.headers.get('X-Control-Token') != robot.token:
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
                elif self.path == '/auto':
                    robot.set_auto(payload)
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
    parser.add_argument('--port', help='auto = the Nucleo ST-LINK port, bt = the paired HC-06, or a name: COM5, /dev/cu.usbmodem1103')
    parser.add_argument('--http-port', type=int, default=8766)
    geometry_options = parser.add_mutually_exclusive_group()
    geometry_options.add_argument('--placeholder-geometry', action='store_true', help='Illustrative 100/300/220 mm geometry; not calibrated')
    geometry_options.add_argument('--geometry-mm', type=float, nargs=3, metavar=('DIAMETER', 'WHEELBASE', 'TRACK'), help='Measured dimensions; enables ESTIMATED odometry only')
    args = parser.parse_args()
    if args.port in ('auto', 'bt'):
        found = find_port(args.port)
        if not found:
            parser.error('No Nucleo board found. Connect it, or give the port name.' if args.port == 'auto'
                         else 'No paired HC-06 found. Pair it first, or give the port name.')
        args.port = found
    if args.geometry_mm and any(not math.isfinite(v) or v <= 0 for v in args.geometry_mm):
        parser.error('All dimensions must be finite and positive')
    robot = Dashboard(args.port, args.geometry_mm or ([100, 300, 220] if args.placeholder_geometry else None), args.placeholder_geometry)
    if args.port and not args.demo:
        robot.switch('live')
    server = HTTPServer(('127.0.0.1', args.http_port), handler(robot))
    server.timeout = .2
    thread = threading.Thread(target=robot.run, daemon=True)
    thread.start()
    print(f"Open http://127.0.0.1:{args.http_port} in a browser. Ctrl+C stops. Mode: {'SIMULATION (no robot)' if robot.robot.demo else args.port}", flush=True)
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
