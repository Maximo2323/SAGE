#!/usr/bin/env python3
"""Local STM32 dashboard. Python standard library only; serial supports macOS/Linux."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import secrets
import select
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


class Robot:
    def __init__(self, serial=None, demo=False, geometry=None, placeholder_geometry=False):
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
        self.sim_rpm = self.sim_steer = 0.
        self.sim_goal = 0.
        self.token = secrets.token_urlsafe(24)
        self.closed = False

    def send(self, keys):
        if self.serial and not self.demo:
            self.serial.write(keys)
        if self.demo:
            for key in keys:
                if key == ' ':
                    self.sim_goal = self.sim_rpm = self.sim_steer = 0.
                elif key in 'ad':
                    self.sim_steer = max(-30, min(30, self.sim_steer + (5 if key == 'a' else -5)))
                elif key in 'ws':
                    self.sim_goal = (1 if key == 'w' else -1) * (100 if self.speed == '0' else int(self.speed)*10)*2

    def stop(self):
        self.send(' ')
        self.drive = self.turn = ''

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

    def ingest(self, data, now):
        dt = 0 if self.received is None else now - self.received
        self.data, self.received = data, now
        if self.geometry and 0 < dt <= .3:
            diameter, wheelbase, track = self.geometry
            distance = (data['rpm_left'] + data['rpm_right']) / 2 * math.pi * diameter / 60 * dt
            delta = distance * math.tan(math.radians(data['steer'])) / wheelbase
            self.pose[0] += distance * math.cos(self.pose[2] + delta / 2)
            self.pose[1] += distance * math.sin(self.pose[2] + delta / 2)
            self.pose[2] += delta
            if abs(distance) > .1:
                self.trail.append(self.pose.copy())
                self.trail = self.trail[-400:]

    def tick(self, now, dt):
        with self.lock:
            if self.drive and now - self.last_input > .2:
                self.stop()
            if self.demo:
                rate = 1000 if abs(self.sim_goal) < abs(self.sim_rpm) else 200
                self.sim_rpm += max(-rate*dt, min(rate*dt, self.sim_goal-self.sim_rpm))
                data = dict(zip(FIELDS, [int(self.sim_rpm), int(self.sim_rpm), int(self.sim_rpm), int(self.sim_rpm/2), int(self.sim_rpm/2), int(self.sim_steer), int(math.degrees(self.pose[2])) % 360, int(math.degrees(self.pose[2])) % 360, 0, 0]))
                data['flags'] = []
                self.ingest(data, now)
            elif self.serial:
                for line in self.serial.lines():
                    data = parse_line(line)
                    if data is not None:
                        self.ingest(data, now)

    def snapshot(self):
        with self.lock:
            age = None if self.received is None else time.monotonic() - self.received
            return dict(mode='SIMULATION' if self.demo else 'SERIAL', data=self.data,
                        stale=age is None or age > .5, age=age, error=self.error,
                        geometry=self.geometry, placeholder_geometry=self.placeholder_geometry, pose=self.pose.copy(), trail=self.trail.copy())

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
        self.robot = Robot(demo=True, geometry=[100, 300, 220])
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
                               placeholder_geometry=mode == 'live' and self.placeholder)
            if mode == 'live':
                try:
                    self.robot.serial = Serial(self.port)
                except OSError as exc:
                    self.robot.error = f'Cannot open {self.port}: {exc.strerror}. Close other serial apps, then click Live to retry.'

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
            else:
                self.reply(404, {'error': 'Not found'})

        def do_POST(self):
            if not self.valid_host() or self.path not in ('/control', '/mode') or self.headers.get('X-Control-Token') != robot.token:
                self.reply(403, {'error': 'Forbidden'})
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size < 512:
                    raise ValueError('Invalid body length')
                payload = json.loads(self.rfile.read(size))
                if self.path == '/mode':
                    if not isinstance(payload, dict) or set(payload) != {'mode'}:
                        raise ValueError('Invalid mode request')
                    robot.switch(payload['mode'])
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
