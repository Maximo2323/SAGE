import math
import os
import random
import re
import struct
import subprocess
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
import dashboard
from dashboard import Robot, Serial, Dashboard, SimCar, parse_line

LINE = 'tgt -20 rpm -19 -21 cmd -10 -11 steer 5 g -12 r 13 rst 2 ov 4 STALL IMU-STALE\r\n'

class Port:
    def __init__(self): self.sent = ''
    def write(self, s): self.sent += s
    def lines(self): return []
    def close(self): self.closed = True

class Tests(unittest.TestCase):
    def state(self, drive='', turn='', speed='1', stop=False):
        return dict(drive=drive, turn=turn, speed=speed, stop=stop)

    def test_parser(self):
        frame = parse_line(LINE)
        self.assertEqual(frame['rpm_left'], -19)
        self.assertEqual(frame['flags'], ['STALL', 'IMU-STALE'])
        for bad in ['booting', LINE.replace('rpm', 'speed'), LINE+'garbage', LINE[:30]]:
            self.assertIsNone(parse_line(bad))

    def test_release_turn_and_watchdog(self):
        port = Port(); r = Robot(port)
        r.command(self.state('w', 'a')); r.command(self.state('w', 'a'))
        self.assertEqual(port.sent, '1aww')
        r.command(self.state())
        self.assertEqual(port.sent, '1aww ')
        r.command(self.state('s')); r.tick(r.last_input+.21, .01)
        self.assertTrue(port.sent.endswith('s '))
        previous = port.sent; r.tick(r.last_input+.3, .01)
        self.assertEqual(port.sent, previous)

    def test_invalid_state_sends_nothing(self):
        port = Port(); r = Robot(port)
        for state in [None, {}, self.state(drive='x'), self.state(speed='99'), self.state(stop=1)]:
            with self.assertRaises(ValueError): r.command(state)
        self.assertEqual(port.sent, '')

    def test_odometry_and_stale_gap(self):
        r = Robot(geometry=[100,300,220])
        frame = parse_line('tgt 60 rpm 60 60 cmd 30 30 steer 0 g 0 r 0 rst 0 ov 0')
        r.ingest(frame, 1); r.ingest(frame, 1.1)
        self.assertAlmostEqual(r.pose[0], math.pi*10)
        previous = r.pose.copy(); r.ingest(frame, 5)
        self.assertEqual(previous, r.pose)
        with patch('dashboard.time.monotonic', return_value=6):
            self.assertTrue(r.snapshot()['stale'])
        uncalibrated = Robot(); uncalibrated.ingest(frame, 1); uncalibrated.ingest(frame, 1.1)
        self.assertEqual(uncalibrated.pose, [0,0,0])

    @unittest.skipIf(os.name == 'nt', 'POSIX pty only')
    def test_serial_pty_fragmented_frames_and_commands(self):
        import pty
        master, slave = pty.openpty()
        serial = Serial(os.ttyname(slave))
        try:
            os.write(master, LINE[:20].encode()); time.sleep(.01)
            self.assertEqual(serial.lines(), [])
            os.write(master, LINE[20:].encode()); time.sleep(.01)
            self.assertEqual(parse_line(serial.lines()[0])['overruns'], 4)
            serial.write('w ')
            self.assertEqual(os.read(master, 2), b'w ')
        finally:
            serial.close(); os.close(master); os.close(slave)

    def test_simulated_stop(self):
        r = Robot(demo=True, geometry=[100,300,220]); r.command(self.state('w'))
        now = r.last_input
        r.tick(now, .1)
        self.assertTrue(0 < r.data['target'] <= 20)  # ramp of control.c: 200 RPM/s
        r.command(self.state(stop=True)); r.tick(now+.1, .1)
        self.assertEqual(r.data['target'], 0)
        self.assertEqual(r.snapshot()['mode'], 'SIMULATION')

FIRMWARE = dashboard.FIRMWARE


def drive(car, seconds, key='w'):
    """Hold a key as the browser does (one key each 80 ms)."""
    for _ in range(int(seconds / .08)):
        car.keys.append(key)
        car.advance(.08)


class SimTests(unittest.TestCase):
    """The sim runs the firmware control core (control.c) with the values of constants.h."""

    def test_no_python_controller(self):
        for name in ('Pid', 'load_constants', 'wrap180'):
            self.assertFalse(hasattr(dashboard, name), name)
        self.assertNotIn('kp *', Path(dashboard.__file__).read_text(encoding='utf-8'))

    def test_dashboard_calls_the_c_core(self):
        car = SimCar()
        calls = []
        real = car.lib

        class Spy:
            def __getattr__(self, name):
                attr = getattr(real, name)
                if not callable(attr):
                    return attr
                def call(*args):
                    calls.append(name)
                    return attr(*args)
                return call
        car.lib = Spy()
        robot = Robot(demo=True)
        robot.sim = car
        robot.command(dict(drive='w', turn='a', speed='5', stop=False))
        robot.tick(1., .2)
        for name in ('control_key', 'control_tick', 'control_wheels', 'sim_steering', 'view'):
            self.assertIn(name, calls)
        v = car.view()
        self.assertEqual(v['speed_pct'], 50)
        self.assertGreater(v['last_seq'], 0)       # the C state advanced: encoder samples
        self.assertGreater(v['last_count'], 0)     # and heading reports

    def test_config_is_the_compiled_constants(self):
        car = SimCar()
        text = (FIRMWARE / 'App/Config/constants.h').read_text(encoding='utf-8')
        for name in ('WHEEL_KP', 'WHEEL_KI', 'WHEEL_KD', 'HEADING_KP', 'HEADING_KI', 'HEADING_KD', 'WHEEL_DIAMETER_MM'):
            literal = re.search(rf'#define\s+{name}\s+([-\d.]+)f', text)[1]
            self.assertAlmostEqual(car.c[name], float(literal), places=6)
        self.assertEqual(car.c['WHEEL_DIAMETER_MM'], 65.0)

    def test_integral_removes_speed_error(self):
        car = SimCar()
        drive(car, 0.1, '5'); drive(car, 4)
        self.assertAlmostEqual(car.rpm[0], 100, delta=2)  # SIM_MOTOR_GAIN < 1: the feedforward alone stays low

    def test_heading_hold_turns_and_commanded_differs_from_actual(self):
        car = SimCar()
        drive(car, 0.1, '5'); drive(car, 1)
        car.keys.append('a'); drive(car, 0.1)
        v = car.view()
        self.assertEqual(v['heading_goal'], car.c['HEADING_STEP_DEG'])
        self.assertNotEqual(round(car.steer_actual, 3), round(v['steer_deg'], 3))  # the servo lags
        drive(car, 15)
        self.assertAlmostEqual(car.lib.wrap180(car.yaw - 0), car.c['HEADING_STEP_DEG'], delta=2)

    def test_stop_key_and_command_timeout(self):
        car = SimCar()
        drive(car, 1)
        self.assertGreater(car.view()['target_rpm'], 0)
        car.advance(.5)                          # no key: hold-to-drive timeout, then the ramp down
        self.assertEqual(car.view()['drive_dir'], 0)
        drive(car, 1); car.keys.append(' '); car.advance(.01)
        v = car.view()
        self.assertEqual((v['target_rpm'], v['cmd_l']), (0, 0))

    def test_stale_imu_stops_the_car(self):
        car = SimCar()
        drive(car, 1)
        car.imu_resets += 1                      # the sensor did a reset: the heading is not valid
        drive(car, .1)
        self.assertEqual(car.view()['drive_dir'], 0)
        self.assertIn('IMU-STALE', car.telemetry()['flags'])

    def test_rebuild_restarts_the_controller(self):
        car = SimCar()
        drive(car, 1, 'a'); drive(car, 1)
        x = car.x
        car.mtime = -1                           # as after a change of constants.h
        car.advance(.001)
        v = car.view()
        self.assertEqual((v['heading_target'], v['target_rpm'], v['drive_dir']), (0, 0, 0))
        self.assertGreaterEqual(car.x, x)        # the car keeps its pose
        self.assertAlmostEqual(car.lib.wrap180(car.imu_raw - car.zero), 0, delta=1e-3)  # new heading zero

    def test_build_error_is_reported_and_old_core_kept(self):
        car = SimCar()
        with patch('dashboard.ControlLib.build', side_effect=RuntimeError('Control core does not compile: x')):
            car.mtime = -1
            car.advance(.01)
        self.assertIn('does not compile', car.error)
        drive(car, .2)                           # still runs
        self.assertEqual(Robot(demo=True).snapshot()['sim']['error'], '')


class WorldTests(unittest.TestCase):
    """Track editor backend: file, validation, start pose, TOF ray casting with heights."""
    WALL = {'type': 'wall', 'pts': [[1000, -500], [1000, 500]], 'h': 0.2}

    def test_save_load_and_validation(self):
        path = Path(tempfile.mkdtemp()) / 'track.json'
        world = {'objects': [self.WALL, {'type': 'circle', 'x': 1, 'y': 2, 'r': 30, 'h': 1.5, 'label': 'post'},
                             {'type': 'start', 'x': 0, 'y': 0, 'yaw': 0}]}
        dashboard.save_world(world, path)
        self.assertEqual(dashboard.load_world(path), world)
        for bad in [None, {'objects': [{'type': 'tree'}]}, {'objects': [{**self.WALL, 'h': float('nan')}]},
                    {'objects': [{**self.WALL, 'pts': [[0, 0]]}]}, {'objects': [{'type': 'box', 'x': 0, 'y': 0, 'w': -1, 'd': 1, 'h': 1}]},
                    {'objects': [{'type': 'start', 'x': 0, 'y': 0, 'yaw': 0}] * 2}, {'objects': [{**self.WALL, 'label': 'x' * 99}]}]:
            with self.assertRaises(ValueError):
                dashboard.save_world(bad, path)
        self.assertEqual(dashboard.load_world(path), world)  # a rejected track does not touch the file
        self.assertEqual(dashboard.load_world(path.with_name('missing.json')), dashboard.DEFAULT_WORLD)
        old = dashboard.validate_world({'objects': [{'type': 'checkpoint', 'x': 0, 'y': 0}]})  # file from before headings
        self.assertEqual(old['objects'][0]['yaw'], 90.0)

    def test_ray_heights_and_shapes(self):
        world = {'objects': [self.WALL, {'type': 'circle', 'x': 500, 'y': 0, 'r': 100, 'h': 2.0},
                             {'type': 'box', 'x': -700, 'y': -100, 'w': 200, 'd': 200, 'h': 0.05}]}
        self.assertAlmostEqual(dashboard.ray_distance(world, 0, 0, 0, 0.03, 4000), 400)   # circle first
        self.assertAlmostEqual(dashboard.ray_distance(world, 0, 0, 180, 0.03, 4000), 500)  # low box
        self.assertIsNone(dashboard.ray_distance(world, 0, 0, 180, 0.08, 4000))           # too high to see it
        self.assertIsNone(dashboard.ray_distance(world, 0, 0, 90, 0.03, 4000))            # nothing there
        self.assertIsNone(dashboard.ray_distance(world, 0, 0, 0, 0.03, 300))              # out of range

    def test_sim_starts_at_start_and_reads_sensors(self):
        world = {'objects': [self.WALL, {'type': 'start', 'x': 0, 'y': 0, 'yaw': 0}]}
        car = SimCar(world=world)
        self.assertEqual((car.x, car.y, car.yaw), (0, 0, 0))
        front, low, high = car.tof()
        self.assertEqual(front['mm'], int(1000 - 1.18 * car.wheelbase))           # wall 0.2 m > 80 mm sensor
        self.assertEqual((low['mm'], high['mm']), (-1, -1))                       # nothing on the right
        car.yaw = 90.                                                             # the wall is now on the right
        self.assertGreater(car.tof()[1]['mm'], 0)
        self.assertEqual(car.tof()[2]['mm'], -1)                                  # 1.5 m sensor sees over it
        self.assertEqual(car.telemetry()['dist'], [s['mm'] for s in car.tof()])
        drive(car, .5)
        self.assertEqual(car.view()['heading_target'], 0)                         # heading zero = start heading

    def test_move_car_and_endpoints_need_sim(self):
        with patch('dashboard.WORLD_FILE', Path(tempfile.mkdtemp()) / 'track.json'):
            d = Dashboard(port='fake')
            d.move_car({'x': 123, 'y': -45})
            self.assertEqual(d.robot.snapshot()['pose'][:2], [123, -45])
            with self.assertRaises(ValueError):
                d.move_car({'x': 'a', 'y': 0})
            d.set_world({'objects': [self.WALL]})
            self.assertEqual(d.robot.sim.world['objects'], [self.WALL])
            with patch('dashboard.Serial', return_value=Port()):
                d.switch('live')
            with self.assertRaises(ValueError):
                d.move_car({'x': 0, 'y': 0})


class RouteTests(unittest.TestCase):
    """Route blocks: run in the sim through the firmware keys and the heading goal."""
    def run_route(self, world, steps, seconds):
        car = SimCar(world=world)
        car.route = dashboard.Route(dashboard.validate_route(steps))
        for _ in range(seconds):
            car.advance(1)
            if not car.route.running:
                break
        self.assertFalse(car.route.running, car.route.log)
        self.assertIn('route done', car.route.log[-1])
        return car

    def test_drive_checkpoint_turn(self):
        world = {'objects': [{'type': 'start', 'x': 0, 'y': 0, 'yaw': 0},
                             {'type': 'checkpoint', 'x': 1000, 'y': 0, 'yaw': 0, 'label': 'C1'}]}
        car = self.run_route(world, [{'type': 'drive', 'm': 1, 'speed': 30}, {'type': 'checkpoint', 'label': 'C1', 's': 1},
                                     {'type': 'turn', 'deg': 90, 'speed': 30}], 60)
        error = float(re.search(r'off by ([\d.]+) cm', ' '.join(car.route.log))[1])
        self.assertLess(error, 5)
        self.assertAlmostEqual(car.lib.wrap180(car.yaw - 90), 0, delta=4)

    def test_find_lost_stops_at_the_end_of_the_wall(self):
        world = {'objects': [{'type': 'start', 'x': 0, 'y': 0, 'yaw': 0},
                             {'type': 'wall', 'pts': [[-500, -500], [2000, -500]], 'h': 0.2}]}
        car = self.run_route(world, [{'type': 'find', 'sensor': 'right_low', 'until': 'lost', 'mm': 425, 'tol': 100,
                                      'max': 5, 'speed': 50}], 60)
        self.assertAlmostEqual(car.x + .5 * car.wheelbase, 2000, delta=20)  # the side sensor is at the wall end

    def test_align_from_the_edge_to_the_checkpoint(self):
        world = {'objects': [{'type': 'start', 'x': 0, 'y': 0, 'yaw': 0}, {'type': 'checkpoint', 'x': 2500, 'y': 0, 'yaw': 0, 'label': 'C1'},
                             {'type': 'wall', 'pts': [[-500, -500], [2000, -500]], 'h': 0.2}]}
        find = {'type': 'find', 'sensor': 'right_low', 'until': 'lost', 'mm': 425, 'tol': 100, 'max': 5, 'speed': 50}
        m = dashboard.edge_to_checkpoint(SimCar(world=world), world['objects'][1], find)
        self.assertAlmostEqual(m, 2.5 - 2.0 + .15, delta=.01)  # side sensor 0.15 m ahead of the rear axle
        car = self.run_route(world, [find, {'type': 'align', 'label': 'C1', 'm': m, 'speed': 20}], 60)
        error = float(re.search(r'off by ([\d.]+) cm', ' '.join(car.route.log))[1])
        self.assertLess(error, 2)

    def test_find_holds_the_distance_to_the_wall(self):
        world = {'objects': [{'type': 'start', 'x': 0, 'y': 300, 'yaw': 0},
                             {'type': 'wall', 'pts': [[-500, -500], [6000, -500]], 'h': 0.2}]}
        find = {'type': 'find', 'sensor': 'right_low', 'until': 'lost', 'mm': 600, 'tol': 300, 'max': 10, 'speed': 40,
                'hold': 425, 'kp': .05}
        car = self.run_route(world, [find], 120)
        self.assertAlmostEqual(car.y, 0, delta=40)                          # 300 mm too far at the start
        self.assertAlmostEqual(car.x + .5 * car.wheelbase, 6000, delta=20)  # still stops at the end

    def test_find_seen_then_after_metres(self):
        world = {'objects': [{'type': 'start', 'x': 0, 'y': 0, 'yaw': 0},
                             {'type': 'wall', 'pts': [[1000, -500], [6000, -500]], 'h': 0.2}]}
        car = self.run_route(world, [{'type': 'find', 'sensor': 'right_low', 'until': 'seen', 'mm': 425, 'tol': 100,
                                      'max': 3, 'speed': 40, 'hold': 425, 'after': 2}], 60)
        self.assertAlmostEqual(car.x + .5 * car.wheelbase, 3000, delta=30)  # side sensor 2 m past the start of the wall

    def test_loop_starts_again_at_step_one(self):
        car = SimCar(world={'objects': [{'type': 'start', 'x': 0, 'y': 0, 'yaw': 0}]})
        car.route = dashboard.Route(dashboard.validate_route([{'type': 'drive', 'm': .3, 'speed': 30}]), loop=True)
        for _ in range(10):
            car.advance(1)
        self.assertTrue(car.route.running)
        self.assertGreaterEqual(car.route.lap, 3)
        self.assertIn('lap 1 done', ' '.join(car.route.log))

    BOX = {'type': 'box', 'x': 1500, 'y': -300, 'w': 300, 'd': 600, 'h': .3}

    def drive_past(self, box, seconds=150, remove_at=None, hook=None, max_offset=None):
        """Drive 4 m toward box; returns the car and how often a body point was inside the box."""
        car = SimCar(world={'objects': [{'type': 'start', 'x': 0, 'y': 0, 'yaw': 0}, box]})
        if max_offset:
            car.c['AVOID_MAX_OFFSET_MM'] = max_offset
        car.route = dashboard.Route(dashboard.validate_route([{'type': 'drive', 'm': 4, 'speed': 30}]))
        hits = 0
        for k in range(seconds * 20):
            car.advance(.05)
            if remove_at and 'stop and wait' in ' '.join(car.route.log) and k * .05 > remove_at:
                car.world = {'objects': [o for o in car.world['objects'] if o is not box]}
            if hook:
                hook(car)
            L, T, yaw = car.wheelbase, car.track, math.radians(car.yaw)
            for fx in (-.183 * L, .5 * L, 1.184 * L):
                for fy in (-.34 * T, 0, .34 * T):
                    px, py = car.x + fx * math.cos(yaw) - fy * math.sin(yaw), car.y + fx * math.sin(yaw) + fy * math.cos(yaw)
                    hits += box['x'] <= px <= box['x'] + box['w'] and box['y'] <= py <= box['y'] + box['d']
            if not car.route.running:
                break
        return car, hits

    def test_goes_around_a_persistent_obstacle(self):
        for box in (self.BOX, {**self.BOX, 'd': 1800}):  # 0.3 m and 1.5 m to the left of the line
            car, hits = self.drive_past(box)
            log = ' '.join(car.route.log)
            self.assertIn('route done', log)
            self.assertIn('back on the line', log)
            self.assertEqual(hits, 0, log)
            self.assertAlmostEqual(car.y, 0, delta=150)
            self.assertAlmostEqual(car.lib.wrap180(car.yaw), 0, delta=5)  # straight again, not still swerving

    def test_waits_for_a_temporary_obstacle(self):
        car, hits = self.drive_past(self.BOX, remove_at=5)
        log = ' '.join(car.route.log)
        self.assertIn('obstacle gone', log)
        self.assertNotIn('go around', log)
        self.assertAlmostEqual(car.x, 4000, delta=60)

    def test_new_obstacle_during_the_pass(self):
        for stays in (False, True):
            other = {'type': 'box', 'x': 2600, 'y': 300, 'w': 200, 'd': 600, 'h': .3}  # in the detour path

            def hook(car, t={}):
                log = ' '.join(car.route.log)
                if 'passing' in log and 'n' not in t:
                    t['n'] = car.now
                    car.world = {'objects': car.world['objects'] + [other]}
                if not stays and 'during the detour' in log and car.now - t['n'] > 3000:
                    car.world = {'objects': [o for o in car.world['objects'] if o is not other]}
            car, hits = self.drive_past(self.BOX, hook=hook)
            log = ' '.join(car.route.log)
            self.assertIn('during the detour: stop and wait', log)
            self.assertEqual(hits, 0)
            self.assertIn('route done', log)
            self.assertIn('go around both' if stays else 'continue the detour', log)
            self.assertAlmostEqual(car.y, 0, delta=150)  # back on the first line

    def test_obstacle_wider_than_max_offset_fails(self):
        car, hits = self.drive_past({**self.BOX, 'd': 1800}, max_offset=1000)
        self.assertIn('FAILED, obstacle needs more than 1 m', car.route.log[-1])
        self.assertEqual(hits, 0)

    def test_reading_is_corrected_for_the_car_angle(self):
        world = {'objects': [{'type': 'start', 'x': 0, 'y': 0, 'yaw': 0},
                             {'type': 'wall', 'pts': [[-5000, -1000], [5000, -1000]], 'h': 0.2}]}
        car = SimCar(world=world)
        route = car.route = dashboard.Route(dashboard.validate_route([{'type': 'find', 'sensor': 'right_low', 'until': 'seen',
                                              'mm': 3000, 'tol': 10, 'max': 1, 'speed': 10}]))
        car.advance(.001)                         # route heading = start heading
        car.yaw = 20.                             # the IMU sees the car turned 20 deg
        car.advance(.3)
        square = 1000 - .34 * car.track * math.cos(math.radians(20)) + .5 * car.wheelbase * math.sin(math.radians(20))  # turned left: the sensor moves away
        self.assertAlmostEqual(route.reading, square, delta=15)

    def test_run_from_a_checkpoint(self):
        with patch('dashboard.WORLD_FILE', Path(tempfile.mkdtemp()) / 'track.json'), \
             patch('dashboard.ROUTE_FILE', Path(tempfile.mkdtemp()) / 'route.json'):
            d = Dashboard()
            d.set_world({'objects': [{'type': 'start', 'x': 0, 'y': 0, 'yaw': 0},
                                     {'type': 'checkpoint', 'x': 5000, 'y': 2000, 'yaw': 90, 'label': 'C1'}]})
            d.set_route([{'type': 'drive', 'm': 9, 'speed': 30}, {'type': 'checkpoint', 'label': 'C1', 's': 0},
                         {'type': 'drive', 'm': .5, 'speed': 30}])
            with self.assertRaises(ValueError):
                d.run_route({'run': True, 'from': 0})        # not a checkpoint step
            d.run_route({'run': True, 'from': 1})
            car = d.robot.sim
            self.assertEqual(car.route.i, 2)
            for _ in range(20):
                car.advance(1)
            self.assertIn('route done', car.route.log[-1])
            self.assertAlmostEqual(car.x, 5000, delta=30)        # drove 0.5 m along the checkpoint heading
            self.assertAlmostEqual(car.y, 2500, delta=60)
            with self.assertRaises(ValueError):
                d.set_speed({'x': 3})
            d.set_speed({'x': 4})
            d.run_route({'run': True})
            state = {'drive': 'w', 'turn': '', 'speed': '5', 'stop': False, 'generation': d.generation}
            with self.assertRaises(ValueError):
                d.command(state)                            # no manual keys while a route runs
            d.command({**state, 'drive': '', 'stop': True})  # Stop always works
            self.assertEqual(d.snapshot()['sim_speed'], 4)

    def test_find_fails_after_max_and_validation(self):
        car = SimCar(world={'objects': [{'type': 'start', 'x': 0, 'y': 0, 'yaw': 0}]})
        car.route = dashboard.Route(dashboard.validate_route([{'type': 'find', 'sensor': 'front', 'until': 'seen', 'mm': 300, 'tol': 50, 'max': .3, 'speed': 30}]))
        for _ in range(20):
            car.advance(1)
        self.assertFalse(car.route.running)
        self.assertIn('FAILED', car.route.log[-1])
        for bad in [None, [{'type': 'jump'}], [{'type': 'drive', 'm': float('nan'), 'speed': 30}],
                    [{'type': 'drive', 'm': 1, 'speed': 5}], [{'type': 'drive', 'm': 1, 'speed': 25}], [{'type': 'find', 'sensor': 'front', 'until': 'seen', 'mm': 1, 'tol': 1, 'max': 1, 'speed': 10, 'hold': 300}], [{'type': 'find', 'sensor': 'left', 'until': 'seen', 'mm': 1, 'tol': 1, 'max': 1, 'speed': 10}]]:
            with self.assertRaises(ValueError):
                dashboard.validate_route(bad)


class ReplayTests(unittest.TestCase):
    """The same input sequence through the firmware side (Tests/replay.c: Bno085 struct,
    heading_hold_sample, control.c) and through the dashboard sim path (ctypes). The full
    controller state must be identical after each step."""

    @classmethod
    def setUpClass(cls):
        cls.exe = Path(tempfile.mkdtemp()) / ('replay.exe' if os.name == 'nt' else 'replay')
        sources = ['Tests/replay.c', 'App/Control/heading_hold.c', 'App/Control/control.c', 'App/Control/pid.c',
                   'App/Control/math_utils.c', str(Path(dashboard.__file__).with_name('control_binding.c'))]
        includes = ['-ITests/stubs', '-IApp/Control', '-IApp/Config', '-IApp/Sensors', '-IApp/Sensors/sh2']
        subprocess.run([dashboard.c_compiler(), '-std=c11', '-O2', *includes, *sources, '-lm', '-o', str(cls.exe)],
                       cwd=FIRMWARE, check=True)

    def sequence(self):
        """Keys, encoder samples with counter wrap and missed samples, heading reports with repeated,
        same-ms, late, wrapped and +-180 values, a stale period and a sensor reset."""
        rng = random.Random(7)
        seq, counts, yaw, t_imu, out = 0, [0xFFA0, 0x0050], 170., 0, ['k 53 0', 'k 119 0']
        for now in range(1, 3000):
            if now % 80 == 0:
                out.append(f'k {ord(rng.choice("wwwwasd 2"))} {now}')
            out.append(f't {now}')
            if now % 10 == 0:
                seq += 1 + (rng.random() < .05)
                counts = [(c + rng.randint(-20, 90)) & 0xFFFF for c in counts]
                out.append(f'w {seq} {counts[0]} {counts[1]} {now}')
            if now % 5 == 0 and not 1500 <= now < 1900 and rng.random() < .9:
                yaw = yaw + rng.uniform(-3, 5)  # drift: the heading goes around several times
                yaw = (yaw + 180) % 360 - 180 if rng.random() < .97 else 180.0
                t_imu = now - (rng.random() < .1)
                out.append(f'i {float(numpy_float32(yaw)).hex()} {t_imu} {rng.choice([1, 1, 2])}')
            if now % 20 == 0:
                out.append(f's {now}')
            if now == 700:
                out.append(f'z {float(numpy_float32(12.5)).hex()}')
            if now == 2200:
                out.append('r')
        return out

    def host(self, commands):
        """The dashboard path: SimCar's fake BNO085 state, validity check and ctypes calls."""
        car = SimCar()
        lib, ctl, rows = car.lib, car.ctl, []
        lib.control_init(ctl, True, 0)
        car.zero = 0.
        for line in commands:
            op, *a = line.split()
            if op == 'k': lib.control_key(ctl, bytes([int(a[0])]), int(a[1]))
            elif op == 't': lib.control_tick(ctl, int(a[0]))
            elif op == 'w': lib.control_wheels(ctl, *map(int, a))
            elif op == 'i':
                car.imu_raw, car.imu_t = float.fromhex(a[0]), int(a[1])
                car.imu_count += int(a[2])
            elif op == 's': car.steering(int(a[0]))
            elif op == 'z': car.zero, car.imu_reset_zero = float.fromhex(a[0]), car.imu_resets
            elif op == 'r': car.imu_resets += 1
            rows.append([float(v).hex() for v in car.lib.view(ctl).values()])
        return rows

    def test_firmware_and_dashboard_paths_match(self):
        commands = self.sequence()
        fw = subprocess.run([str(self.exe)], input='\n'.join(commands) + '\n', capture_output=True, text=True, check=True)
        firmware = [[float.fromhex(x).hex() for x in row.split()] for row in fw.stdout.splitlines()]
        host = self.host(commands)
        self.assertEqual(len(firmware), len(commands))
        for step, (a, b) in enumerate(zip(firmware, host)):
            if a != b:
                names = SimCar().lib.view_names
                diff = [(n, x, y) for n, x, y in zip(names, a, b) if x != y]
                self.fail(f'step {step} {commands[step]!r}: {diff}')
        # The sequence exercises the cases: missed samples, heading output, -180 error,
        # stale period and sensor reset (heading reference lost), repeated reports.
        names = SimCar().lib.view_names
        col = {n: [float.fromhex(r[names.index(n)]) for r in host] for n in names}
        self.assertGreater(max(col['overruns']), 0)
        self.assertGreater(max(map(abs, col['heading_out'])), 1)
        yaw = col['current_yaw']
        self.assertTrue(any(a > 150 and b < -150 or a < -150 and b > 150 for a, b in zip(yaw, yaw[1:])), 'no +-180 crossing')
        self.assertGreater(sum(1 for a, b in zip(col['have_prev'], col['have_prev'][1:]) if a and not b), 1)


def numpy_float32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


class ModeTests(unittest.TestCase):
    def test_sim_never_writes_serial_and_all_speed_digits(self):
        port = Port()
        robot = Robot(serial=port, demo=True)
        for digit in '1234567890':
            robot.command(dict(drive='w', turn='', speed=digit, stop=False))
            robot.sim.advance(.01)
            v = robot.sim.view()
            self.assertEqual((v['speed_pct'], v['drive_dir']), (100 if digit == '0' else int(digit)*10, 1))
        self.assertEqual(port.sent, '')

    def test_mode_switch_resets_and_rejects_old_commands(self):
        dashboard = Dashboard(port='fake', geometry=[100,300,220], placeholder=True)
        old = dashboard.snapshot()['generation']
        with patch('dashboard.Serial', return_value=Port()):
            dashboard.switch('live')
        self.assertEqual(dashboard.snapshot()['mode'], 'SERIAL')
        self.assertIsNone(dashboard.snapshot()['data'])
        with self.assertRaises(ValueError):
            dashboard.command(dict(drive='w',turn='',speed='1',stop=False,generation=old))
        port = dashboard.robot.serial
        dashboard.switch('sim')
        self.assertTrue(port.closed)
        self.assertEqual(port.sent, ' ')
        self.assertIsNone(dashboard.robot.serial)
        start = next(o for o in dashboard.world['objects'] if o['type'] == 'start')
        self.assertEqual(dashboard.snapshot()['pose'], [start['x'], start['y'], math.radians(start['yaw'])])  # back at the start

    def test_failed_live_connection_stays_explicit_and_can_return_to_sim(self):
        dashboard = Dashboard(port='missing')
        with patch('dashboard.Serial', side_effect=OSError(16, 'Resource busy')):
            dashboard.switch('live')
        state = dashboard.snapshot()
        self.assertEqual(state['mode'], 'SERIAL')
        self.assertTrue(state['stale'])
        self.assertIn('Resource busy', state['error'])
        dashboard.switch('sim')
        self.assertEqual(dashboard.snapshot()['error'], '')

if __name__ == '__main__': unittest.main()
