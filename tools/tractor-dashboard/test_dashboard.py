import math
import os
import pty
import time
import unittest
from unittest.mock import patch
from dashboard import Robot, Serial, Dashboard, parse_line

LINE = 'tgt -20 rpm -19 -21 cmd -10 -11 steer 5 g -12 r 13 rst 2 ov 4 STALL IMU-STALE\r\n'

class Port:
    def __init__(self): self.sent = ''
    def write(self, s): self.sent += s
    def lines(self): return []

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

    def test_serial_pty_fragmented_frames_and_commands(self):
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
            os.close(serial.fd); os.close(master); os.close(slave)

    def test_simulated_stop(self):
        r = Robot(demo=True, geometry=[100,300,220]); r.command(self.state('w'))
        now = r.last_input
        r.tick(now, .1)
        self.assertEqual(r.data['target'], 20)
        r.command(self.state(stop=True)); r.tick(now+.1, .1)
        self.assertEqual(r.data['target'], 0)
        self.assertEqual(r.snapshot()['mode'], 'SIMULATION')

class ModeTests(unittest.TestCase):
    def test_sim_never_writes_serial_and_all_speed_digits(self):
        port = Port()
        robot = Robot(serial=port, demo=True)
        for digit in '1234567890':
            robot.command(dict(drive='w', turn='', speed=digit, stop=False))
            self.assertEqual(robot.sim_goal, 200 if digit == '0' else int(digit)*20)
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
        dashboard.robot.serial.fd = 123
        port = dashboard.robot.serial
        with patch('dashboard.os.close') as close:
            dashboard.switch('sim')
        close.assert_called_once_with(123)
        self.assertEqual(port.sent, ' ')
        self.assertIsNone(dashboard.robot.serial)
        self.assertEqual(dashboard.snapshot()['pose'], [0,0,0])

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
