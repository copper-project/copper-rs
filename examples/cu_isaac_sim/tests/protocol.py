"""Exercise the native Copper graph with real Python mmap writes and no Isaac/GPU."""

import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "isaac"))
from transport import Connection

BINARY = sys.argv.pop(1)


class ProtocolTest(unittest.TestCase):
    @staticmethod
    def stop_server(process):
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=15)

    def start_server(self, directory, override=False):
        session = Path(directory).resolve()
        log = session / "test.copper"
        args = [BINARY, "--session", str(session), "--log", str(log)]
        if override:
            args += ["--override-controller=stop"]
        process = subprocess.Popen(args, env=dict(os.environ, TMPDIR=str(session)), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.addCleanup(self.stop_server, process)
        connection = Connection(session, timeout=15.0)
        self.addCleanup(connection.close)
        return process, connection

    def test_layout_clock_graph_and_override(self):
        for override, count, requested in ((False, 24, 24), (True, 24, 24), (False, 140, 0), (True, 0, 0)):
            with self.subTest(override=override, requested=requested, count=count), tempfile.TemporaryDirectory(prefix="cu-isaac-test-") as directory:
                process, connection = self.start_server(directory, override)
                # Recycle CopperLists and cross keyframes, including past the old demo limit.
                connection.send({"type": "hello", "version": 1, "observations": requested, "scenario": {"test": True}})
                for sequence in range(count):
                    lease = connection.receive("lease")
                    self.assertEqual(lease["sequence"], sequence)
                    self.assertEqual(lease["point_bytes"], 20)
                    camera, offset = connection.buffer(lease["camera"], 320 * 240 * 3)
                    camera[offset:offset + 320 * 240 * 3] = bytes([sequence, 20, 40]) * (320 * 240)
                    lidar, offset = connection.buffer(lease["lidar"], lease["max_points"] * 20)
                    distance = 2.0 if sequence < count // 2 else 0.5
                    # World +X is robot -X. This checks conversion before the real clearance task.
                    struct.pack_into("<ffffI", lidar, offset, 7.5 - distance, -1.5, 0.3, 0.5, 99_000_000)
                    now = (sequence + 2) * 100_000_000
                    connection.send({"type": "frame", "sequence": sequence, "simulation_time_ns": now,
                                     "camera_time_ns": now, "scan_start_ns": now - 100_000_000,
                                     "scan_end_ns": now - 1_000_000, "points": 1,
                                     "origin_world_m": [7.5, -1.5, 0.3],
                                     "axes_world": [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]})
                    try:
                        command = connection.receive("command")
                    except RuntimeError:
                        stdout, stderr = process.communicate(timeout=15)
                        self.fail((stdout + stderr).decode())
                    self.assertEqual(command["sequence"], sequence)
                    self.assertEqual(command["simulation_time_ns"], now)
                    self.assertAlmostEqual(command["clearance_m"], distance)
                    self.assertEqual(command["mean_rgb"], [sequence, 20.0, 40.0])
                    self.assertEqual(command["left_rad_s"], 0.0 if override or distance < 0.65 else 5.0)
                    self.assertEqual(command["right_rad_s"], command["left_rad_s"])
                if requested == 0:
                    self.assertEqual(connection.receive("lease")["sequence"], count)
                connection.send({"type": "finish"})
                try:
                    finished = connection.receive("finished")
                except RuntimeError:
                    stdout, stderr = process.communicate(timeout=15)
                    self.fail((stdout + stderr).decode())
                self.assertEqual(finished["observations"], count)
                self.assertEqual(finished["log"], str(Path(directory).resolve() / "test.copper"))
                stdout, stderr = process.communicate(timeout=15)
                self.assertEqual(process.returncode, 0, (stdout + stderr).decode())
                self.assertTrue(list(Path(directory).glob("test_*.copper")))

    def test_disconnect_and_invalid_publication_abort(self):
        for invalid in (False, True):
            with self.subTest(invalid=invalid), tempfile.TemporaryDirectory(prefix="cu-isaac-test-") as directory:
                process, connection = self.start_server(directory)
                connection.send({"type": "hello", "version": 1, "observations": 1, "scenario": {}})
                connection.receive("lease")
                if invalid:
                    connection.send({"type": "frame", "sequence": 99, "simulation_time_ns": 200_000_000,
                                     "camera_time_ns": 200_000_000, "scan_start_ns": 100_000_000,
                                     "scan_end_ns": 199_000_000, "points": 1,
                                     "origin_world_m": [0, 0, 0], "axes_world": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]})
                else:
                    connection.close()
                stdout, stderr = process.communicate(timeout=15)
                self.assertNotEqual(process.returncode, 0, (stdout + stderr).decode())


if __name__ == "__main__":
    unittest.main()
