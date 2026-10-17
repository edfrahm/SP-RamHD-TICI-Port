"""
End-to-end replay test on a SYNTHETIC rlog: tests/sim.py writes a simulated drive (known weight) as capnp Event
messages (initData, carParams, carState, can, deviceMotion, extrinsicsCalibration; in this openpilot version
deviceMotion and extrinsicsCalibration carry what older versions called livePose and liveCalibration), and replay.py
runs the estimator over it exactly as on the truck.
"""
import contextlib
import csv
import io
import os
import re
import tempfile
from collections import Counter
from functools import cache

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.ramweightd import replay
from openpilot.sunnypilot.selfdrive.ramweightd.tests.sim import Phase, Sensors, Truck, TruckSim, hilly, varied_schedule, write_pending_dbc
from openpilot.sunnypilot.selfdrive.ramweightd.tests.test_start_weight import vin


@cache
def synthetic_drive(pending: bool):
  d = tempfile.mkdtemp(prefix="ramweightd_replay_")
  seg = os.path.join(d, "0000abcd--0123456789--0")
  os.makedirs(seg)
  dbc = write_pending_dbc(d) if pending else "chrysler_ram_hd_generated"
  sched = varied_schedule(1) + [Phase("stop", 30, park=True, door=True)] + varied_schedule(1)[1:]
  sim = TruckSim(Truck(), Sensors(stability_sensor=pending, transmission=pending), hilly, seed=21).run(sched)
  path = os.path.join(seg, "rlog")
  sim.write_rlog(path, dbc_name=dbc, vin=vin(), car_fw=[("engine", b"52370231AK")], can_hz=20)
  return d, path, dbc, sim.true_mass_lb()


def run_replay(argv):
  buf = io.StringIO()
  with contextlib.redirect_stdout(buf):
    ret = replay.main(argv)
  return ret, buf.getvalue()


class TestReplay(OpenpilotTestCase):
  def test_synthetic_rlog_contents(self):
    from openpilot.tools.lib.logreader import LogReader
    _, path, _, _ = synthetic_drive(False)
    kinds = Counter(m.which() for m in LogReader(path, sources=[]))
    for k in ("initData", "carParams", "carState", "can", "deviceMotion", "extrinsicsCalibration"):
      assert kinds[k] > 0, k

  def check(self, pending: bool, tolerance_pct: float):
    d, path, dbc, true_lb = synthetic_drive(pending)
    csv_path = os.path.join(d, "estimate.csv")
    argv = [path, "--known-mass-lb", f"{true_lb:.0f}", "--body-pitch", "0.003", "--csv", csv_path, "--every", "120"]
    if pending:
      argv += ["--dbc", dbc]
    ret, text = run_replay(argv)
    assert ret == 0, text
    m = re.search(r"known weight (\d+) lb: error ([+-]\d+) lb \(([+-][\d.]+)%\), ([\d.]+) standard deviations", text)
    assert m, text
    assert abs(float(m.group(3))) < tolerance_pct, text
    assert float(m.group(4)) < 3., text
    assert "qualifying stop (park)" in text
    assert vin() not in text and vin()[-6:] not in text  # the VIN is never printed
    with open(csv_path) as f:
      rows = list(csv.DictReader(f))
    assert len(rows) > 5000
    assert int(rows[-1]["mass_updates"]) > 200
    return text

  def test_replay_with_todays_signals(self):
    text = self.check(False, 2.5)
    assert "grade from: pitchOnly" in text
    assert "converter rule: ratioFallback" in text
    assert "stability_accel_mps2" in text  # reported as not in the database

  def test_replay_with_pending_signals(self):
    text = self.check(True, 2.5)
    assert "grade from: sensorAndPitch" in text
    assert "converter rule: signal" in text

  def test_route_name_and_cli(self):
    d, _, _, _ = synthetic_drive(False)
    ret, text = run_replay(["0000abcd--0123456789", "--root", d, "--every", "100000"])
    assert ret == 0 and "final estimate:" in text and "known weight" not in text
