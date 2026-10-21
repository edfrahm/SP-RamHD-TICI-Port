"""
The duty cycle (duty.py): learn until the weight settles, then stop reading the truck, wake for a qualifying stop and an
hourly fuel read, publish only on change. Unit tests on made-up numbers, plus a replay of the SYNTHETIC drive of
test_replay.py with the duty cycle.
"""
import contextlib
import io
import re
import threading
import time
from unittest import mock

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.ramweightd import duty as D
from openpilot.sunnypilot.selfdrive.ramweightd import ramweightd as service
from openpilot.sunnypilot.selfdrive.ramweightd import replay
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import LB_TO_KG, QUALIFYING_STOP_S, Inputs, WeightGradeEstimator
from openpilot.sunnypilot.selfdrive.ramweightd.tests.test_replay import synthetic_drive
from openpilot.sunnypilot.selfdrive.ramweightd.tests.test_start_weight import car_params

MASS = 8300. * LB_TO_KG
SETTLED_STD = 0.05 * MASS
WIDE_STD = 0.5 * MASS


def learn(cycle: D.DutyCycle, t: float, seconds: float, std: float, updates_from: int = 0, moving: bool = True,
          updates_per_s: float = 0.3) -> tuple[float, list[str]]:
  """Steps at 10 Hz while listening; returns (time, events)."""
  events: list[str] = []
  for i in range(int(round(seconds * 10))):
    t += 0.1
    events += cycle.after_step(t, 0.1, moving, MASS, std, updates_from + int(i * 0.1 * updates_per_s))
    if not cycle.listening:
      break
  return t, events


def car_state(v=20., gear="drive", door=False):
  msg = messaging.new_message("carState")
  msg.carState.vEgoRaw = v
  msg.carState.gearShifter = gear
  msg.carState.doorOpen = door
  return msg.carState


def watch(cycle: D.DutyCycle, t: float, seconds: float, v=20., gear="drive", door=False) -> tuple[float, str | None, list[str]]:
  """carState once a second while asleep; stops at the first qualifying stop or mode change."""
  events: list[str] = []
  for _ in range(int(seconds)):
    t += 1.
    why, changes = cycle.check_asleep(t, v, gear, door)
    events += changes
    if why is not None or cycle.listening:
      return t, why, events
  return t, None, events


def drive_inputs(t: float, v: float = 20., gear: str = "drive", door: bool = False) -> Inputs:
  return Inputs(t=t, v=v, gear=gear, door_open=door, brake_pedal=False, brake_pressure=0., accel_pedal_pct=0., acc_active=False,
                acc_torque_request_nm=0., engine_rpm=None, engine_torque_nm=None, rear_wheel_rpm=None, traction_control=None,
                stability_accel=None, comma_accel=None, converter_locked=None, shift_pending=None, exhaust_brake=None,
                trailer_connected=None, fuel_level_pct=None, pitch=None, pitch_std=None, lateral_accel=None)


class TestDutyCycle(OpenpilotTestCase):
  def test_learns_until_settled_then_sleeps(self):
    c = D.DutyCycle(0.)
    assert c.mode == D.LEARN and c.listening
    t, events = learn(c, 0., 120., WIDE_STD)
    assert c.mode == D.LEARN and not events  # not settled: keeps listening
    t, events = learn(c, t, 120., SETTLED_STD, updates_from=100)
    assert c.mode == D.SLEEP and not c.listening and c.settled
    assert any("settled at" in e for e in events), events

  def test_needs_updates_in_this_period(self):
    # a narrow estimate is not "settled" until this learning period has made MIN_UPDATES updates of its own
    c = D.DutyCycle(0.)
    t, _ = learn(c, 0., 10., SETTLED_STD, updates_from=500, updates_per_s=0.)
    assert c.mode == D.LEARN
    t, _ = learn(c, t, (D.MIN_UPDATES + 1) / 0.3 + 1., SETTLED_STD, updates_from=500)
    assert c.mode == D.SLEEP

  def test_learning_limit_counts_driving_only(self):
    c = D.DutyCycle(0.)
    t, _ = learn(c, 0., D.LEARN_LIMIT_S, WIDE_STD, moving=False)  # stopped time does not count
    assert c.mode == D.LEARN
    t, events = learn(c, t, D.LEARN_LIMIT_S + 1., WIDE_STD)
    assert c.mode == D.SLEEP and not c.settled
    assert any("not settled after" in e for e in events), events

  def settled_asleep(self) -> tuple[D.DutyCycle, float]:
    c = D.DutyCycle(0.)
    t, _ = learn(c, 0., 300., SETTLED_STD, updates_from=100)
    assert c.mode == D.SLEEP
    return c, t

  def test_qualifying_stops_wake_it(self):
    for kwargs, reason in (({"gear": "park", "v": 0.}, "park"), ({"door": True, "v": 0.}, "door opened"),
                           ({"v": 0.}, f"stopped {QUALIFYING_STOP_S:.0f} s")):
      c, t = self.settled_asleep()
      t, why, events = watch(c, t, QUALIFYING_STOP_S + 5, **kwargs)
      assert why == reason and c.mode == D.LEARN and not c.settled, (kwargs, why, events)

  def test_short_stops_and_driving_keep_it_asleep(self):
    c, t = self.settled_asleep()
    for _ in range(5):
      t, why, _ = watch(c, t, 60., v=0.)  # a red light
      assert why is None
      t, why, _ = watch(c, t, 60.)
      assert why is None
    assert c.mode == D.SLEEP

  def test_hourly_fuel_read(self):
    c, t = self.settled_asleep()
    t, why, events = watch(c, t, D.FUEL_PERIOD_S + 5)
    assert why is None and c.mode == D.FUEL and c.listening, events
    t, events = learn(c, t, D.FUEL_READ_S + 1., SETTLED_STD, updates_from=200, updates_per_s=0.)
    assert c.mode == D.SLEEP and any("fuel read" in e for e in events), events
    # the next one an hour later
    t, _, _ = watch(c, t, D.FUEL_PERIOD_S - 60.)
    assert c.mode == D.SLEEP
    t, _, _ = watch(c, t, 120.)
    assert c.mode == D.FUEL

  def test_fuel_read_resumes_learning_when_not_settled(self):
    c = D.DutyCycle(0.)
    t, _ = learn(c, 0., D.LEARN_LIMIT_S + 1., WIDE_STD)
    assert c.mode == D.SLEEP and not c.settled
    t, _, _ = watch(c, t, D.FUEL_PERIOD_S + 5)
    t, events = learn(c, t, D.FUEL_READ_S + 1., WIDE_STD)
    assert c.mode == D.LEARN, events

  def test_publishes_on_change_and_heartbeat(self):
    c = D.DutyCycle(0.)
    s = D.Snapshot(D.SLEEP, 9300, 900, 1, 50.2)
    assert c.should_publish(0., s)
    assert not c.should_publish(1., s)
    assert not c.should_publish(D.HEARTBEAT_S - 0.5, s)
    assert c.should_publish(D.HEARTBEAT_S + 0.1, s)  # heartbeat
    assert c.should_publish(D.HEARTBEAT_S + 0.2, D.Snapshot(D.SLEEP, 9300, 900, 1, 50.0))  # fuel moved
    assert c.should_publish(D.HEARTBEAT_S + 0.3, D.Snapshot(D.LEARN, 9300, 900, 1, 50.0))  # mode changed


class TestQualifyingStopWhileAsleep(OpenpilotTestCase):
  def test_counted_once_and_widened(self):
    e = WeightGradeEstimator(MASS, 300., 31., 7.079)
    e.mass.P[0][0] = 300. ** 2  # as if learned
    t = 0.
    for _ in range(20):
      t += 0.1
      e.step(drive_inputs(t))
    events = e.note_qualifying_stop("park", 3.)
    assert e.qualifying_stops == 1 and e.mass.mass_std > 1000. and events
    # the next steps continue the same stop (after a gap, as when the service wakes up): not counted again
    t += 5.
    for _ in range(30):
      t += 0.1
      e.step(drive_inputs(t, v=0., gear="park"))
    assert e.qualifying_stops == 1
    # driving away and parking again is a new stop
    for _ in range(10):
      t += 0.1
      e.step(drive_inputs(t))
    t += 0.1
    e.step(drive_inputs(t, v=0., gear="park"))
    assert e.qualifying_stops == 2


class TestRunnerWake(OpenpilotTestCase):
  def test_wake_keeps_only_the_new_car_state(self):
    from openpilot.sunnypilot.selfdrive.ramweightd.tests.test_ram_signals import TestRunner
    r = TestRunner().runner()
    r.handle_car_state(car_state(v=25.))
    r.handle_accelerometer(messaging.new_message("accelerometer").accelerometer)
    r.accel_sum, r.accel_count = [1., 2., 3.], 4
    parked = car_state(v=0., gear="park")
    r.wake(parked)
    assert r.car_state is parked and r.accel_count == 0 and r.device_motion is None


class TestReplayWithDutyCycle(OpenpilotTestCase):
  def test_synthetic_drive(self):
    _, path, _, true_lb = synthetic_drive(True)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
      ret = replay.main([path, "--duty", "--known-mass-lb", f"{true_lb:.0f}", "--body-pitch", "0.003", "--every", "100000"])
    text = buf.getvalue()
    assert ret == 0, text
    # settles in the first part, sleeps, wakes at the park-and-door stop in the middle and learns again
    assert re.search(r"learn -> sleep: settled at", text), text
    assert "sleep -> learn: qualifying stop (park)" in text, text
    assert text.count("qualifying stop (park): weight kept") == 1, text  # the middle stop, once
    assert re.search(r"learn -> sleep: settled at.*\n(.*\n)*.*sleep -> learn.*\n(.*\n)*.*learn -> sleep: settled at", text), text
    m = re.search(r"duty cycle: listened ([\d.]+) min", text)
    drive_min = (2 * 425. + 30.) / 60.
    assert m and float(m.group(1)) < 0.8 * drive_min, text
    m = re.search(r"known weight (\d+) lb: error ([+-]\d+) lb \(([+-][\d.]+)%\)", text)
    assert m and abs(float(m.group(3))) < 10., text  # settled means within about 10 percent


class TestServiceLoop(OpenpilotTestCase):
  """The service itself over real messaging, with the duty cycle's times shortened and "settled" forced: it must stop
  publishing ten times a second once asleep (heartbeats only), and count one park as one qualifying stop."""

  def test_sleeps_and_wakes_once_per_stop(self):
    Params().put("CarParams", car_params().to_bytes(), block=True)
    pm = messaging.PubMaster(["carState", "can"])
    sub = messaging.sub_sock("ramWeightGrade", timeout=50)
    stop = threading.Event()
    received: list[tuple[float, int]] = []

    def run(seconds: float, gear: str, v: float, until_received: bool = False) -> None:
      end = time.monotonic() + seconds
      while time.monotonic() < end and not (until_received and received):
        cs = messaging.new_message("carState")
        cs.carState.vEgoRaw, cs.carState.gearShifter = v, gear
        pm.send("carState", cs)
        pm.send("can", messaging.new_message("can", 0))
        time.sleep(0.01)
        while (m := messaging.recv_one_or_none(sub)) is not None:
          received.append((time.monotonic(), m.ramWeightGrade.qualifyingStops))

    with mock.patch.object(D, "SETTLE_FRACTION", 100.), mock.patch.object(D, "MIN_UPDATES", 0), \
         mock.patch.object(D, "HEARTBEAT_S", 0.5), mock.patch.object(service, "SLEEP_CHECK_S", 0.05):
      thread = threading.Thread(target=service.main, args=(stop,), daemon=True)
      thread.start()
      try:
        run(15., "drive", 20., until_received=True)  # the service starting up (and learning, then settling)
        run(0.3, "drive", 20.)
        received.clear()
        run(3., "drive", 20.)
        driving = list(received)
        received.clear()
        run(2., "park", 0.)
      finally:
        stop.set()
        thread.join(timeout=5.)
    assert not thread.is_alive()
    # asleep while driving: heartbeats about every 0.5 s, not ten times a second
    assert 3 <= len(driving) <= 8, driving
    assert all(b[0] - a[0] > 0.3 for a, b in zip(driving, driving[1:], strict=False)), driving
    assert all(q == 0 for _, q in driving)
    # the park: one qualifying stop, not one per check
    assert received and max(q for _, q in received) == 1, received
