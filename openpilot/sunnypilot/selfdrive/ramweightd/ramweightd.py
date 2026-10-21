#!/usr/bin/env python3
"""
ramweightd: the weight, air-drag and road-grade estimator service for Ram Heavy Duty trucks
(edfrahm/SP-RamHD-TICI-Port#8).

Runs only while onroad, and only on a Ram Heavy Duty (CarParams brand "chrysler", platform RAM_HD_5TH_GEN); the
process manager decides that (see openpilot/system/manager/process_config.py). While it has something to learn it
reads the truck's signals and the Comma's pose and steps the estimator ten times per second; once the weight has
settled it stops reading them and only watches for a qualifying stop, with an hourly fuel read (duty.py). It publishes
ramWeightGrade, which is logged, when the estimate changes and at least every duty.HEARTBEAT_S. Nothing subscribes to
it to steer, accelerate or brake: in this stage it changes no driving behavior.

Settings read (device settings, "params"):
- CruiseBaseWeightLb: starting weight in pounds with a full tank; 0 uses the published-weight table
- CruiseKnownExtraLb: the known added load on a validation drive; only published (knownExtraKg), never used by the
  estimator, so that accuracy = estimate - (starting weight + known extra) can be measured
- CruiseEmptyWeightLb: the truck's empty (sticker) weight; the published weight never goes below it minus the fuel not
  in the tank (massUnclampedKg keeps the estimate before that floor)
- RamWeightGradeState: the estimate kept between drives; written when the duty mode or the estimate's state changes,
  every minute while learning, and at the end of the drive
"""
import threading
import time

from opendbc.car.structs import car
from openpilot.cereal import log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.ramweightd.duty import SLEEP_CHECK_S, DutyCycle, Snapshot, asleep_check, listening_step
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import lb
from openpilot.sunnypilot.selfdrive.ramweightd.runner import STEP_HZ, WeightGradeRunner, is_ram_hd
from openpilot.sunnypilot.selfdrive.ramweightd.settings import MAX_BASE_WEIGHT_LB

SETTINGS_PERIOD_S = 5.
SAVE_PERIOD_S = 60.


def read_settings(params: Params) -> tuple[float, float, float]:
  """(starting weight, known extra load, empty weight) in pounds; 0 when not set."""
  def number(key):
    try:
      return min(max(float(params.get(key, return_default=True) or 0.), 0.), MAX_BASE_WEIGHT_LB)
    except (TypeError, ValueError):
      return 0.
  return number("CruiseBaseWeightLb"), number("CruiseKnownExtraLb"), number("CruiseEmptyWeightLb")


def save_state(params: Params, runner: WeightGradeRunner, block: bool = False) -> None:
  try:
    params.put("RamWeightGradeState", runner.state(), block=block)
  except Exception:
    cloudlog.exception("ramweightd: could not save RamWeightGradeState")


class Listening:
  """The truck's signals and the Comma's pose, read while the duty cycle listens. Dropping this closes the sockets."""

  def __init__(self):
    self.sm = messaging.SubMaster(["carState", "deviceMotion", "extrinsicsCalibration"])
    self.can_sock = messaging.sub_sock("can")
    self.accel_sock = messaging.sub_sock("accelerometer")

  def feed(self, runner: WeightGradeRunner) -> None:
    for raw in messaging.drain_sock_raw(self.can_sock):
      with log.Event.from_bytes(raw, traversal_limit_in_words=messaging.NO_TRAVERSAL_LIMIT) as evt:
        runner.handle_can_events([evt])
    for evt in messaging.drain_sock(self.accel_sock):
      runner.handle_accelerometer(evt.accelerometer)
    sm = self.sm
    sm.update(0)
    if sm.updated["carState"]:
      runner.handle_car_state(sm["carState"])
    if sm.updated["deviceMotion"]:
      runner.handle_device_motion(sm["deviceMotion"], sm.valid["deviceMotion"])
    if sm.updated["extrinsicsCalibration"]:
      runner.handle_extrinsics(sm["extrinsicsCalibration"])


def snapshot(runner: WeightGradeRunner, duty: DutyCycle) -> Snapshot:
  e = runner.estimator
  level = e.fuel.level_pct
  return Snapshot(duty.mode, round(lb(runner.published_mass_kg(e.mass_kg)[0])), round(lb(e.mass.mass_std)), e.qualifying_stops,
                  None if level is None else round(level, 1))


def main(exit_event: threading.Event | None = None) -> None:
  """exit_event: for tests; the process manager stops the service with SIGINT."""
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  if not is_ram_hd(CP):
    # the process manager only starts this service on a Ram Heavy Duty; idle quietly if started anyway
    cloudlog.warning("ramweightd: not a Ram Heavy Duty, nothing to do")
    while True:
      time.sleep(60.)

  base_lb, extra_lb, empty_lb = read_settings(params)
  persisted = params.get("RamWeightGradeState")
  runner = WeightGradeRunner(CP, base_weight_lb=base_lb, known_extra_lb=extra_lb, persisted=persisted, empty_weight_lb=empty_lb)
  cloudlog.info(f"ramweightd: configuration: {runner.configuration_text}; start {lb(runner.start.mass_kg):.0f} lb " +
                f"({runner.start.source}: {runner.start.detail}); floor: {runner.floor.detail}; kept estimate used: {runner.persisted_used}; " +
                f"signals not in the database: {', '.join(runner.reader.not_in_database)}")

  pm = messaging.PubMaster(["ramWeightGrade"])
  duty = DutyCycle(time.monotonic())
  listening: Listening | None = None
  car_sock = None  # carState, latest message only, while asleep
  rk = Ratekeeper(STEP_HZ, print_delay_threshold=None)
  last_settings = last_save = last_check = time.monotonic()

  try:
    while exit_event is None or not exit_event.is_set():
      now_nanos = time.monotonic_ns()
      t = now_nanos * 1e-9
      events: list[str] = []
      if duty.listening:
        if listening is None:
          listening, car_sock = Listening(), None
          rk = Ratekeeper(STEP_HZ, print_delay_threshold=None)
        listening.feed(runner)
        events += listening_step(runner, duty, now_nanos, 1. / STEP_HZ)
      else:
        if car_sock is None:
          listening, car_sock = None, messaging.sub_sock("carState", conflate=True)  # dropping Listening closes its sockets
        if t - last_check >= SLEEP_CHECK_S:
          last_check = t
          msg = messaging.recv_one_or_none(car_sock)
          if msg is not None:
            events += asleep_check(runner, duty, t, msg.carState)

      if runner.last_output is not None and duty.should_publish(t, snapshot(runner, duty)):
        msg = messaging.new_message("ramWeightGrade")
        msg.valid = True
        runner.fill(msg.ramWeightGrade, now_nanos)
        pm.send("ramWeightGrade", msg)

      for event in events:
        cloudlog.info(f"ramweightd: {event}")
      now = time.monotonic()
      if events or (duty.listening and now - last_save > SAVE_PERIOD_S):
        save_state(params, runner)
        last_save = now
      if now - last_settings > SETTINGS_PERIOD_S:
        runner.set_settings(*read_settings(params))
        last_settings = now
      if duty.listening:
        rk.keep_time()
      else:
        time.sleep(SLEEP_CHECK_S)
  finally:
    # the process manager stops services with SIGINT when the drive ends: keep the estimate for the next drive
    save_state(params, runner, block=True)


if __name__ == "__main__":
  main()
