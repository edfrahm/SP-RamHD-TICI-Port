#!/usr/bin/env python3
"""
ramweightd: the weight, air-drag and road-grade estimator service for Ram Heavy Duty trucks
(edfrahm/SP-RamHD-TICI-Port#8).

Runs only while onroad, and only on a Ram Heavy Duty (CarParams brand "chrysler", platform RAM_HD_5TH_GEN); the
process manager decides that (see openpilot/system/manager/process_config.py). Ten times per second it reads the
truck's signals and the Comma's pose, steps the estimator and publishes ramWeightGrade, which is logged. Nothing
subscribes to it to steer, accelerate or brake: in this stage it changes no driving behavior.

Settings read (device settings, "params"):
- CruiseBaseWeightLb: starting weight in pounds with a full tank; 0 uses the published-weight table
- CruiseKnownExtraLb: the known added load on a validation drive; only published (knownExtraKg), never used by the
  estimator, so that accuracy = estimate - (starting weight + known extra) can be measured
- RamWeightGradeState: the estimate kept between drives; written at qualifying stops, every minute, and at the end
  of the drive
"""
import time

from opendbc.car.structs import car
from openpilot.cereal import log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import lb
from openpilot.sunnypilot.selfdrive.ramweightd.runner import STEP_HZ, WeightGradeRunner, is_ram_hd
from openpilot.sunnypilot.selfdrive.ramweightd.settings import MAX_BASE_WEIGHT_LB

SETTINGS_PERIOD_S = 5.
SAVE_PERIOD_S = 60.


def read_settings(params: Params) -> tuple[float, float]:
  def number(key):
    try:
      return min(max(float(params.get(key, return_default=True) or 0.), 0.), MAX_BASE_WEIGHT_LB)
    except (TypeError, ValueError):
      return 0.
  return number("CruiseBaseWeightLb"), number("CruiseKnownExtraLb")


def save_state(params: Params, runner: WeightGradeRunner, block: bool = False) -> None:
  try:
    params.put("RamWeightGradeState", runner.state(), block=block)
  except Exception:
    cloudlog.exception("ramweightd: could not save RamWeightGradeState")


def main() -> None:
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  if not is_ram_hd(CP):
    # the process manager only starts this service on a Ram Heavy Duty; idle quietly if started anyway
    cloudlog.warning("ramweightd: not a Ram Heavy Duty, nothing to do")
    while True:
      time.sleep(60.)

  base_lb, extra_lb = read_settings(params)
  persisted = params.get("RamWeightGradeState")
  runner = WeightGradeRunner(CP, base_weight_lb=base_lb, known_extra_lb=extra_lb, persisted=persisted)
  cloudlog.info(f"ramweightd: configuration: {runner.configuration_text}; start {lb(runner.start.mass_kg):.0f} lb " +
                f"({runner.start.source}: {runner.start.detail}); kept estimate used: {runner.persisted_used}; " +
                f"signals not in the database: {', '.join(runner.reader.not_in_database)}")

  pm = messaging.PubMaster(["ramWeightGrade"])
  sm = messaging.SubMaster(["carState", "deviceMotion", "extrinsicsCalibration"])
  can_sock = messaging.sub_sock("can")
  rk = Ratekeeper(STEP_HZ, print_delay_threshold=None)
  last_settings = last_save = time.monotonic()

  try:
    while True:
      for raw in messaging.drain_sock_raw(can_sock):
        with log.Event.from_bytes(raw, traversal_limit_in_words=messaging.NO_TRAVERSAL_LIMIT) as evt:
          runner.handle_can_events([evt])
      sm.update(0)
      if sm.updated["carState"]:
        runner.handle_car_state(sm["carState"])
      if sm.updated["deviceMotion"]:
        runner.handle_device_motion(sm["deviceMotion"], sm.valid["deviceMotion"])
      if sm.updated["extrinsicsCalibration"]:
        runner.handle_extrinsics(sm["extrinsicsCalibration"])

      now_nanos = time.monotonic_ns()
      out = runner.step(now_nanos)
      msg = messaging.new_message("ramWeightGrade")
      msg.valid = True
      runner.fill(msg.ramWeightGrade, now_nanos)
      pm.send("ramWeightGrade", msg)

      for event in out.events:
        cloudlog.info(f"ramweightd: {event}")
      now = time.monotonic()
      if out.events or now - last_save > SAVE_PERIOD_S:
        save_state(params, runner)
        last_save = now
      if now - last_settings > SETTINGS_PERIOD_S:
        runner.set_settings(*read_settings(params))
        last_settings = now
      rk.keep_time()
  finally:
    # the process manager stops services with SIGINT when the drive ends: keep the estimate for the next drive
    save_state(params, runner, block=True)


if __name__ == "__main__":
  main()
