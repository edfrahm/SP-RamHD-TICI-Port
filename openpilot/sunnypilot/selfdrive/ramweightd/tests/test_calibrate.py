"""Calibration helper tests on SYNTHETIC drives whose true constants differ from the estimator's starting values."""
import contextlib
import io
import re
from functools import cache

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.ramweightd import calibrate
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import WeightGradeEstimator
from openpilot.sunnypilot.selfdrive.ramweightd.tests.sim import Sensors, Truck, TruckSim, hilly, varied_schedule
from openpilot.sunnypilot.selfdrive.ramweightd.tests.test_replay import synthetic_drive

TRUE = {"EFFICIENCY": 0.82, "ROLLING_RESISTANCE": 0.0105, "BODY_PITCH_PER_ACCEL": 0.004}


@cache
def calibration_windows():
  truck = Truck(efficiency=TRUE["EFFICIENCY"], rolling_resistance=TRUE["ROLLING_RESISTANCE"], body_pitch_per_accel=TRUE["BODY_PITCH_PER_ACCEL"])
  windows = []
  for seed in (31, 32):
    sim = TruckSim(truck, Sensors(stability_sensor=False, transmission=False), hilly, seed=seed).run(varied_schedule(3))
    est = WeightGradeEstimator(3500., 600., 32., 7.1, record_windows=True)  # the shipped constants, not the true ones
    for inp in sim.inputs(seed=seed):
      est.step(inp)
    windows += [(w, off) for w, off, _ in est.windows]
  return truck.mass_kg, windows


class TestCalibrate(OpenpilotTestCase):
  def test_recovers_the_true_constants(self):
    mass_kg, windows = calibration_windows()
    fit = calibrate.fit_constants(windows, mass_kg, body_pitch_used=0.)
    v = fit.values
    assert abs(v["EFFICIENCY"] - TRUE["EFFICIENCY"]) < 0.02, v
    assert abs(v["ROLLING_RESISTANCE"] - TRUE["ROLLING_RESISTANCE"]) < 0.0015, v
    assert abs(v["BODY_PITCH_PER_ACCEL"] - TRUE["BODY_PITCH_PER_ACCEL"]) < 0.0015, v
    assert abs(v["air drag c"] - 1.05) < 0.15, v
    assert fit.windows_used > 0.9 * fit.windows_total

  def test_grade_scale_check(self):
    # with the simulated body pitch corrected, the force follows all of the measured grade
    mass_kg, windows = calibration_windows()
    fit = calibrate.fit_constants(windows, mass_kg, body_pitch_used=0., fit_grade_scale=True)
    assert 0.85 < fit.values["grade scale"] < 1.15, fit.values

  def test_fixed_body_pitch(self):
    mass_kg, windows = calibration_windows()
    fit = calibrate.fit_constants(windows, mass_kg, body_pitch_used=0.004, fit_body_pitch=False)
    assert fit.values["BODY_PITCH_PER_ACCEL"] == 0.004
    assert len(fit.values) == 4

  def test_command_line_on_a_synthetic_rlog(self):
    _, path, _, true_lb = synthetic_drive(False)  # simulated with efficiency 0.85, rolling resistance 0.009, body pitch 0.003
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
      ret = calibrate.main([path, "--known-mass-lb", f"{true_lb:.0f}"])
    text = buf.getvalue()
    assert ret == 0, text
    eff = float(re.search(r"^EFFICIENCY = ([\d.]+)$", text, re.M).group(1))
    crr = float(re.search(r"^ROLLING_RESISTANCE = ([\d.]+)$", text, re.M).group(1))
    bp = float(re.search(r"^BODY_PITCH_PER_ACCEL = ([-\d.]+)$", text, re.M).group(1))
    assert abs(eff - 0.85) < 0.03 and abs(crr - 0.009) < 0.002 and abs(bp - 0.003) < 0.002, text

  def test_fitted_constants_work_on_another_drive(self):
    mass_kg, windows = calibration_windows()
    fit = calibrate.fit_constants(windows, mass_kg, body_pitch_used=0.).values
    truck = Truck(mass_kg=5000., efficiency=TRUE["EFFICIENCY"], rolling_resistance=TRUE["ROLLING_RESISTANCE"],
                  body_pitch_per_accel=TRUE["BODY_PITCH_PER_ACCEL"])
    sim = TruckSim(truck, Sensors(stability_sensor=False, transmission=False), hilly, seed=40).run(varied_schedule(3))

    def error(**constants):
      e = WeightGradeEstimator(3500., 600., 32., 7.1, **constants)
      out = [e.step(i) for i in sim.inputs(seed=40)][-1]
      return out.mass_kg / sim.log.mass_kg[-1] - 1.

    fitted = error(efficiency=fit["EFFICIENCY"], rolling_resistance=fit["ROLLING_RESISTANCE"], body_pitch_per_accel=fit["BODY_PITCH_PER_ACCEL"])
    true = error(efficiency=TRUE["EFFICIENCY"], rolling_resistance=TRUE["ROLLING_RESISTANCE"], body_pitch_per_accel=TRUE["BODY_PITCH_PER_ACCEL"])
    assert abs(fitted) < 0.015 and abs(fitted - true) < 0.005, (fitted, true)
    assert abs(fitted) < abs(error())  # better than the shipped starting values
