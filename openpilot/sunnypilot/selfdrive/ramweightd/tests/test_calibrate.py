"""Calibration helper tests on SYNTHETIC drives whose true constants differ from the estimator's starting values."""
import contextlib
import io
import math
import re
from functools import cache

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.ramweightd import calibrate
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import G, ROTATING_MASS_KG, WeightGradeEstimator
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
    assert abs(fitted) < 0.015 and abs(fitted - true) < 0.008, (fitted, true)
    assert abs(fitted) < abs(error())  # better than the shipped starting values


def grade_effect_ratio(pitch_slow_error: float, seed: int = 31) -> dict:
  """Fit force = grade + acceleration + constant + drag terms on five-second windows of a SYNTHETIC drive, with the
  grade from the Comma's pitch or from ESP_4; returns grade coefficient / acceleration coefficient (1 = physical)."""
  sim = TruckSim(Truck(body_pitch_per_accel=0.), Sensors(pitch_slow_error=pitch_slow_error), hilly, seed=seed).run(varied_schedule(3))
  ins = sim.inputs(seed=seed)
  v = np.array([i.v for i in ins])
  a = np.gradient(np.convolve(v, np.ones(10) / 10, mode="same"), 0.1)
  F = np.array([i.engine_torque_nm * i.engine_rpm * 2 * math.pi / 60 / max(i.v, 1.) for i in ins])
  grades = {"pitch": np.sin(np.array([i.pitch for i in ins])), "ESP_4": (np.array([i.stability_accel for i in ins]) - a) / G}
  ok = np.array([bool(i.converter_locked) and i.engine_torque_nm > 60 and i.v > 5 for i in ins])
  rows = []
  for i0 in range(0, len(v) - 50, 50):
    c = ok[i0:i0 + 50]
    if c.mean() >= 0.8:
      sl = slice(i0, i0 + 50)
      rows.append([F[sl][c].mean(), a[sl][c].mean(), grades["pitch"][sl][c].mean(), grades["ESP_4"][sl][c].mean(), (v[sl][c] ** 2).mean()])
  R = np.array(rows)
  m = sim.truck.mass_kg
  out = {}
  for k, name in ((2, "pitch"), (3, "ESP_4")):
    X = np.vstack([m * G * R[:, k], (m + ROTATING_MASS_KG) * R[:, 1], m * G * np.ones(len(R)), R[:, 4]]).T
    b, *_ = np.linalg.lstsq(X, R[:, 0], rcond=None)
    out[name] = b[0] / b[1]
  out["road_grade_std"] = float(np.std(sim.log.sine))
  return out


class TestRegressionDilution(OpenpilotTestCase):
  def test_slow_pitch_errors_shrink_the_grade_effect(self):
    # Eric's drives: the force followed only 0.17-0.33 of the Comma's pitch grade, but 0.84-0.91 of ESP_4's. With a true
    # scale of 1, a slow pitch error as large as the road's own grade variation gives the same picture: the fit shrinks
    # by about var(true) / (var(true) + var(error)) (regression dilution), while ESP_4 is unaffected.
    clean = grade_effect_ratio(0.)
    assert 0.9 < clean["pitch"] < 1.1 and 0.9 < clean["ESP_4"] < 1.1
    noisy = grade_effect_ratio(0.04)
    assert noisy["pitch"] < 0.45 and 0.9 < noisy["ESP_4"] < 1.1
    expected = noisy["road_grade_std"] ** 2 / (noisy["road_grade_std"] ** 2 + 0.04 ** 2)
    assert abs(noisy["pitch"] - expected) < 0.15
