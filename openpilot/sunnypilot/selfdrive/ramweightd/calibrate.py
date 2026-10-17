#!/usr/bin/env python3
"""
Fit the weight estimator's constants from calibration drives with a known combined weight.

  calibrate.py <route | segment folder | rlog file> ... --known-mass-lb 8400 [--root DIR] [--no-body-pitch]

Each route is one drive; give the combined weight at the start of the drives (truck, people, load, fuel). Drive
varied speeds and grades with strong accelerations and long climbs, and do not refuel during a drive.

It replays the drives exactly like replay.py, collects every clean one-second window, and fits by least squares
(trimming outliers beyond three standard deviations, repeatedly), with the weight known:
  efficiency * engine force - rolling resistance * m*g*cos(grade) - c * speed^2 + body pitch * m*g*a
      = (m + rotating parts) * a + m*g*sine(grade)
- EFFICIENCY and ROLLING_RESISTANCE: the constants the estimator uses.
- BODY_PITCH_PER_ACCEL: how much the body pitches nose-up per m/s2 of acceleration, which tilts the grade sensors. It
  also absorbs any error of the rotating-parts figure (both act in proportion to the acceleration).
- c (air drag) is fitted too, for information only: the estimator learns it on every drive.
It prints the values to put at the top of estimator.py. Check them with replay.py --efficiency ... --rolling-resistance
... --body-pitch ... on other drives with a known weight.
"""
import argparse
import math
import sys
from dataclasses import dataclass

import numpy as np

from openpilot.sunnypilot.selfdrive.ramweightd import estimator as est
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import G, LB_TO_KG, ROTATING_MASS_KG, Window
from openpilot.sunnypilot.selfdrive.ramweightd.replay import add_estimator_arguments, find_log_files, replay, runner_factory

MIN_WINDOWS = 200
NAMES = ("EFFICIENCY", "ROLLING_RESISTANCE", "air drag c", "BODY_PITCH_PER_ACCEL")


@dataclass
class Fit:
  values: dict[str, float]
  stds: dict[str, float]
  correlations: np.ndarray
  windows_used: int
  windows_total: int
  residual_rms_n: float


def fit_constants(windows: list[tuple[Window, float]], known_mass_kg: float, body_pitch_used: float, fit_body_pitch: bool = True) -> Fit:
  """windows: (window, fuel burned since the start of its drive in kg). body_pitch_used: the value the windows' grade
  was corrected with."""
  X, y = [], []
  for w, burned in windows:
    m = known_mass_kg - burned
    row = [w.engine_force, -m * G * w.cosine, -w.v2]
    if fit_body_pitch:
      row.append(m * G * w.accel)
    X.append(row)
    y.append((m + ROTATING_MASS_KG) * w.accel + m * G * w.sine)
  X_all, y_all = np.array(X), np.array(y)
  keep = np.ones(len(y_all), dtype=bool)
  for _ in range(10):
    theta, *_ = np.linalg.lstsq(X_all[keep], y_all[keep], rcond=None)
    resid = y_all - X_all @ theta
    sigma = np.std(resid[keep])
    new_keep = np.abs(resid) < 3. * sigma
    if np.array_equal(new_keep, keep):
      break
    keep = new_keep
  Xk = X_all[keep]
  dof = max(int(keep.sum()) - Xk.shape[1], 1)
  sigma2 = float(np.sum((y_all[keep] - Xk @ theta) ** 2) / dof)
  cov = sigma2 * np.linalg.inv(Xk.T @ Xk)
  std = np.sqrt(np.diag(cov))
  corr = cov / np.outer(std, std)
  names = NAMES if fit_body_pitch else NAMES[:3]
  values = dict(zip(names, (float(x) for x in theta), strict=True))
  stds = dict(zip(names, (float(x) for x in std), strict=True))
  if fit_body_pitch:
    values["BODY_PITCH_PER_ACCEL"] += body_pitch_used
  else:
    values["BODY_PITCH_PER_ACCEL"], stds["BODY_PITCH_PER_ACCEL"] = body_pitch_used, 0.
  return Fit(values, stds, corr, int(keep.sum()), len(y_all), math.sqrt(sigma2))


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  add_estimator_arguments(parser)
  parser.add_argument("--known-mass-lb", type=float, required=True, help="combined weight at the start of each drive")
  parser.add_argument("--no-body-pitch", action="store_true", help="keep BODY_PITCH_PER_ACCEL as given by --body-pitch")
  args = parser.parse_args(argv)

  windows: list[tuple[Window, float]] = []
  for item in args.logs:
    files = find_log_files([item], args.root)
    runner = replay(files, runner_factory(args, record_windows=True))
    if runner is None:
      print(f"{item}: no carParams, skipped")
      continue
    # fuel burned since the start of the drive (no refueling during calibration drives)
    drive = [(w, offset) for w, offset, _ in runner.estimator.windows]
    windows += drive
    print(f"{item}: {len(files)} file(s), {len(drive)} clean one-second windows")

  if len(windows) < MIN_WINDOWS:
    print(f"only {len(windows)} clean windows: need at least {MIN_WINDOWS}. Drive longer, with more varied speeds and grades.")
    return 1

  fit = fit_constants(windows, args.known_mass_lb * LB_TO_KG, args.body_pitch, not args.no_body_pitch)
  print()
  print(f"fitted on {fit.windows_used} of {fit.windows_total} windows (the rest beyond three standard deviations); " +
        f"residual {fit.residual_rms_n:.0f} N rms")
  print("statistical uncertainty below is optimistic: errors of neighboring windows are linked")
  for k, v in fit.values.items():
    print(f"  {k:22s} {v:10.5f}  +/- {fit.stds[k]:.5f}")
  names = list(fit.stds)[:fit.correlations.shape[0]]
  worst = max(((abs(fit.correlations[i, j]), names[i], names[j]) for i in range(len(names)) for j in range(i)), default=(0., "", ""))
  if worst[0] > 0.95:
    print(f"warning: {worst[1]} and {worst[2]} are {worst[0]:.2f} correlated: the drives do not separate them well. Add drives " +
          "with other speeds, grades and accelerations.")
  v = fit.values
  ok = 0.6 < v["EFFICIENCY"] < 1.0 and 0.004 < v["ROLLING_RESISTANCE"] < 0.02 and abs(v["BODY_PITCH_PER_ACCEL"]) < 0.02
  if not ok:
    print("warning: these values are outside the physically plausible range; check the known weight and the drives")
  print()
  print("Put these at the top of openpilot/sunnypilot/selfdrive/ramweightd/estimator.py:")
  print(f"EFFICIENCY = {v['EFFICIENCY']:.3f}")
  print(f"ROLLING_RESISTANCE = {v['ROLLING_RESISTANCE']:.4f}")
  print(f"BODY_PITCH_PER_ACCEL = {v['BODY_PITCH_PER_ACCEL']:.4f}")
  print(f"(air drag on these drives: {v['air drag c']:.2f} N/(m/s)^2; the estimator learns it each drive. " +
        f"Currently EFFICIENCY = {est.EFFICIENCY}, ROLLING_RESISTANCE = {est.ROLLING_RESISTANCE}, " +
        f"BODY_PITCH_PER_ACCEL = {est.BODY_PITCH_PER_ACCEL}.)")
  return 0


if __name__ == "__main__":
  sys.exit(main())
