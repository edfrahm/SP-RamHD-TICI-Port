#!/usr/bin/env python3
"""
Replay a recorded drive through the weight and grade estimator, offline, exactly as ramweightd runs it on the truck.

  replay.py <route | segment folder | rlog file> ... [--known-mass-lb N] [--base-weight-lb N] [--every S] [--csv FILE]

Examples:
  on the Mac:     python openpilot/sunnypilot/selfdrive/ramweightd/replay.py 00000779--9043655153 --root ~/src/ram-hd-c3/rlogs
  on the comma 3: cd /data/openpilot && PYTHONPATH=/data/openpilot /usr/local/venv/bin/python \\
                    openpilot/sunnypilot/selfdrive/ramweightd/replay.py 00000779--9043655153 --known-mass-lb 8400

A route name reads every segment <root>/<route>--<n>/rlog* in order (default root: the comma 3's drive folder
/data/media/0/realdata, else ~/src/ram-hd-c3/rlogs). It prints the estimate over time and, with --known-mass-lb,
the final accuracy. The device settings are not read: give --base-weight-lb to replay with a starting-weight
setting (default 0: the published-weight table, or the platform weight). The VIN is never printed.
"""
import argparse
import csv
import glob
import os
import re
import sys
from collections import Counter

from openpilot.sunnypilot.selfdrive.ramweightd import estimator as est
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import lb
from openpilot.sunnypilot.selfdrive.ramweightd.runner import STEP_NANOS, WeightGradeRunner

DEVICE_ROOT = "/data/media/0/realdata"
MAC_ROOT = os.path.expanduser("~/src/ram-hd-c3/rlogs")
WANTED = ("carParams", "carState", "can", "deviceMotion", "extrinsicsCalibration")
MPS_TO_MPH = 2.2369363


def _segment_number(path: str) -> int:
  m = re.search(r"--(\d+)(?:/|$)", path)
  return int(m.group(1)) if m else 0


def find_log_files(items: list[str], root: str | None = None) -> list[str]:
  """Route names, segment folders or rlog files -> rlog files in drive order."""
  roots = [root] if root else [r for r in (DEVICE_ROOT, MAC_ROOT) if os.path.isdir(r)]
  files: list[str] = []
  for item in items:
    item = os.path.expanduser(item)
    if os.path.isfile(item):
      files.append(item)
    elif os.path.isdir(item):
      found = sorted(glob.glob(os.path.join(item, "rlog*")))
      if found:
        files.append(found[0])
      else:  # a folder of segment folders
        segs = sorted(glob.glob(os.path.join(item, "*--*", "rlog*")), key=_segment_number)
        files.extend(segs)
    else:
      segs = []
      for r in roots:
        segs = sorted(glob.glob(os.path.join(r, f"{item}--*", "rlog*")), key=_segment_number)
        if segs:
          break
      if not segs:
        raise FileNotFoundError(f"no rlog files for {item!r} under {roots}")
      # one file per segment (prefer rlog.zst if both exist)
      by_seg: dict[int, str] = {}
      for f in segs:
        by_seg.setdefault(_segment_number(f), f)
      files.extend(by_seg[k] for k in sorted(by_seg))
  return files


def iterate_events(files: list[str]):
  from openpilot.tools.lib.logreader import LogReader
  for f in files:
    yield from LogReader(f, sources=[])


def replay(files: list[str], make_runner, on_step=None, on_event=None) -> WeightGradeRunner | None:
  """Feed the logs to a runner built from the first carParams; step it ten times per second of log time."""
  runner = None
  next_step = None
  for evt in iterate_events(files):
    which = evt.which()
    if which not in WANTED:
      continue
    t = evt.logMonoTime
    if runner is None:
      if which == "carParams":
        runner = make_runner(evt.carParams)
        next_step = t
      continue
    if which == "carParams":
      continue
    if t - next_step > 1_000_000_000:  # a gap (missing segment): restart the clock
      next_step = t
    while t >= next_step:
      out = runner.step(next_step)
      if on_step is not None:
        on_step(runner, next_step, out)
      for e in out.events:
        if on_event is not None:
          on_event(runner, next_step, e)
      next_step += STEP_NANOS
    if which == "can":
      runner.handle_can_events([evt])
    elif which == "carState":
      runner.handle_car_state(evt.carState)
    elif which == "deviceMotion":
      runner.handle_device_motion(evt.deviceMotion, evt.valid)
    elif which == "extrinsicsCalibration":
      runner.handle_extrinsics(evt.extrinsicsCalibration)
  return runner


def add_estimator_arguments(parser: argparse.ArgumentParser) -> None:
  parser.add_argument("logs", nargs="+", help="route names, segment folders or rlog files")
  parser.add_argument("--root", help=f"folder holding <route>--<n> segment folders (default {DEVICE_ROOT} or {MAC_ROOT})")
  parser.add_argument("--base-weight-lb", type=float, default=0., help="replay as if CruiseBaseWeightLb were set to this")
  parser.add_argument("--dbc", help="message database name or file (default: the platform's)")
  parser.add_argument("--table", help="published-weight table file (default: ram_hd_weights.json next to the service)")
  parser.add_argument("--efficiency", type=float, default=est.EFFICIENCY)
  parser.add_argument("--rolling-resistance", type=float, default=est.ROLLING_RESISTANCE)
  parser.add_argument("--body-pitch", type=float, default=est.BODY_PITCH_PER_ACCEL, help="body pitch per acceleration, rad per m/s2")


def runner_factory(args, record_windows: bool = False):
  def make(CP):
    r = WeightGradeRunner(CP, base_weight_lb=args.base_weight_lb, known_extra_lb=getattr(args, "known_extra_lb", 0.),
                          dbc_name=args.dbc, table_path=args.table, record_windows=record_windows)
    e = r.estimator
    e.efficiency = args.efficiency
    e.rolling_resistance = args.rolling_resistance
    e.grade.body_pitch_per_accel = args.body_pitch
    return r
  return make


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  add_estimator_arguments(parser)
  parser.add_argument("--known-mass-lb", type=float, help="the true combined weight, to report the accuracy")
  parser.add_argument("--known-extra-lb", type=float, default=0., help="as the CruiseKnownExtraLb setting (published only)")
  parser.add_argument("--every", type=float, default=60., help="print a line every this many seconds of drive")
  parser.add_argument("--csv", help="write the estimate at 10 Hz to this file")
  args = parser.parse_args(argv)

  files = find_log_files(args.logs, args.root)
  print(f"replaying {len(files)} log file(s): {files[0]}{' ... ' + files[-1] if len(files) > 1 else ''}")

  stats = {"t0": None, "steps": 0, "clean": 0, "moving": 0, "last_print": None, "reasons": Counter(), "sources": Counter(),
           "lock_rules": Counter(), "grade_min": 0., "grade_max": 0.}
  csv_file = open(args.csv, "w", newline="") if args.csv else None
  writer = csv.writer(csv_file) if csv_file else None
  if writer:
    writer.writerow(["t_s", "speed_mps", "grade", "grade_std", "mass_lb", "mass_std_lb", "drag", "drag_std", "clean",
                     "mass_updates", "grade_source", "converter_rule", "converter_locked"])

  def on_step(runner, t, out):
    if stats["t0"] is None:
      stats["t0"] = t
      print(f"configuration: {runner.configuration_text}")
      print(f"starting weight: {lb(runner.start.mass_kg):.0f} lb +/- {lb(runner.start.std_kg):.0f} ({runner.start.source}: " +
            f"{runner.start.detail})")
      missing = ", ".join(runner.reader.not_in_database)
      print(f"signals not in the database (unavailable): {missing or 'none'}")
      print("    time  speed    grade (std)     weight lb (std)     drag (std)  clean  updates  grade from       converter")
    ts = (t - stats["t0"]) * 1e-9
    inp = runner.last_inputs
    stats["steps"] += 1
    stats["clean"] += out.clean
    if inp.v > est.STOP_SPEED_MPS:
      stats["moving"] += 1
      stats["reasons"].update(out.not_clean_reasons)
    stats["sources"][out.grade_source] += 1
    stats["lock_rules"][out.converter_lock_rule] += 1
    stats["grade_min"] = min(stats["grade_min"], out.grade)
    stats["grade_max"] = max(stats["grade_max"], out.grade)
    if writer:
      writer.writerow([f"{ts:.1f}", f"{inp.v:.3f}", f"{out.grade:.5f}", f"{out.grade_std:.5f}", f"{lb(out.mass_kg):.1f}",
                       f"{lb(out.mass_std_kg):.1f}", f"{out.drag:.4f}", f"{out.drag_std:.4f}", int(out.clean), out.mass_updates,
                       out.grade_source, out.converter_lock_rule, int(out.converter_locked)])
    if stats["last_print"] is None or ts - stats["last_print"] >= args.every:
      stats["last_print"] = ts
      clean_pct = 100. * stats["clean"] / max(stats["moving"], 1)
      print(f"{ts:7.0f}s {inp.v * MPS_TO_MPH:4.0f}mph {100 * out.grade:+6.2f}% ({100 * out.grade_std:4.2f}) " +
            f"{lb(out.mass_kg):8.0f} ({lb(out.mass_std_kg):6.0f}) {out.drag:6.2f} ({out.drag_std:4.2f}) {clean_pct:5.1f}% " +
            f"{out.mass_updates:6d}/{out.rejected_updates:<4d} {out.grade_source:16s} {out.converter_lock_rule}")

  def on_event(runner, t, e):
    print(f"{(t - stats['t0']) * 1e-9:7.0f}s  {e}")

  runner = replay(files, runner_factory(args), on_step, on_event)
  if csv_file:
    csv_file.close()
  if runner is None or runner.last_output is None:
    print("no carParams in these logs: nothing replayed")
    return 1

  out = runner.last_output
  moving = max(stats["moving"], 1)
  print()
  print(f"drive: {stats['steps'] / 10. / 60.:.1f} min, moving {stats['moving'] / 10. / 60.:.1f} min, " +
        f"clean {100. * stats['clean'] / moving:.1f}% of moving time, {out.mass_updates} weight updates " +
        f"({out.rejected_updates} rejected as outliers), {out.qualifying_stops} qualifying stops")
  print(f"grade range {100 * stats['grade_min']:+.1f}% to {100 * stats['grade_max']:+.1f}%; grade from: "
        + ", ".join(f"{k} {100. * v / stats['steps']:.0f}%" for k, v in stats["sources"].most_common()))
  print("converter rule: " + ", ".join(f"{k} {100. * v / stats['steps']:.0f}%" for k, v in stats["lock_rules"].most_common()))
  print("moving but not clean, why (share of moving time): " +
        ", ".join(f"{k} {100. * v / moving:.0f}%" for k, v in stats["reasons"].most_common(8)))
  print(f"signals available at the end: {', '.join(runner.available_fields(runner.reader.last_nanos))}")
  print(f"final estimate: {lb(out.mass_kg):.0f} lb +/- {lb(out.mass_std_kg):.0f} lb, air drag {out.drag:.2f} +/- {out.drag_std:.2f} " +
        f"N/(m/s)^2, weight-drag correlation {out.mass_drag_correlation:+.2f}")
  if args.known_mass_lb:
    err = lb(out.mass_kg) - args.known_mass_lb
    sigmas = abs(err) / max(lb(out.mass_std_kg), 1e-6)
    print(f"known weight {args.known_mass_lb:.0f} lb: error {err:+.0f} lb ({100. * err / args.known_mass_lb:+.1f}%), " +
          f"{sigmas:.1f} standard deviations ({'within' if sigmas <= 2. else 'outside'} two)")
  return 0


if __name__ == "__main__":
  sys.exit(main())
