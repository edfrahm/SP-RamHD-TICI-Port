#!/usr/bin/env python3
"""Builds planned_speed_replay_00000783.json.gz for test_planned_speed.py: what the planner published and saw, once
per published plan (20 per second), between two clock times of a drive. Read-only.

  PYTHONPATH=. python openpilot/sunnypilot/selfdrive/controls/lib/tests/data/make_planned_speed_replay.py \\
    OUT.json.gz 14:48:25 14:48:55 <rlog dir>/00000783--e3bae4dd6c--10/rlog.zst
"""
import datetime
import gzip
import json
import sys

from openpilot.tools.lib.logreader import LogReader

SANE_NS = 1767225600 * 10**9  # 2026-01-01: the comma 3's clock can be unset
FIELDS = ("vEgo", "vCruise", "carSet", "signal", "leadPresent", "leadD", "leadV", "laneChangeState", "forceDecel",
          "enabled", "override", "icbmTarget")


def main():
  out, start, end, files = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:]
  rows, last, off = [], {}, None
  for f in files:
    for m in LogReader(f):
      w, t = m.which(), m.logMonoTime * 1e-9
      if w == "initData" and off is None and m.initData.wallTimeNanos > SANE_NS:
        off = m.initData.wallTimeNanos * 1e-9 - t
      elif w == "carState":
        c = m.carState
        last.update(vEgo=c.vEgo, vCruise=c.vCruise, carSet=c.cruiseState.speed, signal=c.leftBlinker != c.rightBlinker)
      elif w == "radarState":
        lead = m.radarState.leadOne
        last.update(leadPresent=lead.present, leadD=lead.dRel, leadV=lead.vLead)
      elif w == "modelV2":
        last.update(laneChangeState=str(m.modelV2.meta.laneChangeState))
      elif w == "controlsState":
        last.update(forceDecel=m.controlsState.forceDecel)
      elif w == "carControl":
        last.update(enabled=m.carControl.enabled, override=m.carControl.cruiseControl.override)
      elif w == "longitudinalPlanSP":
        last.update(icbmTarget=m.longitudinalPlanSP.vTarget)
      elif w == "longitudinalPlan" and off is not None:
        clock = datetime.datetime.fromtimestamp(t + off).strftime("%H:%M:%S.%f")[:11]
        if start <= clock <= end and len(last) == len(FIELDS):
          p = m.longitudinalPlan
          rows.append(dict(clock=clock, speeds=[round(x, 3) for x in p.speeds], accels=[round(x, 3) for x in p.accels],
                           allowThrottle=p.allowThrottle, **{k: round(v, 3) if isinstance(v, float) else v for k, v in last.items()}))
  with gzip.open(out, "wt") as f:
    json.dump({"route": "00000783--e3bae4dd6c", "rows": rows}, f, separators=(",", ":"))
  print(len(rows), "rows")


if __name__ == "__main__":
  main()
