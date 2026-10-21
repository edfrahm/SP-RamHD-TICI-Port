#!/usr/bin/env python3
"""Builds take_over_braking_replay_00000783.json.gz for test_take_over_braking.py: what the take-over-braking alert reads,
once per driving-model frame (20 per second), between two clock times of a drive. Read-only.

  PYTHONPATH=.:opendbc_repo python openpilot/sunnypilot/selfdrive/controls/lib/tests/data/make_take_over_braking_replay.py \\
    OUT.json.gz 14:48:20 14:49:05 <rlog dir>/00000783--e3bae4dd6c--{9,10,11}/rlog.zst

The windshield unit's distance (DAS_4 byte 7, 254 = no car) and speed (DAS_5 byte 3, km/h) are read from the raw bus 2
frames, as carStateSP.factoryLead reads them on the truck.
"""
import datetime
import gzip
import json
import sys

from openpilot.tools.lib.logreader import LogReader

SANE_NS = 1767225600 * 10**9  # 2026-01-01: the comma 3's clock can be unset
FIELDS = ("vEgo", "cruise", "setSpeed", "signal", "brake", "steering", "camPresent", "camD", "camV", "camProb",
          "laneChangeState", "unitD", "unitKph", "factoryBrake")


def main():
  out, start, end, files = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:]
  rows, last, off = [], {}, None
  for f in files:
    for m in LogReader(f):
      w, t = m.which(), m.logMonoTime * 1e-9
      if w == "initData" and off is None and m.initData.wallTimeNanos > SANE_NS:
        off = m.initData.wallTimeNanos * 1e-9 - t
      elif w == "can":
        for c in m.can:
          if c.src == 2 and c.address == 0x1F5:
            last["unitD"] = c.dat[7]
          elif c.src == 2 and c.address == 0x271:
            last["unitKph"] = c.dat[3]
          elif c.src == 2 and c.address == 0x1F4:
            last["factoryBrake"] = bool((c.dat[4] >> 4) & 0x7)  # DAS_3 ACC_DECEL_REQ
      elif w == "carState":
        c = m.carState
        last.update(vEgo=c.vEgo, cruise=c.cruiseState.enabled, setSpeed=c.cruiseState.speed, brake=c.brakePressed,
                    signal=c.leftBlinker != c.rightBlinker)
      elif w == "selfdriveStateSP":
        last.update(steering=m.selfdriveStateSP.mads.enabled)
      elif w == "radarState":
        lead = m.radarState.leadOne
        last.update(camPresent=lead.present, camD=lead.dRel, camV=lead.vLead, camProb=lead.modelProb)
      elif w == "modelV2" and off is not None:
        last.update(laneChangeState=str(m.modelV2.meta.laneChangeState))
        clock = datetime.datetime.fromtimestamp(t + off).strftime("%H:%M:%S.%f")[:11]
        if start <= clock <= end and len(last) == len(FIELDS):
          rows.append(dict(clock=clock, **{k: round(v, 3) if isinstance(v, float) else v for k, v in last.items()}))
  with gzip.open(out, "wt") as f:
    json.dump({"route": "00000783--e3bae4dd6c", "rows": rows}, f, separators=(",", ":"))
  print(len(rows), "rows")


if __name__ == "__main__":
  main()
