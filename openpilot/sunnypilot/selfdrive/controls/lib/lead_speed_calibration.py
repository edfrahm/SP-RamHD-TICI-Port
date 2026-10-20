"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

MPH = CV.MPH_TO_MS

# How slow the camera model reads the speed of the car ahead (edfrahm/SP-RamHD-TICI-Port#11), learned on the truck
# from steady following: 5 s with the car ahead in view throughout and the gap holding within 0.3 m/s, so both went the
# same speed and any difference is the camera's. Over every recorded drive (749 stretches, 48 drives) it read slow in
# all 31 drives with enough data, the same with the driver on the pedals as with the cruise control, more the farther
# the car, and by different amounts for different driving models: a measurement error, so it is learned per distance
# and per driving model, and kept between drives in the setting RamLeadSpeedCalibration.
PARAM = "RamLeadSpeedCalibration"
BIN_EDGES = [15., 30., 45., 60., 80.]  # m
BIN_CENTERS = [22.5, 37.5, 52.5, 70.]
# Starting values: this port's driving model, 83 stretches in 6 drives (2026-10-16 to 10-19): about 1.0 mph slow at
# 30 m, 1.9 at 45 m and 2.7 at 60 m; little data beyond 60 m, so the last band starts at the 60 m value.
PRIOR = [0.6 * MPH, 1.4 * MPH, 2.3 * MPH, 2.7 * MPH]
WINDOW_FRAMES = int(5. / DT_MDL)
STEADY_GAP_RATE = 0.3  # m/s, the gap's trend over the stretch
MIN_SPEED = 15.  # m/s, about 34 mph
LEARN_WINDOWS = 50.  # each stretch moves its band's figure 1/50 of the way (about 4 min of steady following for most of it)
MAX_STEP_PER_DRIVE = 0.5 * MPH  # a band's figure moves at most this far in one drive
READING_LIMITS = (-2. * MPH, 8. * MPH)  # a stretch reading outside these is not used
USE_FRACTION = 0.9  # corrected by a little less than learned, so it errs toward following farther back
MAX_CORRECTION = 4. * MPH


def prior_correction(lead_d: float) -> float:
  return LeadSpeedCalibration("").correction(lead_d)


class LeadSpeedCalibration:
  def __init__(self, model_ref: str, saved: dict | None = None):
    self.model_ref = model_ref
    self.values = list(PRIOR)  # m/s the camera reads slow, per distance band
    self.counts = [0] * len(PRIOR)  # stretches learned from, per band
    if isinstance(saved, dict) and saved.get("model") == model_ref:
      try:  # a damaged setting falls back to the starting values
        values = [float(v) for v in saved["values"]]
        counts = [int(c) for c in saved.get("counts", self.counts)]
        if len(values) == len(PRIOR) == len(counts) and all(READING_LIMITS[0] <= v <= READING_LIMITS[1] for v in values):
          self.values, self.counts = values, counts
      except (KeyError, TypeError, ValueError):
        pass
    self.drive_start = list(self.values)
    self.window: list[tuple[float, float, float]] = []
    self.changed = False

  def correction(self, lead_d: float) -> float:
    """m/s to add to the planner's speed for the car ahead at lead_d m"""
    slow = float(np.interp(lead_d, [BIN_EDGES[0], *BIN_CENTERS], [0., *self.values]))
    return float(np.clip(USE_FRACTION * slow, 0., MAX_CORRECTION))

  def update(self, v_ego: float, lead_present: bool, lead_d: float, lead_v: float, lane_change: bool) -> bool:
    """One planner frame; True when a steady stretch was learned from"""
    if not lead_present or lane_change or v_ego < MIN_SPEED or not BIN_EDGES[0] <= lead_d < BIN_EDGES[-1]:
      self.window = []
      return False
    self.window.append((lead_d, lead_v, v_ego))
    if len(self.window) < WINDOW_FRAMES:
      return False

    w = np.array(self.window)
    self.window = []
    gap_rate = float(np.polyfit(np.arange(len(w)) * DT_MDL, w[:, 0], 1)[0])
    if abs(gap_rate) > STEADY_GAP_RATE:
      return False
    reads_slow = float(np.mean(w[:, 2] - w[:, 1])) + gap_rate
    if not READING_LIMITS[0] <= reads_slow <= READING_LIMITS[1]:
      return False

    i = int(np.searchsorted(BIN_EDGES, np.mean(w[:, 0]), side="right")) - 1
    learned = self.values[i] + (reads_slow - self.values[i]) / LEARN_WINDOWS
    self.values[i] = float(np.clip(learned, self.drive_start[i] - MAX_STEP_PER_DRIVE, self.drive_start[i] + MAX_STEP_PER_DRIVE))
    self.counts[i] += 1
    self.changed = True
    return True

  def to_param(self) -> dict:
    return {"model": self.model_ref, "values": [round(v, 4) for v in self.values], "counts": self.counts,
            "values_mph": [round(v / MPH, 2) for v in self.values]}
