"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.lead_speed_calibration import LeadSpeedCalibration, prior_correction

MPH = CV.MPH_TO_MS

# Settings > Cruise > Custom Ram ACC Tuning, Ram Heavy Duty trucks only, and only with Intelligent Cruise Button
# Management in use (the truck's own cruise control does the speed control and sunnypilot presses its buttons)
RAM_ACC_TUNING_PARAM = "RamAccTuning"
RAM_ACC_TUNING_PLATFORM = "RAM_HD_5TH_GEN"


def ram_acc_tuning_available(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> bool:
  return CP.carFingerprint == RAM_ACC_TUNING_PLATFORM and not CP.openpilotLongitudinalControl and not CP_SP.pcmCruiseSpeed


# Planned-speed target for Intelligent Cruise Button Management on a car whose own adaptive cruise control does the
# following and braking (edfrahm/SP-RamHD-TICI-Port#11). It follows jvePilot's method (Chrysler, Jeep and Ram 1500
# cruise buttons): the average of the planner's speed plan, which includes the cars the camera sees, plus a capped
# look-ahead of the planned acceleration; and, when the plan says coast, a set speed 4 mph below the current speed.
# A lowered set speed makes the Ram HD's cruise control close the throttle and coast (Phase 1 study: 0.05 to 0.06 g,
# no brake request), so a target at or below the current speed means "coast", never "brake".
LOOKAHEAD_S = 4.  # s, jvePilot: the planned acceleration is projected this far ahead
LOOKAHEAD_DOWN = 10. * MPH  # jvePilot's cap on the look-ahead, both ways
LOOKAHEAD_UP = 10. * MPH
COAST_DROP = 4. * MPH  # jvePilot: set speed below the current speed when the plan says coast
MAX_BELOW_EGO = 10. * MPH  # the target never asks for more than this below the current speed
HYSTERESIS = 0.75 * MPH  # jvePilot: a rise is taken at once, a drop only once it is this large

# With a car ahead, the target stays near that car's speed: at most its speed within a 2 s time gap (the truck's own
# cruise control follows at about 1.6 s), rising to 5 mph above it at a 4 s gap or more, so the truck still closes a
# large gap; and never above the truck's own speed while that car is slower. So the cruise control is never told to
# speed up toward slower traffic the camera already sees.
LEAD_GAP_BP = [2., 4.]  # s, distance to the car ahead divided by the truck's speed
LEAD_MARGIN_V = [0., 5. * MPH]
# The planner's speed for the car ahead reads low, a camera measurement error that grows with distance; corrected by
# the figure the truck learns from its own steady following (lead_speed_calibration.py).
lead_speed_correction = prior_correction

# Distant-car memory. The driving model can lose confidence in a distant car: on 2026-10-19 at 14:48:45 it still placed
# a car about 112 m ahead at about 61 mph, but with a probability of 0.05 to 0.5, below the 0.5 the planner needs, so
# for 7.5 s the plan saw an empty road and asked for the 75 mph set speed. A car that was seen for a while and fades
# out far away is therefore remembered for a few seconds at its last speed, which the target then stays at.
MEMORY_SEEN_S = 1.  # the car was seen at least this long
MEMORY_MIN_DIST = 60.  # m, and faded out at least this far ahead (a nearer car that vanishes has usually left the lane)
MEMORY_S = 10.  # s, remembered at most this long
MEMORY_GONE_NEAR = 40.  # m, forgotten once the truck would be this close to it and the camera still does not see it
MEMORY_GONE_FAR = 150.  # m, or once it would be this far ahead
LEAD_SPEED_TAU = 0.5  # s, smoothing of the car ahead's speed

# Lane changes get no rules of their own (Eric, 2026-10-20: a situational decision, not a rule, until the Comma detects
# traffic in the next lane): the rules above apply to whichever car the driving model reports in the path, and the
# remembered far car is forgotten at a turn signal or lane change so it cannot add an effect of its own.


class PlannedSpeed:
  def __init__(self, dt: float = DT_MDL, calibration: LeadSpeedCalibration | None = None):
    self.dt = dt
    self.calibration = calibration or LeadSpeedCalibration("")
    self.v_target = 0.
    self.raw_target = 0.
    self.reset_next = True

    self.lead_seen_t = 0.  # how long the car ahead has been seen
    self.lead_v = 0.  # its smoothed speed
    self.lead_d = 0.  # its distance when last seen, then reckoned from both speeds
    self.lead_lost_t = MEMORY_S  # how long since it was last seen
    self.memory = False  # the remembered car currently limits the target

  def reset(self) -> None:
    self.reset_next = True

  def _update_memory(self, v_ego: float, lead_present: bool, lead_d: float, lead_v: float, lane_change: bool) -> None:
    if lead_present:
      alpha = self.dt / (LEAD_SPEED_TAU + self.dt) if self.lead_seen_t > 0. else 1.
      self.lead_v += alpha * (lead_v + self.calibration.correction(lead_d) - self.lead_v)
      self.lead_d = lead_d
      self.lead_seen_t += self.dt
      self.lead_lost_t = 0.
      self.memory = False
      return

    self.lead_lost_t += self.dt
    self.lead_d += (self.lead_v - v_ego) * self.dt
    if not self.memory and self.lead_lost_t <= self.dt * 1.5:
      self.memory = self.lead_seen_t >= MEMORY_SEEN_S and self.lead_d >= MEMORY_MIN_DIST
    if self.memory and (lane_change or self.lead_lost_t > MEMORY_S or not MEMORY_GONE_NEAR < self.lead_d < MEMORY_GONE_FAR):
      self.memory = False
    if self.lead_lost_t > MEMORY_S:
      self.lead_seen_t = 0.

  def update(self, v_ego: float, speeds, accels, allow_throttle: bool, lead_present: bool, lead_d: float, lead_v: float,
             lane_change: bool = False, force_decel: bool = False) -> float:
    """Returns the target in m/s. speeds and accels are the planner's plan (longitudinalPlan.speeds/.accels);
    lane_change is a turn signal on or a lane change under way."""
    self._update_memory(v_ego, lead_present, lead_d, lead_v, lane_change)

    if force_decel or not allow_throttle or len(speeds) == 0:
      raw = v_ego - COAST_DROP
    else:
      raw = float(np.mean(speeds)) + float(np.clip(np.mean(accels) * LOOKAHEAD_S, -LOOKAHEAD_DOWN, LOOKAHEAD_UP))
      if lead_present:
        gap_s = self.lead_d / max(v_ego, 1.)
        raw = min(raw, self.lead_v + float(np.interp(gap_s, LEAD_GAP_BP, LEAD_MARGIN_V)))
        if self.lead_v < v_ego:
          raw = min(raw, v_ego)
      elif self.memory:
        raw = min(raw, self.lead_v)
    raw = max(raw, v_ego - MAX_BELOW_EGO, 0.)
    self.raw_target = raw

    if self.reset_next or raw > self.v_target or raw < self.v_target - HYSTERESIS:
      self.v_target = raw
    self.reset_next = False
    return self.v_target
