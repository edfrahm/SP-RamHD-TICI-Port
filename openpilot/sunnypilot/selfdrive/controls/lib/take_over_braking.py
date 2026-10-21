"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections import deque
from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.lead_speed_calibration import LeadSpeedCalibration

# Take-over-braking alert (edfrahm/SP-RamHD-TICI-Port#28), Ram Heavy Duty trucks only. A warning only: it brakes nothing,
# changes no steering or speed, and disengages nothing. The truck's own cruise control brakes up to about 0.3 g; when the
# braking needed to keep the minimum margin behind the car ahead is more than that, the driver gets a loud repeating alert
# and a message to brake (Volvo's method, Eric 2026-10-20). Neither the driver's braking nor the factory's silences it. It
# ends when the need passes, or when the truck's measured slowing meets the need.
PARAM = "RamTakeOverBrakingAlert"  # Settings > Cruise, on unless the driver turns it off
PLATFORM = "RAM_HD_5TH_GEN"

G = 9.81  # m/s² per g
MPH = CV.MPH_TO_MS


def take_over_braking_available(CP: structs.CarParams) -> bool:
  return CP.carFingerprint == PLATFORM


# ---- Every number the alert uses, in one place -------------------------------------------------------------------
@dataclass(frozen=True)
class MarginInputs:
  """The minimum margin: the gap needed so that if the car ahead brakes as hard as it can now, the truck, after the
  driver's reaction time and braking as hard as it can, stops this buffer behind it:
    truck speed × reaction time + the truck's stopping distance − the car ahead's stopping distance + buffer.
  Placeholders until the truck braking model (#26) replaces them."""
  reaction_s: float = 1.5  # s, the driver's reaction time (approved in #26)
  truck_braking_g: float = 0.6  # the truck's braking, the stopping-distance part of the margin
  lead_braking_g: float = 0.8  # the car ahead's braking (a good passenger car)
  buffer_m: float = 10.  # m, left between the two once both have stopped


CRUISE_LIMIT_G = 0.3  # the most the factory cruise control brakes (observed: 2026-10-19, route 00000783, 14:48:55-59)

# The room before the margin counts as at least the gap the closing eats during the driver's reaction time. Without
# this floor, being inside the margin while closing at all counts as more than any limit, and the factory cruise
# follows at 1.5-1.6 s and starts slowing for traffic at 1.7-2.5 s, both inside the 2.5 s the placeholder margin asks
# for at highway speed: replayed on every recorded drive, that alerted about 156 times an hour, nearly all of it
# ordinary following. With it, at or inside the margin the need is the closing speed over twice the reaction time, so
# the alert sounds there when the truck closes faster than about 20 mph.
ROOM_FLOOR_ON_REACTION = True

# An alert starts only at 40 mph or more (once started, it runs to its end at any speed). Below that, the placeholder
# margin asks for more braking than ordinary stops behind stopped traffic use: replayed, starting at 30 mph added 13-16
# alerts in 14.7 hours, nearly every one with Eric already braking for a stopped or nearly stopped car.
MIN_START_SPEED = 40. * MPH

CONFIRM_S = 0.3  # s, the need stays above the limit this long before the alert starts: rejects single bad camera frames
END_S = 0.5  # s, the need stays passed (or met) this long before the alert ends: a dropped frame does not end it
SLOWING_WINDOW_S = 0.5  # s, the truck's measured slowing is the trend of its speed over this long

# When neither sunnypilot steering nor the factory cruise control is engaged, the alert uses the camera only, and only
# for a car within 60 m that the camera is at least 50% sure of (Eric, 2026-10-20). Over all 79 recorded drives the
# camera's driving model was that sure of the car ahead in 96-98% of frames within 60 m, and 70% or less beyond.
CAMERA_ONLY_MAX_DIST = 60.  # m
CAMERA_ONLY_MIN_PROB = 0.5

# The camera reads the car ahead slow in steady following; the learned figure (RamLeadSpeedCalibration) corrects that.
# But a car the truck is closing in on already reads too fast (against the factory unit, #11), so the correction would
# only make the alert later then: it fades out once the corrected reading says the truck is closing in by 1 mph.
CORRECTION_FADE_CLOSING = 1. * MPH

# The factory windshield unit's own car (carStateSP.factoryLead), filled in only while the factory cruise is engaged.
# Its speed is capped at the set speed: a reading within 1 km/h of the lowest set speed of the last second (the reading
# follows a set-speed change within 0.7 s) only says the car goes at least that fast. Its speed is then worked out
# from the unit's distance trend over at least 1 s plus the truck's speed. A car it drops (254, or the factory cruise
# disengaging) is held for up to 2 s, its distance reckoned from both speeds; a held car is dropped at a lane change.
# A reading 5 m or more from where its car should be is another car (from one frame to the next, 0.05 s, the same car
# moves about 2 m at most): on 0000076a the unit read 114 m for a quarter second between readings of 136 and 137 m.
UNIT_CAP_KPH = 1.
UNIT_CAP_WINDOW_S = 1.
UNIT_TREND_S = 1.
UNIT_HOLD_S = 2.
UNIT_NEW_CAR_JUMP = 5.  # m
# ------------------------------------------------------------------------------------------------------------------


class Source(IntEnum):
  none = 0
  factoryUnit = 1
  camera = 2


def minimum_margin(m: MarginInputs, v_ego: float, v_lead: float) -> float:
  return (v_ego * m.reaction_s + v_ego ** 2 / (2. * m.truck_braking_g * G)
          - max(v_lead, 0.) ** 2 / (2. * m.lead_braking_g * G) + m.buffer_m)


def braking_needed(gap: float, closing: float, margin: float, reaction_s: float = 0.) -> float:
  """m/s² of slowing needed to stop the closing before the gap shrinks to the margin: closing² / (2 × room), with the
  room counted as at least closing × reaction_s. With no such floor, already inside the margin while closing counts as
  more than any limit."""
  if closing <= 0.:
    return 0.
  room = max(gap - margin, closing * reaction_s)
  if room <= 0.:
    return float('inf')
  return closing ** 2 / (2. * room)


class FactoryUnitLead:
  """The factory windshield unit's car: its reported distance and speed, with the speed cap and dropouts handled."""
  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.set_speeds: deque[float] = deque(maxlen=int(round(UNIT_CAP_WINDOW_S / dt)) + 1)
    # the unit's distances and the truck's speeds over the last second, from consecutive readings of the same car only
    self.trend: deque[tuple[float, float]] = deque(maxlen=int(round(UNIT_TREND_S / dt)) + 1)
    self.available = False  # a car seen now, or held through a dropout
    self.live = False  # seen in this frame
    self.d = 0.
    self.v = 0.
    self.speed_from_trend = False
    self.speed_known = False  # a real (uncapped) reading, or a trend, has given its speed
    self.hold_frames = int(round(UNIT_HOLD_S / dt))
    self.lost_frames = self.hold_frames

  def update(self, v_ego: float, present: bool, d: float, v_reading: float, set_speed: float, lane_change: bool) -> bool:
    self.set_speeds.append(set_speed)
    self.live = present
    if not present:
      self.trend.clear()
      self.lost_frames += 1
      if self.available:
        self.d += (self.v - v_ego) * self.dt
        if lane_change or self.lost_frames > self.hold_frames:
          self.available = False
      return self.available

    # another car: none before, or a jump from where the last one is (seen, or reckoned through a dropout)
    if not self.available or abs(d - self.d) >= UNIT_NEW_CAR_JUMP:
      self.trend.clear()
      self.speed_known = False
    self.trend.append((d, v_ego))
    self.d, self.available, self.lost_frames = d, True, 0

    at_cap = round(v_reading * CV.MS_TO_KPH) >= round(min(self.set_speeds) * CV.MS_TO_KPH) - UNIT_CAP_KPH
    self.speed_from_trend = False
    if not at_cap:
      self.v, self.speed_known = v_reading, True
    elif len(self.trend) == self.trend.maxlen:
      # faster than the set speed (or a reading at the cap): the unit's distance trend plus the truck's speed
      w = np.array(self.trend)
      gap_rate = float(np.polyfit(np.arange(len(w)) * self.dt, w[:, 0], 1)[0])
      self.v, self.speed_known, self.speed_from_trend = float(np.mean(w[:, 1])) + gap_rate, True, True
    elif not self.speed_known:
      self.v = v_reading  # at least this fast: the lowest it can be, so the closing reads the largest it can be
    return True


class TakeOverBraking:
  def __init__(self, calibration: LeadSpeedCalibration | None = None, margin: MarginInputs | None = None,
               dt: float = DT_MDL):
    self.dt = dt
    self.calibration = calibration or LeadSpeedCalibration("")
    self.margin_inputs = margin or MarginInputs()
    self.unit = FactoryUnitLead(dt)
    self.speeds: deque[float] = deque(maxlen=int(round(SLOWING_WINDOW_S / dt)))
    self.confirm_frames = int(round(CONFIRM_S / dt))
    self.end_frames = int(round(END_S / dt))
    self.min_start_speed = MIN_START_SPEED

    self.active = False
    self.over_frames = 0
    self.clear_frames = 0
    # what the last frame saw, for the logs
    self.source = Source.none
    self.gap = 0.
    self.lead_v = 0.
    self.closing = 0.
    self.margin = 0.
    self.need = 0.  # m/s²
    self.slowing = 0.  # m/s², the truck's measured slowing (positive when slowing)
    self.over = False

  def reset(self) -> None:
    """Forget everything seen (the setting was turned off)"""
    self.unit = FactoryUnitLead(self.dt)
    self.speeds.clear()
    self.active, self.over, self.over_frames, self.clear_frames = False, False, 0, 0
    self.source, self.gap, self.lead_v, self.closing, self.margin, self.need = Source.none, 0., 0., 0., 0., 0.

  def braking_needed(self, v_ego: float, gap: float, lead_v: float) -> tuple[float, float]:
    """The minimum margin (m) and the slowing needed to keep it (m/s²)"""
    margin = minimum_margin(self.margin_inputs, v_ego, lead_v)
    reaction_s = self.margin_inputs.reaction_s if ROOM_FLOOR_ON_REACTION else 0.
    return margin, braking_needed(gap, v_ego - lead_v, margin, reaction_s)

  def choose_car(self, v_ego: float, camera_ok: bool, camera_d: float, camera_v: float) -> tuple[Source, float, float]:
    """Which car ahead: the factory unit's whenever it reports one, else the camera's. Returns the source, the gap and
    that car's speed."""
    if self.unit.available:
      return Source.factoryUnit, self.unit.d, self.unit.v
    if camera_ok:
      return Source.camera, camera_d, self.camera_speed(v_ego, camera_d, camera_v)
    return Source.none, 0., 0.

  def camera_speed(self, v_ego: float, d: float, v: float) -> float:
    correction = self.calibration.correction(d)
    closing = v_ego - (v + correction)
    return v + correction * float(np.interp(closing, [0., CORRECTION_FADE_CLOSING], [1., 0.]))

  def _measured_slowing(self, v_ego: float) -> float:
    """The slope of the truck's speed over the last half second (a least-squares line), as slowing"""
    self.speeds.append(v_ego)
    n = len(self.speeds)
    if n < 2:
      return 0.
    x = np.arange(n) - (n - 1) / 2.
    return -float(np.dot(x, self.speeds) / (np.dot(x, x) * self.dt))

  def update(self, v_ego: float, cruise_engaged: bool, steering_engaged: bool, set_speed: float, lane_change: bool,
             camera_present: bool, camera_d: float, camera_v: float, camera_prob: float,
             unit_present: bool, unit_d: float, unit_v: float) -> bool:
    """One planner frame. camera_* is the planner's car ahead (radarState.leadOne), unit_* the factory windshield unit's
    (carStateSP.factoryLead), set_speed the factory cruise control's set speed, all in m and m/s. True while alerting."""
    self.slowing = self._measured_slowing(v_ego)
    self.unit.update(v_ego, unit_present, unit_d, unit_v, set_speed, lane_change)

    # With nothing engaged, the camera's car counts only within 60 m and 50% sure. An alert under way keeps every
    # source, so the driver's braking, which disengages the factory cruise control (and its unit's car), does not end it.
    engaged = cruise_engaged or steering_engaged or self.active
    camera_ok = camera_present and (engaged or (camera_d <= CAMERA_ONLY_MAX_DIST and camera_prob >= CAMERA_ONLY_MIN_PROB))
    self.source, self.gap, self.lead_v = self.choose_car(v_ego, camera_ok, camera_d, camera_v)

    if self.source == Source.none:
      self.closing, self.margin, self.need = 0., 0., 0.
    else:
      self.closing = v_ego - self.lead_v
      self.margin, self.need = self.braking_needed(v_ego, self.gap, self.lead_v)
    self.over = self.need > CRUISE_LIMIT_G * G and self.slowing < self.need

    if not self.active:
      self.over_frames = self.over_frames + 1 if self.over and v_ego >= self.min_start_speed else 0
      if self.over_frames >= self.confirm_frames:
        self.active, self.clear_frames = True, 0
    else:
      self.clear_frames = 0 if self.over else self.clear_frames + 1
      if self.clear_frames >= self.end_frames:
        self.active, self.over_frames = False, 0
    return self.active
