"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The following gap in seconds and where its distance comes from, for the box below the MAX speed (follow_gap.py). No
screen code here, so it can be tested and replayed on a computer without a display.

The distance is the factory windshield unit's own reading of the car its cruise control follows (carStateSP.factoryLead,
Ram Heavy Duty only, filled in only while the factory cruise control is engaged), or else the comma camera's car ahead
(modelV2.leadsV3[0]) when the camera is at least 50% sure of it. Both are measured from the windshield.
"""
from enum import IntEnum

from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter

PARAM = "ShowFollowingGap"
MIN_SPEED = 5. * CV.MPH_TO_MS  # below this the gap in seconds grows without meaning; a dash instead
CAMERA_MIN_PROB = 0.5          # the camera's car ahead counts only when the camera is at least 50% sure of it
MAX_SHOWN_S = 10.              # 10 s or more reads "10+ s": far past any distance setting
DASH = '–'                     # no gap to show: the MAX speed box's dash (hud_renderer.CRUISE_DISABLED_CHAR)
# The camera's distance jitters frame to frame. On drive 00000789 (16 minutes) 1 s of smoothing brings the shown
# camera number from about 3 changes a second to 2; the factory unit's number changes about once a second either way.
SMOOTHING_S = 1.
# A reading at least this far from the shown gap is another car (a cut-in) and is shown at once. A share of the gap,
# not a fixed amount: far cars jitter more (on 00000789 a fixed 0.5 s restarted the number 412 times, this 68).
NEW_CAR_JUMP_S = 0.5
NEW_CAR_JUMP_SHARE = 0.25

class GapSource(IntEnum):
  none = 0
  factory = 1
  camera = 2


def follow_gap(v_ego: float, factory_present: bool, factory_d_rel: float,
               camera_prob: float, camera_d_rel: float) -> tuple[float | None, GapSource]:
  """Seconds to the car ahead at the current speed (distance in m ÷ v_ego in m/s), and where the distance came from.
  None below MIN_SPEED or with no car ahead."""
  if v_ego < MIN_SPEED:
    return None, GapSource.none
  if factory_present and factory_d_rel > 0.:
    return factory_d_rel / v_ego, GapSource.factory
  if camera_prob >= CAMERA_MIN_PROB and camera_d_rel > 0.:
    return camera_d_rel / v_ego, GapSource.camera
  return None, GapSource.none


def gap_text(gap: float | None) -> str:
  if gap is None:
    return DASH
  if gap >= MAX_SHOWN_S:
    return "10+ s"
  return f"{gap:.1f} s"


class FollowGap:
  """follow_gap() smoothed over SMOOTHING_S; starts over when the car is lost, the source changes, or the reading
  jumps by NEW_CAR_JUMP_S and NEW_CAR_JUMP_SHARE of the shown gap or more"""
  def __init__(self, dt: float):
    self._filter = FirstOrderFilter(0., SMOOTHING_S, dt, initialized=False)
    self.gap: float | None = None
    self.source = GapSource.none

  def update(self, v_ego: float, factory_present: bool, factory_d_rel: float,
             camera_prob: float, camera_d_rel: float) -> tuple[float | None, GapSource]:
    gap, source = follow_gap(v_ego, factory_present, factory_d_rel, camera_prob, camera_d_rel)
    new_car = self.gap is not None and gap is not None and abs(gap - self.gap) >= max(NEW_CAR_JUMP_S, NEW_CAR_JUMP_SHARE * self.gap)
    if gap is None or source != self.source or new_car:
      self._filter.initialized = False
    self.gap = None if gap is None else self._filter.update(gap)
    self.source = source
    return self.gap, self.source
