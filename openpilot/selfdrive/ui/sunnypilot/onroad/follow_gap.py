"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Following gap in seconds (Settings > Visuals > Following Gap in Seconds): the distance to the car ahead divided by the
car's own speed, in a box below the MAX speed, so the driver can see the gap while choosing the factory cruise
control's distance setting. Display only: nothing is sent to the car.

The distance is the factory windshield unit's own reading of the car its cruise control follows (carStateSP.factoryLead,
Ram Heavy Duty only, filled in only while the factory cruise control is engaged), or else the comma camera's car ahead
(modelV2.leadsV3[0]) when the camera is at least 50% sure of it. Both are measured from the windshield.
"""
from enum import IntEnum

import pyray as rl

from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG, COLORS, CRUISE_DISABLED_CHAR
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached

PARAM = "ShowFollowingGap"
MIN_SPEED = 5. * CV.MPH_TO_MS  # below this the gap in seconds grows without meaning; a dash instead
CAMERA_MIN_PROB = 0.5          # the camera's car ahead counts only when the camera is at least 50% sure of it
MAX_SHOWN_S = 10.              # 10 s or more reads "10+ s": far past any distance setting
SMOOTHING_S = 0.5              # the camera's distance jitters frame to frame; this keeps the number readable
NEW_CAR_JUMP_S = 0.5           # a reading this far from the shown gap is another car (a cut-in): shown at once

BOX_HEIGHT = 165
BOX_GAP = 25      # below the MAX speed box
TEXT_MARGIN = 14  # text shrinks to keep this much room from the box's sides


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
    return CRUISE_DISABLED_CHAR
  if gap >= MAX_SHOWN_S:
    return "10+ s"
  return f"{gap:.1f} s"


class FollowGap:
  """follow_gap() smoothed over SMOOTHING_S; starts over when the car is lost, the source changes, or the reading
  jumps by NEW_CAR_JUMP_S or more"""
  def __init__(self, dt: float):
    self._filter = FirstOrderFilter(0., SMOOTHING_S, dt, initialized=False)
    self.gap: float | None = None
    self.source = GapSource.none

  def update(self, v_ego: float, factory_present: bool, factory_d_rel: float,
             camera_prob: float, camera_d_rel: float) -> tuple[float | None, GapSource]:
    gap, source = follow_gap(v_ego, factory_present, factory_d_rel, camera_prob, camera_d_rel)
    if gap is None or source != self.source or (self.gap is not None and abs(gap - self.gap) >= NEW_CAR_JUMP_S):
      self._filter.initialized = False
    self.gap = None if gap is None else self._filter.update(gap)
    self.source = source
    return self.gap, self.source


class FollowGapRenderer:
  def __init__(self):
    self._follow_gap = FollowGap(1. / gui_app.target_fps)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)

  def update(self) -> None:
    if not ui_state.show_following_gap:
      return
    sm = ui_state.sm
    factory = sm['carStateSP'].factoryLead
    leads = sm['modelV2'].leadsV3
    camera_prob, camera_d_rel = (leads[0].prob, leads[0].x[0]) if len(leads) and len(leads[0].x) else (0., 0.)
    self._follow_gap.update(sm['carState'].vEgo, factory.present, factory.dRel, camera_prob, camera_d_rel)

  def render(self, rect: rl.Rectangle) -> None:
    if not ui_state.show_following_gap:
      return

    # same width and left edge as the MAX speed box, right below it
    width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - width) // 2
    y = rect.y + 45 + UI_CONFIG.set_speed_height + BOX_GAP
    box = rl.Rectangle(x, y, width, BOX_HEIGHT)
    rl.draw_rectangle_rounded(box, 0.35, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(box, 0.35, 10, 6, COLORS.BORDER_TRANSLUCENT)

    gap = self._follow_gap.gap
    self._draw_centered(self._font_semi_bold, tr("GAP"), 40, x, width, y + 12, COLORS.GREY)
    self._draw_centered(self._font_bold, gap_text(gap), 62, x, width, y + 58, COLORS.WHITE if gap is not None else COLORS.DARK_GREY)
    if gap is not None:
      source = tr("FACTORY") if self._follow_gap.source == GapSource.factory else tr("CAMERA")
      self._draw_centered(self._font_semi_bold, source, 28, x, width, y + 125, COLORS.GREY)

  @staticmethod
  def _draw_centered(font: rl.Font, text: str, size: int, x: float, width: float, y: float, color: rl.Color) -> None:
    """centered in the box's width, shrunk (about the same middle) if it would come closer than TEXT_MARGIN to a side"""
    room = width - 2 * TEXT_MARGIN
    text_width = measure_text_cached(font, text, size).x
    if text_width > room:
      fitted = int(size * room / text_width)
      y += (size - fitted) / 2
      size = fitted
      text_width = measure_text_cached(font, text, size).x
    rl.draw_text_ex(font, text, rl.Vector2(x + (width - text_width) / 2, y), size, 0, color)
