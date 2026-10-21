"""
Tests for the following gap in seconds (Settings > Visuals > Following Gap in Seconds):
- the gap is the distance to the car ahead divided by the car's own speed, shown with one decimal;
- the factory windshield unit's distance is used when it reports a car, else the camera's car ahead when the camera is
  at least 50% sure of it;
- a dash with no car ahead, below 5 mph, and at a standstill;
- the number is smoothed against jitter, but a new car (lost and found, a cut-in) or a new source shows at once;
- the setting exists, is backed up, defaults to off, and the screen and the settings panel use the same setting.

The settings panel needs a graphics window, which a test machine does not have, so the toggle tests read its code with
Python's own parser, as test_ram_acc_tuning.py does.
"""
import ast
import os
import re
import unittest

from openpilot.common.basedir import BASEDIR
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.hud_renderer import CRUISE_DISABLED_CHAR
from openpilot.selfdrive.ui.sunnypilot.onroad.follow_gap import PARAM, FollowGap, GapSource, follow_gap, gap_text
from openpilot.selfdrive.ui.sunnypilot.tests.test_ram_acc_tuning import PARAMS_KEYS, joined

UI_DIR = os.path.join(BASEDIR, "openpilot", "selfdrive", "ui", "sunnypilot")
VISUALS = os.path.join(UI_DIR, "layouts", "settings", "visuals.py")
UI_STATE = os.path.join(UI_DIR, "ui_state.py")
HUD = os.path.join(UI_DIR, "onroad", "hud_renderer.py")

MPH_60 = 60 * CV.MPH_TO_MS  # 26.8 m/s
NO_CAMERA = (0., 0.)


class TestFollowGap(unittest.TestCase):
  def test_gap_is_distance_over_speed(self):
    # 43 m at 60 mph: 1.6 s, the gap the factory cruise control held on Eric's drives
    gap, source = follow_gap(MPH_60, True, 43., *NO_CAMERA)
    self.assertAlmostEqual(gap, 43. / MPH_60)
    self.assertEqual(source, GapSource.factory)
    self.assertEqual(gap_text(gap), "1.6 s")
    self.assertEqual(gap_text(follow_gap(20., True, 50., *NO_CAMERA)[0]), "2.5 s")

  def test_factory_preferred_over_camera(self):
    gap, source = follow_gap(MPH_60, True, 43., 0.9, 60.)
    self.assertEqual(source, GapSource.factory)
    self.assertAlmostEqual(gap, 43. / MPH_60)

  def test_camera_when_factory_has_no_car(self):
    # factory cruise off (or 254, no car): the unit reports nothing, so the camera's car ahead is used
    gap, source = follow_gap(MPH_60, False, 254., 0.9, 60.)
    self.assertEqual(source, GapSource.camera)
    self.assertAlmostEqual(gap, 60. / MPH_60)
    self.assertEqual(follow_gap(MPH_60, True, 0., 0.9, 60.)[1], GapSource.camera)

  def test_camera_needs_half_sure(self):
    self.assertEqual(follow_gap(MPH_60, False, 254., 0.5, 60.)[1], GapSource.camera)
    self.assertEqual(follow_gap(MPH_60, False, 254., 0.49, 60.), (None, GapSource.none))

  def test_no_car_ahead_is_a_dash(self):
    gap, source = follow_gap(MPH_60, False, 254., 0.1, 80.)
    self.assertIsNone(gap)
    self.assertEqual(source, GapSource.none)
    self.assertEqual(gap_text(gap), CRUISE_DISABLED_CHAR)
    self.assertEqual(follow_gap(MPH_60, False, 0., *NO_CAMERA), (None, GapSource.none))

  def test_low_speed_is_a_dash(self):
    for v_ego in (0., 1., 4.9 * CV.MPH_TO_MS):
      self.assertEqual(follow_gap(v_ego, True, 10., 0.9, 10.), (None, GapSource.none), v_ego)
    self.assertEqual(follow_gap(5. * CV.MPH_TO_MS, True, 10., 0.9, 10.)[1], GapSource.factory)

  def test_text(self):
    self.assertEqual(gap_text(0.84), "0.8 s")
    self.assertEqual(gap_text(2.26), "2.3 s")
    self.assertEqual(gap_text(9.94), "9.9 s")
    self.assertEqual(gap_text(10.), "10+ s")
    self.assertEqual(gap_text(35.), "10+ s")


class TestFollowGapSmoothing(unittest.TestCase):
  DT = 0.05  # the comma 3 screen runs at 20 frames a second

  def test_settles_on_steady_gap(self):
    fg = FollowGap(self.DT)
    for _ in range(60):
      gap, source = fg.update(MPH_60, True, 43., *NO_CAMERA)
    self.assertAlmostEqual(gap, 43. / MPH_60, places=3)
    self.assertEqual(source, GapSource.factory)

  def test_first_reading_shown_as_is(self):
    fg = FollowGap(self.DT)
    self.assertAlmostEqual(fg.update(MPH_60, True, 43., *NO_CAMERA)[0], 43. / MPH_60)

  def test_jitter_smoothed(self):
    fg = FollowGap(self.DT)
    fg.update(20., False, 254., 0.9, 40.)
    gap, _ = fg.update(20., False, 254., 0.9, 46.)  # a one-frame 6 m (0.3 s) jitter moves the shown gap under 0.05 s
    self.assertLess(gap - 2., 0.05)

  def test_cut_in_shown_at_once(self):
    fg = FollowGap(self.DT)
    for _ in range(40):
      fg.update(20., True, 50., *NO_CAMERA)  # 2.5 s
    self.assertAlmostEqual(fg.update(20., True, 20., *NO_CAMERA)[0], 1.)  # a car cuts in at 20 m: 1.0 s at once

  def test_new_car_and_new_source_start_over(self):
    fg = FollowGap(self.DT)
    for _ in range(40):
      fg.update(20., False, 254., 0.9, 80.)  # 4 s on the camera
    self.assertAlmostEqual(fg.update(20., True, 30., 0.9, 80.)[0], 1.5)  # the factory unit's car: 1.5 s at once
    self.assertEqual(fg.update(20., False, 254., 0.1, 0.), (None, GapSource.none))  # car lost: a dash
    self.assertAlmostEqual(fg.update(20., True, 50., *NO_CAMERA)[0], 2.5)  # the next car starts fresh

  def test_low_speed_clears(self):
    fg = FollowGap(self.DT)
    fg.update(MPH_60, True, 43., *NO_CAMERA)
    self.assertEqual(fg.update(1., True, 5., *NO_CAMERA), (None, GapSource.none))


class TestFollowGapToggle(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    with open(VISUALS) as f:
      cls.tree = ast.parse(f.read())
    cls.defs = {}
    for node in ast.walk(cls.tree):
      if isinstance(node, ast.Dict) and any(isinstance(k, ast.Constant) and k.value == "RocketFuel" for k in node.keys):
        cls.defs = {k.value: v for k, v in zip(node.keys, node.values, strict=True)}

  def test_toggle_in_visuals_panel(self):
    self.assertIn(PARAM, self.defs)
    title, desc, _ = self.defs[PARAM].elts
    self.assertEqual(title.body.args[0].value, "Following Gap in Seconds")
    text = joined(desc.args[0])
    for words in ("in seconds at your current speed", "divided by your speed", "below the MAX speed",
                  "factory windshield unit", "FACTORY", "comma camera", "CAMERA", "below 5 mph",
                  "nothing is sent to the car"):
      self.assertIn(words, text)

  def test_setting_defaults_off(self):
    with open(PARAMS_KEYS) as f:
      keys = f.read()
    m = re.search(r'\{"' + PARAM + r'", \{([^}]*)\}\}', keys)
    self.assertIsNotNone(m)
    self.assertEqual([a.strip() for a in m.group(1).split(",")], ["PERSISTENT | BACKUP", "BOOL", '"0"'])

  def test_screen_reads_the_setting(self):
    with open(UI_STATE) as f:
      self.assertIn(f'self.show_following_gap = self.params.get_bool("{PARAM}")', f.read())
    with open(HUD) as f:
      hud = f.read()
    self.assertIn("self.follow_gap_renderer.update()", hud)
    self.assertIn("self.follow_gap_renderer.render(rect)", hud)


if __name__ == "__main__":
  unittest.main()
