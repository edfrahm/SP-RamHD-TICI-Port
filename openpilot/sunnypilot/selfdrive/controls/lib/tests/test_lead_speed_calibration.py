"""The camera correction the truck learns from its own steady following (lead_speed_calibration.py,
edfrahm/SP-RamHD-TICI-Port#11): only steady stretches count, it learns slowly and per distance band, at most 0.5 mph
per drive, starts over for another driving model, and is kept between drives."""
import unittest

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib import lead_speed_calibration as lsc
from openpilot.sunnypilot.selfdrive.controls.lib.lead_speed_calibration import LeadSpeedCalibration

MPH = CV.MPH_TO_MS
DT = lsc.DT_MDL


def follow(cal, seconds, gap=40., v=29., reads_slow_mph=3., gap_rate=0., lane_change=False):
  """Steady following: the car ahead truly at the truck's speed plus gap_rate, the camera reading it slow"""
  learned = 0
  for i in range(int(round(seconds / DT))):
    d = gap + gap_rate * i * DT
    learned += cal.update(v, True, d, v + gap_rate - reads_slow_mph * MPH, lane_change)
  return learned


class TestLeadSpeedCalibration(unittest.TestCase):
  def test_learns_slowly_toward_the_reading(self):
    cal = LeadSpeedCalibration("m")
    self.assertEqual(follow(cal, 5.), 1)
    self.assertAlmostEqual(cal.values[1] / MPH, 1.4 + (3. - 1.4) / 50, places=3)

  def test_at_most_half_a_mph_per_drive(self):
    cal = LeadSpeedCalibration("m")
    follow(cal, 3600.)
    self.assertAlmostEqual(cal.values[1] / MPH, 1.9, places=3)
    self.assertEqual(cal.values[0], lsc.PRIOR[0])  # other bands untouched

  def test_learns_per_distance_band(self):
    cal = LeadSpeedCalibration("m")
    follow(cal, 5., gap=55.)
    self.assertNotEqual(cal.values[2], lsc.PRIOR[2])
    self.assertEqual(cal.values[1], lsc.PRIOR[1])

  def test_gap_changing_is_not_steady(self):
    """The car ahead pulling away at 0.5 m/s: its real speed differs from the truck's, so the reading is not used"""
    cal = LeadSpeedCalibration("m")
    self.assertEqual(follow(cal, 20., gap_rate=0.5), 0)
    self.assertEqual(cal.values, lsc.PRIOR)

  def test_steady_but_not_perfectly_counts_the_gap_trend(self):
    """Gap growing 0.2 m/s with the camera reading 3 mph slow of the truck: it really reads 3 mph slow of the car"""
    cal = LeadSpeedCalibration("m")
    cal.values[1] = 3. * MPH
    cal.drive_start[1] = 3. * MPH
    follow(cal, 5., gap_rate=0.2)
    self.assertAlmostEqual(cal.values[1] / MPH, 3., places=2)

  def test_ignored_during_lane_changes_and_breaks(self):
    cal = LeadSpeedCalibration("m")
    self.assertEqual(follow(cal, 20., lane_change=True), 0)
    follow(cal, 4.)
    cal.update(29., False, 0., 0., False)  # the car ahead out of view for a frame: the stretch starts over
    self.assertEqual(follow(cal, 4.), 0)

  def test_slow_or_far_or_odd_readings_ignored(self):
    cal = LeadSpeedCalibration("m")
    self.assertEqual(follow(cal, 10., v=12.), 0)  # under 34 mph
    self.assertEqual(follow(cal, 10., gap=90.), 0)  # beyond 80 m
    self.assertEqual(follow(cal, 10., reads_slow_mph=12.), 0)  # outside the plausible range

  def test_applies_90_percent_and_never_more_than_4_mph(self):
    cal = LeadSpeedCalibration("m")
    self.assertAlmostEqual(cal.correction(37.5), 0.9 * lsc.PRIOR[1], places=4)
    cal.values = [9. * MPH] * 4
    self.assertAlmostEqual(cal.correction(70.) / MPH, 4., places=3)
    self.assertEqual(cal.correction(10.), 0.)


class TestLeadSpeedCalibrationKept(OpenpilotTestCase):
  def test_kept_between_drives_for_the_same_model(self):
    cal = LeadSpeedCalibration("model-a")
    follow(cal, 60.)
    Params().put(lsc.PARAM, cal.to_param(), block=True)
    again = LeadSpeedCalibration("model-a", Params().get(lsc.PARAM))
    self.assertEqual(again.values, [round(v, 4) for v in cal.values])
    self.assertEqual(again.counts, cal.counts)
    self.assertEqual(again.drive_start, again.values)  # a new drive's 0.5 mph limit counts from here

  def test_damaged_setting_falls_back_to_the_starting_values(self):
    for saved in ({"model": "m", "values": ["x", 1, 2, 3]}, {"model": "m", "values": [float("nan")] * 4},
                  {"model": "m", "values": [1., 2.]}, {"model": "m", "values": [50.] * 4}, {"model": "m"}, "junk", None):
      self.assertEqual(LeadSpeedCalibration("m", saved).values, lsc.PRIOR, saved)

  def test_another_driving_model_starts_over(self):
    cal = LeadSpeedCalibration("model-a")
    follow(cal, 60.)
    Params().put(lsc.PARAM, cal.to_param(), block=True)
    self.assertEqual(LeadSpeedCalibration("model-b", Params().get(lsc.PARAM)).values, lsc.PRIOR)


if __name__ == "__main__":
  unittest.main()
