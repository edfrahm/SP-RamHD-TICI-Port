"""Planned-speed target for Intelligent Cruise Button Management on the Ram Heavy Duty (Custom Ram ACC Tuning,
edfrahm/SP-RamHD-TICI-Port#11): the rules of planned_speed.py, a replay of the 2026-10-19 near-miss, and how the
planner hands the target to the cruise buttons (toggle, ceiling, soft-disable).

The replay (data/planned_speed_replay_00000783.json.gz, made by data/make_planned_speed_replay.py) is route
00000783--e3bae4dd6c from 14:48:25 to 14:48:54: what the planner published (its speed plan, which includes the car
ahead the camera sees) and what it saw (the car ahead, the truck's speed, the set speed), 20 times a second. At
14:48:31 the truck's cruise control sped up from 52 to 73 mph toward the 75 mph set speed, while the camera showed
slower traffic about 100 m ahead at about 60 mph; Intelligent Cruise Button Management's target stayed at 75.
"""
import gzip
import json
import os
import time
import unittest

import numpy as np

from opendbc.car import structs
from openpilot.cereal import custom, messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib import planned_speed as ps
from openpilot.sunnypilot.selfdrive.controls.lib.planned_speed import PlannedSpeed

MPH = CV.MPH_TO_MS
REPLAY = os.path.join(os.path.dirname(__file__), "data", "planned_speed_replay_00000783.json.gz")
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


def flat_plan(v, a=0.):
  return [v + a * t for t in np.linspace(0., 2.5, 17)], [a] * 17


def run(p, n, v_ego, speeds, accels, allow_throttle=True, lead=(False, 0., 0.), **kw):
  out = 0.
  for _ in range(n):
    out = p.update(v_ego, speeds, accels, allow_throttle, *lead, **kw)
  return out


class TestPlannedSpeedRules(unittest.TestCase):
  def test_plan_average_plus_look_ahead(self):
    """jvePilot: the average of the plan plus 4 s of the average planned acceleration"""
    p = PlannedSpeed()
    speeds, accels = flat_plan(25., 0.25)
    self.assertAlmostEqual(run(p, 1, 25., speeds, accels), np.mean(speeds) + 1., places=3)

  def test_look_ahead_capped_at_10_mph(self):
    p = PlannedSpeed()
    speeds, accels = flat_plan(25., 2.)
    self.assertAlmostEqual(run(p, 1, 25., speeds, accels), np.mean(speeds) + 10 * MPH, places=3)
    p = PlannedSpeed()
    speeds, accels = flat_plan(25., -2.)
    self.assertAlmostEqual(run(p, 1, 25., speeds, accels), 25. - 10 * MPH, places=3)  # also the deepest drop

  def test_coast_when_the_plan_says_so_and_on_soft_disable(self):
    for kw in ({"allow_throttle": False}, {"force_decel": True}):
      p = PlannedSpeed()
      speeds, accels = flat_plan(30.)
      self.assertAlmostEqual(run(p, 1, 30., speeds, accels, **kw), 30. - 4 * MPH, places=3)

  def test_never_more_than_10_mph_below_the_truck(self):
    p = PlannedSpeed()
    speeds, accels = flat_plan(30.)
    self.assertAlmostEqual(run(p, 1, 30., speeds, accels, lead=(True, 10., 5.)), 30. - 10 * MPH, places=3)

  def test_rises_at_once_drops_only_by_0_75_mph(self):
    p = PlannedSpeed()
    run(p, 1, 25., *flat_plan(25.))
    self.assertAlmostEqual(run(p, 1, 25., *flat_plan(25. - 0.5 * MPH)), 25., places=3)
    self.assertAlmostEqual(run(p, 1, 25., *flat_plan(25. - 1.0 * MPH)), 25. - 1.0 * MPH, places=3)
    self.assertAlmostEqual(run(p, 1, 25., *flat_plan(25. + 0.2 * MPH)), 25. + 0.2 * MPH, places=3)

  def test_reset_takes_the_new_value(self):
    p = PlannedSpeed()
    run(p, 1, 25., *flat_plan(25.))
    p.reset()
    self.assertAlmostEqual(run(p, 1, 25., *flat_plan(25. - 0.5 * MPH)), 25. - 0.5 * MPH, places=3)

  def test_starting_correction_grows_with_distance(self):
    """Before the truck has learned anything: 90% of the measured values, nothing under 15 m, held beyond 70 m"""
    self.assertEqual(ps.lead_speed_correction(10.), 0.)
    self.assertAlmostEqual(ps.lead_speed_correction(37.5) / MPH, 0.9 * 1.4, places=3)
    self.assertAlmostEqual(ps.lead_speed_correction(52.5) / MPH, 0.9 * 2.3, places=3)
    self.assertAlmostEqual(ps.lead_speed_correction(110.) / MPH, 0.9 * 2.7, places=3)

  def test_car_ahead_caps_by_time_gap(self):
    """At most the car's speed within a 2 s gap, up to 5 mph above it at 4 s or more"""
    speeds, accels = flat_plan(30., 1.)  # the plan alone would ask for far more
    for gap_s, margin in ((1.5, 0.), (2., 0.), (3., 2.5 * MPH), (4., 5 * MPH), (6., 5 * MPH)):
      p = PlannedSpeed()
      target = run(p, 1, 26., speeds, accels, lead=(True, 26. * gap_s, 27. - ps.lead_speed_correction(26. * gap_s)))
      self.assertAlmostEqual(target, 27. + margin, places=3)

  def test_behind_a_slower_car_never_above_the_truck(self):
    """Far enough back to close in (4 s), but the car is slower: hold the truck's speed, don't speed up toward it"""
    p = PlannedSpeed()
    speeds, accels = flat_plan(30., 1.)
    target = run(p, 1, 28., speeds, accels, lead=(True, 112., 27. - ps.lead_speed_correction(112.)))
    self.assertAlmostEqual(target, 28., places=3)

  def test_car_ahead_does_not_raise_the_target(self):
    p = PlannedSpeed()
    speeds, accels = flat_plan(25.)
    self.assertAlmostEqual(run(p, 1, 25., speeds, accels, lead=(True, 60., 35.)), 25., places=3)

  def test_car_ahead_speed_corrected(self):
    """The planner's speed for the car ahead reads low, more the farther the car (about 1.9 mph at 45 m): in steady
    following within 2 s, the target is the truck's own speed, not below it (which would make the truck fall back)"""
    p = PlannedSpeed()
    speeds, accels = flat_plan(27.)
    self.assertAlmostEqual(run(p, 1, 27., speeds, accels, lead=(True, 45., 27. - ps.lead_speed_correction(45.))), 27., places=3)


class TestDistantCarMemory(unittest.TestCase):
  def setUp(self):
    self.p = PlannedSpeed()
    self.speeds, self.accels = flat_plan(31., 0.5)  # an empty road: the plan speeds up

  def see(self, seconds, d, v, v_ego=29.):
    run(self.p, int(seconds / ps.DT_MDL), v_ego, self.speeds, self.accels, lead=(True, d, v))

  def lose(self, seconds, v_ego=29., **kw):
    return run(self.p, int(seconds / ps.DT_MDL), v_ego, self.speeds, self.accels, **kw)

  def test_remembered_after_fading_far_ahead(self):
    self.see(2., 105., 27.)
    self.assertAlmostEqual(self.lose(5.), 27. + ps.lead_speed_correction(105.), places=2)
    self.assertTrue(self.p.memory)

  def test_forgotten_after_10_s(self):
    self.see(2., 140., 28.9)
    self.lose(10.2)
    self.assertFalse(self.p.memory)
    self.assertGreater(self.p.v_target, 31.)

  def test_not_remembered_when_lost_close(self):
    """A near car that vanishes has usually left the lane"""
    self.see(2., 40., 27.)
    self.lose(0.5)
    self.assertFalse(self.p.memory)

  def test_not_remembered_when_seen_too_briefly(self):
    self.see(0.5, 105., 27.)
    self.lose(0.5)
    self.assertFalse(self.p.memory)

  def test_forgotten_at_a_lane_change(self):
    self.see(2., 105., 27.)
    self.lose(1.)
    self.assertTrue(self.p.memory)
    self.lose(0.05, lane_change=True)
    self.assertFalse(self.p.memory)

  def test_forgotten_once_the_truck_would_be_close(self):
    """Closing at 4 m/s from 70 m: within 40 m after 7.5 s, where the camera would surely see it"""
    self.see(2., 70., 25. - ps.lead_speed_correction(70.), v_ego=29.)
    self.lose(7., v_ego=29.)
    self.assertTrue(self.p.memory)
    self.lose(1., v_ego=29.)
    self.assertFalse(self.p.memory)

  def test_a_one_frame_return_keeps_it(self):
    self.see(2., 105., 27.)
    self.lose(1.)
    self.see(0.05, 110., 27.5)
    self.lose(1.)
    self.assertTrue(self.p.memory)


class TestLaneChangeUnchanged(unittest.TestCase):
  """Eric, 2026-10-20: no lane-change rules until the Comma detects traffic in the next lane. A turn signal changes
  nothing by itself: the target is the same with and without it (the remembered far car aside)."""

  def test_signal_changes_nothing(self):
    for lead in ((False, 0., 0.), (True, 43., 24.6), (True, 87., 24.6)):
      a, b = PlannedSpeed(), PlannedSpeed()
      plain = run(a, 30, 29., *flat_plan(31., 0.5), lead=lead)
      signalled = run(b, 30, 29., *flat_plan(31., 0.5), lead=lead, lane_change=True)
      self.assertAlmostEqual(plain, signalled, places=6)


class TestReplay00000783(unittest.TestCase):
  """14:48:31-14:48:53: the target stays near the car ahead's speed (about 60 mph), not at the 75 mph set speed"""

  @classmethod
  def setUpClass(cls):
    with gzip.open(REPLAY, "rt") as f:
      cls.rows = json.load(f)["rows"]
    p = PlannedSpeed()
    for r in cls.rows:
      target = p.update(r["vEgo"], r["speeds"], r["accels"], r["allowThrottle"], r["leadPresent"], r["leadD"], r["leadV"],
                        r["signal"] or r["laneChangeState"] != "off", r["forceDecel"])
      r["new"] = min(target, r["vCruise"] * CV.KPH_TO_MS) / MPH
      r["memory"] = p.memory
    cls.window = [r for r in cls.rows if "14:48:31" <= r["clock"] <= "14:48:53.00"]

  def test_today_the_target_was_the_75_mph_set_speed(self):
    self.assertTrue(all(abs(r["icbmTarget"] / MPH - 74.6) < 0.5 for r in self.window))

  def test_target_stays_well_below_75(self):
    """At most about 69 mph, where the corrected camera reading put the car ahead slightly faster than the truck and
    3.4 s ahead (so the gap rule lets the truck close in); about 65 mph on average"""
    new = [r["new"] for r in self.window]
    self.assertLess(max(new), 70.)
    self.assertTrue(62. < np.mean(new) < 67., np.mean(new))

  def test_target_near_the_car_ahead(self):
    """With the camera's car ahead: within 6 mph of its speed (averaged over 1 s; a single reading jumps by up to
    7 mph), and never above the truck's speed while that car is slower"""
    with_lead = [(i, r) for i, r in enumerate(self.rows) if r in self.window and r["leadPresent"] and r["clock"] >= "14:48:33"]
    self.assertGreater(len(with_lead), 200)
    for i, r in with_lead:
      seen = [x["leadV"] + ps.lead_speed_correction(x["leadD"]) for x in self.rows[max(0, i - 19):i + 1] if x["leadPresent"]]
      if len(seen) < 10:
        continue  # a car back for a frame or two after the camera lost it (14:48:52.75)
      lead_mph = np.mean(seen) / MPH
      self.assertLess(r["new"] - lead_mph, 6., r["clock"])
      if lead_mph < r["vEgo"] / MPH - 1.:  # a slower car: never told to speed up toward it (a held drop aside)
        self.assertLessEqual(r["new"], r["vEgo"] / MPH + ps.HYSTERESIS / MPH, r["clock"])

  def test_camera_loses_the_car_the_memory_holds(self):
    """14:48:45.5-14:48:53: the camera still placed the car about 112 m ahead at about 61 mph, under the planner's
    confidence; the plan saw an empty road and asked for 75. The memory keeps the target near the car's last speed,
    below the truck's speed (coast)."""
    lost = [r for r in self.window if r["clock"] >= "14:48:46" and not r["leadPresent"]]
    self.assertGreater(len(lost), 120)
    self.assertTrue(all(r["memory"] for r in lost))
    self.assertGreater(max(np.mean(r["speeds"]) / MPH for r in lost), 74.)  # the plan itself went back up to 75
    for r in lost:
      self.assertLess(r["new"], 67., r["clock"])
      self.assertLess(r["new"], r["vEgo"] / MPH, r["clock"])


class TestPlannerHandsTargetToButtons(OpenpilotTestCase):
  """LongitudinalPlannerSP.update_planned_speed: only on the Ram Heavy Duty with the toggle on; the driver's set speed
  stays the ceiling; a soft-disable coasts instead of asking for 0"""

  def make(self, fingerprint="RAM_HD_5TH_GEN", toggle=True):
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
    Params().put_bool("RamAccTuning", toggle)
    CP = structs.CarParams(carFingerprint=fingerprint, openpilotLongitudinalControl=False, pcmCruise=True)
    CP_SP = structs.CarParamsSP(pcmCruiseSpeed=False, intelligentCruiseButtonManagementAvailable=True)
    planner = LongitudinalPlannerSP(CP, CP_SP, None)
    planner.output_v_target, planner.source = 33., LongitudinalPlanSource.cruise
    return planner

  def sm(self, v_ego=26.8, v_cruise_mph=75., force_decel=False, lead=None):
    msgs = {}
    for name in ("carState", "carControl", "radarState", "modelV2", "controlsState"):
      msgs[name] = messaging.new_message(name)
    cs = msgs["carState"].carState
    cs.vEgo, cs.vCruise = v_ego, v_cruise_mph * CV.MPH_TO_KPH
    msgs["carControl"].carControl.enabled = True
    msgs["controlsState"].controlsState.forceDecel = force_decel
    if lead is not None:
      l = msgs["radarState"].radarState.leadOne
      l.present, l.dRel, l.vLead = True, lead[0], lead[1]
    return {k: getattr(v, k) for k, v in msgs.items()}

  def test_off_leaves_todays_target(self):
    planner = self.make(toggle=False)
    planner.update_planned_speed(self.sm(lead=(100., 26.8)), *flat_plan(30., 0.5), True)
    self.assertEqual((planner.icbm_v_target, planner.icbm_source), (33., LongitudinalPlanSource.cruise))

  def test_other_cars_never_use_it(self):
    planner = self.make(fingerprint="RAM_1500_5TH_GEN")
    self.assertIsNone(planner.planned_speed)
    planner.update_planned_speed(self.sm(lead=(30., 20.)), *flat_plan(30.), True)
    self.assertEqual(planner.icbm_v_target, 33.)

  def test_on_follows_the_car_ahead(self):
    planner = self.make()
    planner.update_planned_speed(self.sm(v_ego=29., lead=(45., 26.8)), *flat_plan(31., 0.5), True)
    self.assertAlmostEqual(planner.icbm_v_target, 26.8 + ps.lead_speed_correction(45.), places=2)
    self.assertEqual(planner.icbm_source, LongitudinalPlanSource.plannedSpeed)

  def test_driver_set_speed_stays_the_ceiling(self):
    planner = self.make()
    planner.update_planned_speed(self.sm(v_ego=26., v_cruise_mph=60.), *flat_plan(28., 1.), True)
    self.assertAlmostEqual(planner.icbm_v_target, 60 * MPH, places=2)
    self.assertEqual(planner.icbm_source, LongitudinalPlanSource.cruise)

  def test_camera_correction_learns_with_the_toggle_off_and_is_kept(self):
    """It only observes, so it learns from steady following whether or not the feature is on, and saves the figures"""
    planner = self.make(toggle=False)
    sm = self.sm(v_ego=29., lead=(40., 29. - 3. * MPH))
    for _ in range(int(60. / ps.DT_MDL) + 1):
      planner.update_planned_speed(sm, *flat_plan(29.), True)
    self.assertEqual(planner.lead_cal.counts[1], 12)
    self.assertGreater(planner.lead_cal.values[1], 1.4 * MPH)
    saved = None
    for _ in range(100):  # the planner saves without waiting for the disk
      if (saved := Params().get("RamLeadSpeedCalibration")) is not None:
        break
      time.sleep(0.01)
    self.assertIsNotNone(saved)
    self.assertEqual(saved["counts"][1], 12)
    self.assertEqual(planner.icbm_v_target, 33.)  # and the cruise buttons' target stays today's

  def test_soft_disable_coasts_instead_of_zero(self):
    """Today the planner's cruise speed is 0 during a soft-disable and the buttons hold - toward 0"""
    planner = self.make()
    planner.output_v_target = 0.
    planner.update_planned_speed(self.sm(v_ego=30.8, force_decel=True), *flat_plan(30.8, -1.), True)
    self.assertAlmostEqual(planner.icbm_v_target, 30.8 - 4 * MPH, places=2)
    self.assertEqual(planner.icbm_source, LongitudinalPlanSource.plannedSpeed)


if __name__ == "__main__":
  unittest.main()
