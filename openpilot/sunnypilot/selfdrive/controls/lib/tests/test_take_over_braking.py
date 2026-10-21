"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Tests for the take-over-braking alert (edfrahm/SP-RamHD-TICI-Port#28), on made-up drives and on the recorded near miss
of 2026-10-19 (route 00000783).
"""
import gzip
import json
import os
import unittest

from opendbc.car import structs
from openpilot.cereal import custom, messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib import take_over_braking as tob
from openpilot.sunnypilot.selfdrive.controls.lib.lead_speed_calibration import LeadSpeedCalibration
from openpilot.sunnypilot.selfdrive.controls.lib.take_over_braking import G, MPH, MarginInputs, Source, TakeOverBraking

REPLAY = os.path.join(os.path.dirname(__file__), "data", "take_over_braking_replay_00000783.json.gz")
DT = 0.05
EventNameSP = custom.OnroadEventSP.EventName
NO_CAMERA = (False, 0., 0., 0.)


def alert_with_prior():
  return TakeOverBraking(LeadSpeedCalibration(""))


class Drive:
  """A made-up drive, 20 frames a second: the truck and one car ahead, each at a speed with a steady slowing"""
  def __init__(self, v=31.3, lead_v=31.3, gap=60., cruise=True, steer=True, set_speed=None):
    self.alert = alert_with_prior()
    self.v, self.lead_v, self.gap = v, lead_v, gap
    self.truck_slowing, self.lead_slowing = 0., 0.
    self.cruise, self.steer = cruise, steer
    self.set_speed = set_speed if set_speed is not None else v + 5.
    self.t = 0.
    self.log = []

  def step(self, camera=True, unit=False, prob=0.99, lane_change=False, camera_override=None, unit_override=None):
    """One frame. The camera reports the car where it is (or camera_override = (present, d, v, prob)); the unit, when
    asked and engaged, reports it too (or unit_override = (present, d, v))."""
    cam = (True, self.gap, self.lead_v, prob) if camera else NO_CAMERA
    if camera_override is not None:
      cam = camera_override
    un = (unit and self.cruise, self.gap, min(self.lead_v, self.set_speed)) if unit_override is None else unit_override
    active = self.alert.update(self.v, self.cruise, self.steer, self.set_speed, lane_change, *cam, *un)
    self.log.append((self.t, active, self.alert.need, self.alert.source, self.gap, self.v))
    self.v = max(self.v - self.truck_slowing * DT, 0.)
    self.lead_v = max(self.lead_v - self.lead_slowing * DT, 0.)
    self.gap += (self.lead_v - self.v) * DT
    self.t += DT
    return active

  def run(self, seconds, **kw):
    return [self.step(**kw) for _ in range(int(round(seconds / DT)))]


class TestRules(unittest.TestCase):
  def test_minimum_margin_with_the_placeholders(self):
    """65 mph behind a car at 65 mph: 1.5 s of travel, the truck's longer stop (0.6 g against 0.8 g) and 10 m: about
    71 m, a 2.5 s gap"""
    v = 65 * MPH
    m = tob.minimum_margin(MarginInputs(), v, v)
    self.assertAlmostEqual(m, v * 1.5 + v ** 2 / (2 * 0.6 * G) - v ** 2 / (2 * 0.8 * G) + 10., places=6)
    self.assertAlmostEqual(m / v, 2.46, places=2)

  def test_braking_needed_outside_the_margin(self):
    self.assertAlmostEqual(tob.braking_needed(100., 10., 60., 1.5), 100. / 80.)
    self.assertEqual(tob.braking_needed(100., -1., 60., 1.5), 0.)

  def test_room_never_less_than_the_reaction_closing(self):
    """At or inside the margin the need is the closing speed over twice the reaction time, so it is above 0.3 g only
    when closing faster than about 20 mph"""
    self.assertAlmostEqual(tob.braking_needed(50., 9., 60., 1.5), 9. / 3.)
    limit_closing = 2 * 1.5 * tob.CRUISE_LIMIT_G * G
    self.assertAlmostEqual(limit_closing / MPH, 19.7, places=1)
    self.assertLess(tob.braking_needed(50., limit_closing - 0.1, 60., 1.5), tob.CRUISE_LIMIT_G * G)
    self.assertGreater(tob.braking_needed(50., limit_closing + 0.1, 60., 1.5), tob.CRUISE_LIMIT_G * G)
    # without the floor (the rule as first written), inside the margin while closing at all is more than any limit
    self.assertEqual(tob.braking_needed(50., 0.5, 60.), float('inf'))

  def test_every_number_in_one_place(self):
    self.assertEqual(MarginInputs(), MarginInputs(reaction_s=1.5, truck_braking_g=0.6, lead_braking_g=0.8, buffer_m=10.))
    self.assertEqual((tob.CRUISE_LIMIT_G, tob.CONFIRM_S, tob.END_S, tob.MIN_START_SPEED / MPH), (0.3, 0.3, 0.5, 40.))
    self.assertEqual((tob.CAMERA_ONLY_MAX_DIST, tob.CAMERA_ONLY_MIN_PROB), (60., 0.5))


class TestMadeUpDrives(unittest.TestCase):
  def test_fast_approach_to_slow_traffic_alerts(self):
    """70 mph toward traffic at 30 mph, 150 m ahead, nothing braking: it alerts once 0.3 g is needed for 0.3 s"""
    d = Drive(v=70 * MPH, lead_v=30 * MPH, gap=150.)
    d.run(6.)
    first = next(i for i, x in enumerate(d.log) if x[1])
    over = next(i for i, x in enumerate(d.log) if x[2] > 0.3 * G)
    self.assertEqual(first - over, int(round(tob.CONFIRM_S / DT)) - 1)
    self.assertGreater(d.log[first][4], 60.)  # well before the car

  def test_ordinary_following_inside_the_margin_is_quiet(self):
    """Following at 1.5 s, the factory cruise control's own gap and inside the 2.5 s margin, while the speeds wobble by
    2 mph either way (the rule as first written alerted on every such wobble)"""
    d = Drive(v=65 * MPH, lead_v=65 * MPH, gap=1.5 * 65 * MPH)
    for i in range(int(60. / DT)):
      d.lead_v = 65 * MPH + 2 * MPH * (1 if (i // 40) % 2 else -1)
      self.assertFalse(d.step(unit=True))

  def test_driver_braking_does_not_silence_it(self):
    """Once alerting, the driver brakes lightly: the factory cruise control disengages (its unit's car goes), sunnypilot
    steering is off and the car is beyond 60 m; the alert keeps going while the truck slows less than needed"""
    d = Drive(v=72 * MPH, lead_v=20 * MPH, gap=130., steer=False)
    d.lead_slowing = 2.
    while not d.step(unit=True):
      self.assertLess(d.t, 10.)
    self.assertGreater(d.gap, 60.)
    d.cruise = False  # the brake pedal disengages the factory cruise control
    d.truck_slowing = 0.15 * G
    for active in d.run(2.):
      self.assertTrue(active)
    self.assertGreater(d.alert.need, d.alert.slowing)

  def test_factory_braking_does_not_silence_it(self):
    """The factory cruise control brakes at its 0.3 g, which is not enough: the alert stays on"""
    d = Drive(v=72 * MPH, lead_v=10 * MPH, gap=120.)
    while not d.step(unit=True):
      self.assertLess(d.t, 10.)
    d.truck_slowing = 0.3 * G
    for active in d.run(2.):
      self.assertTrue(active)
    self.assertGreater(d.alert.need, 0.3 * G)

  def test_ends_when_the_trucks_slowing_meets_the_need(self):
    d = Drive(v=60 * MPH, lead_v=20 * MPH, gap=100.)
    while not d.step(unit=True):
      self.assertLess(d.t, 10.)
    d.truck_slowing = 0.75 * G
    d.cruise = False
    states = d.run(2.)
    self.assertFalse(states[-1])
    self.assertLess(states.index(False) * DT, tob.SLOWING_WINDOW_S + tob.END_S + 0.1)

  def test_ends_when_the_need_passes(self):
    d = Drive(v=70 * MPH, lead_v=40 * MPH, gap=130.)
    while not d.step():
      self.assertLess(d.t, 10.)
    d.lead_v = 72 * MPH  # the car ahead speeds away
    states = d.run(1.)
    self.assertFalse(states[-1])
    self.assertEqual(states.index(False) + 1, int(round(tob.END_S / DT)))  # half a second with no need

  def test_nothing_engaged_camera_only_within_60_m(self):
    """Nothing engaged: a car 70 m ahead that needs hard braking does not alert until it is within 60 m"""
    d = Drive(v=55 * MPH, lead_v=0., gap=75., cruise=False, steer=False)
    d.run(1.5)
    beyond = [x for x in d.log if x[4] > 60.]
    self.assertTrue(beyond and all(not x[1] and x[3] == Source.none for x in beyond))
    first = next(i for i, x in enumerate(d.log) if x[1])
    within = next(i for i, x in enumerate(d.log) if x[4] <= 60.)
    self.assertEqual(first - within, int(round(tob.CONFIRM_S / DT)) - 1)  # from the first frame within 60 m

  def test_engaged_the_camera_counts_beyond_60_m(self):
    for cruise, steer in ((False, True), (True, False)):
      d = Drive(v=55 * MPH, lead_v=0., gap=75., cruise=cruise, steer=steer)
      d.run(0.5)
      self.assertTrue(d.log[-1][1])
      self.assertGreater(d.log[-1][4], 60.)

  def test_nothing_engaged_needs_the_camera_50_percent_sure(self):
    d = Drive(v=55 * MPH, lead_v=0., gap=55., cruise=False, steer=False)
    self.assertFalse(any(d.run(1., prob=0.45)))
    self.assertTrue(d.run(0.5, prob=0.55)[-1])

  def test_single_bad_camera_frames_rejected(self):
    """Steady following; the camera places a much nearer, slower car for a few frames (a car switching, a bad frame).
    Up to 5 frames (0.25 s) is ignored; 6 frames (0.3 s) alerts: the confirmation costs 0.3 s of warning."""
    for bad_frames, alerts in ((1, False), (3, False), (5, False), (6, True)):
      d = Drive(v=65 * MPH, lead_v=65 * MPH, gap=45.)
      d.run(2.)
      bad = (True, 15., 30 * MPH, 0.9)
      seen = [d.step(camera_override=bad) for _ in range(bad_frames)]
      seen += d.run(1.)
      self.assertEqual(any(seen), alerts, bad_frames)

  def test_starts_only_at_40_mph(self):
    """35 mph toward a stopped car, braking gently as at a traffic light: no alert, though the margin calls for more"""
    d = Drive(v=35 * MPH, lead_v=0., gap=55.)
    d.truck_slowing = 0.2 * G
    states = d.run(4.)
    self.assertTrue(any(x[2] > 0.3 * G for x in d.log))
    self.assertFalse(any(states))

  def test_started_above_40_mph_runs_on_below_it(self):
    d = Drive(v=45 * MPH, lead_v=0., gap=55.)
    while not d.step():
      self.assertLess(d.t, 5.)
    d.truck_slowing = 0.25 * G
    d.run(2.)
    self.assertLess(d.v, 40 * MPH)
    self.assertTrue(d.log[-1][1])


class TestFactoryUnit(unittest.TestCase):
  def test_the_units_car_comes_first(self):
    """The unit reports a slower car at 120 m while the camera has a car at the truck's speed nearer: the unit's counts"""
    d = Drive(v=72 * MPH, lead_v=50 * MPH, gap=120.)
    d.run(3., unit=True, camera_override=(True, 60., 72 * MPH, 0.9))
    self.assertEqual(d.alert.source, Source.factoryUnit)
    self.assertTrue(d.log[-1][1])

  def test_speed_at_the_cap_comes_from_the_distance_trend(self):
    """The unit's speed reads the set speed (the car is faster, or the reading is capped) while its distance shrinks at
    5 m/s: after a second of readings, the car's speed is the truck's minus 5 m/s"""
    v, set_speed = 30., 28.
    unit = tob.FactoryUnitLead()
    for i in range(30):
      unit.update(v, True, 120. - 5. * i * DT, set_speed, set_speed, False)
    self.assertTrue(unit.speed_from_trend)
    self.assertAlmostEqual(unit.v, v - 5., places=3)

  def test_a_short_cap_reading_keeps_the_last_real_speed(self):
    """14:48:58 on route 00000783: the unit read the 75 mph set speed for a stopping car 95 m ahead for one frame"""
    unit = tob.FactoryUnitLead()
    set_speed = 121 * CV.KPH_TO_MS
    for i in range(5):
      unit.update(27., True, 97. - i, 41 * CV.KPH_TO_MS, set_speed, False)
    unit.update(27., True, 92., 120 * CV.KPH_TO_MS, set_speed, False)
    self.assertAlmostEqual(unit.v, 41 * CV.KPH_TO_MS)
    self.assertFalse(unit.speed_from_trend)

  def test_a_dropout_is_held_2_s(self):
    unit = tob.FactoryUnitLead()
    for _ in range(20):
      unit.update(30., True, 80., 25., 35., False)
    held = [unit.update(30., False, 254., 35., 35., False) for _ in range(int(2.5 / DT))]
    self.assertTrue(all(held[:int(2. / DT)]))
    self.assertFalse(held[-1])
    self.assertAlmostEqual(unit.d, 80. - 5. * 2. - 5. * DT, places=1)  # reckoned from both speeds while held

  def test_a_held_car_is_dropped_at_a_lane_change(self):
    unit = tob.FactoryUnitLead()
    for _ in range(20):
      unit.update(30., True, 80., 25., 35., False)
    unit.update(30., False, 254., 35., 35., False)
    self.assertFalse(unit.update(30., False, 254., 35., 35., True))

  def test_a_jump_is_another_car(self):
    """Drive 0000076a: at the cap the unit read 136 m, then 114 m for a quarter second, then 137 m. That is not the same
    car, so its distance trend is not read as a car closing at 60 mph"""
    d = Drive(v=80 * MPH, lead_v=82 * MPH, gap=136., set_speed=80 * MPH)
    d.run(3., unit=True, camera=False)
    cap = 80 * MPH
    seen = [d.step(camera=False, unit_override=(True, 114., cap)) for _ in range(5)]
    seen += [d.step(camera=False, unit_override=(True, 137., cap)) for _ in range(20)]
    self.assertFalse(any(seen))
    self.assertLess(d.alert.closing, 1 * MPH)


class TestCameraCorrection(unittest.TestCase):
  def test_applied_in_steady_following_faded_when_closing(self):
    a = alert_with_prior()
    full = a.calibration.correction(50.)
    self.assertGreater(full, 0.)
    self.assertAlmostEqual(a.camera_speed(25., 50., 25. - full), 25.)  # steady: corrected in full
    self.assertAlmostEqual(a.camera_speed(25., 50., 20.), 20.)  # closing 5 m/s: not corrected


class TestReplay00000783(unittest.TestCase):
  """2026-10-19, 14:48:20-14:49:05: the truck's cruise sped toward traffic that was stopping; Eric braked at 14:49:00.29"""

  @classmethod
  def setUpClass(cls):
    with gzip.open(REPLAY, "rt") as f:
      cls.rows = json.load(f)["rows"]
    a = alert_with_prior()
    for r in cls.rows:
      lane_change = r["signal"] or r["laneChangeState"] != "off"
      r["alert"] = a.update(r["vEgo"], r["cruise"], r["steering"], r["setSpeed"], lane_change,
                            r["camPresent"], r["camD"], r["camV"], r["camProb"],
                            r["cruise"] and r["unitD"] < 254, r["unitD"], r["unitKph"] * CV.KPH_TO_MS)
      r["need"], r["source"] = a.need, a.source
    cls.first = next(r for r in cls.rows if r["alert"])
    cls.braked = next(r for r in cls.rows if r["brake"])

  def test_eric_braked_at_14_49_00(self):
    self.assertEqual(self.braked["clock"], "14:49:00.29")

  def test_alerts_more_than_5_s_before_eric_braked(self):
    self.assertTrue("14:48:54" <= self.first["clock"] <= "14:48:55", self.first["clock"])
    self.assertGreater(float(self.braked["clock"][6:]) + 60. - float(self.first["clock"][6:]), 5.)
    self.assertEqual(self.first["source"], Source.factoryUnit)
    self.assertTrue(0.3 * G < self.first["need"] < 0.35 * G)

  def test_quiet_while_following_before(self):
    """The rule as first written alerted at 14:48:23 and 14:48:29, following inside the margin and closing at under
    1 mph"""
    self.assertFalse(any(r["alert"] for r in self.rows if r["clock"] < "14:48:54"))

  def test_eric_braking_does_not_end_it(self):
    after = [r for r in self.rows if "14:48:54.7" <= r["clock"] <= "14:49:00.8"]
    self.assertTrue(all(r["alert"] for r in after))
    self.assertTrue(any(r["brake"] and not r["cruise"] for r in after))  # his braking disengaged the factory cruise

  def test_ends_once_his_braking_meets_the_need(self):
    self.assertFalse(any(r["alert"] for r in self.rows if r["clock"] >= "14:49:02"))


class TestPlannerAddsTheAlert(OpenpilotTestCase):
  """LongitudinalPlannerSP: Ram Heavy Duty only, on unless turned off; the event goes to selfdrived, the readings to
  the log"""

  def make(self, fingerprint="RAM_HD_5TH_GEN", setting=None):
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
    if setting is not None:
      Params().put_bool(tob.PARAM, setting)
    CP = structs.CarParams(carFingerprint=fingerprint, openpilotLongitudinalControl=False, pcmCruise=True)
    CP_SP = structs.CarParamsSP(pcmCruiseSpeed=True)
    return LongitudinalPlannerSP(CP, CP_SP, None)

  def sm(self, v_ego, lead_d, lead_v, unit=False):
    msgs = {name: messaging.new_message(name) for name in ("carState", "radarState", "modelV2", "carStateSP",
                                                           "selfdriveStateSP")}
    cs = msgs["carState"].carState
    cs.vEgo, cs.cruiseState.enabled, cs.cruiseState.speed = v_ego, unit, v_ego + 3.
    lead = msgs["radarState"].radarState.leadOne
    lead.present, lead.dRel, lead.vLead, lead.modelProb = True, lead_d, lead_v, 0.99
    fl = msgs["carStateSP"].carStateSP.factoryLead
    fl.present, fl.dRel, fl.vLead = unit, lead_d, lead_v
    msgs["selfdriveStateSP"].selfdriveStateSP.mads.enabled = True
    return {k: getattr(v, k) for k, v in msgs.items()}

  def run_frames(self, planner, sm, n):
    for _ in range(n):
      planner.events_sp.clear()
      planner.update_take_over_braking(sm)
    return [e.name for e in planner.events_sp.to_msg()]

  def test_on_by_default(self):
    """The comma writes each setting's default when it starts (system/manager/manager.py)"""
    self.assertIs(Params().get_default_value(tob.PARAM), True)

  def test_adds_the_alert_and_logs_what_it_saw(self):
    planner = self.make(setting=True)
    events = self.run_frames(planner, self.sm(32., 80., 5., unit=True), 10)
    self.assertIn(EventNameSP.takeOverBraking, events)
    self.assertTrue(planner.take_over_braking.active)
    self.assertEqual(planner.take_over_braking.source, Source.factoryUnit)

  def test_off_adds_nothing(self):
    planner = self.make(setting=False)
    self.assertEqual(self.run_frames(planner, self.sm(32., 80., 5., unit=True), 10), [])
    self.assertFalse(planner.take_over_braking.active)

  def test_other_cars_never_run_it(self):
    planner = self.make(fingerprint="RAM_1500_5TH_GEN", setting=True)
    self.assertIsNone(planner.take_over_braking)
    self.assertEqual(self.run_frames(planner, self.sm(32., 80., 5.), 10), [])

  def test_published_fields(self):
    class PM:
      def send(self, name, msg):
        self.msg = msg
    planner = self.make(setting=True)
    sm = self.sm(32., 80., 5., unit=True)
    self.run_frames(planner, sm, 10)
    pm = PM()
    planner.resolver.distance = 0.  # the speed-limit resolver sets this on its first update, which this test skips
    planner.publish_longitudinal_plan_sp(FakeSubMaster(sm), pm)
    out = pm.msg.longitudinalPlanSP.takeOverBraking
    self.assertTrue(out.enabled and out.active and out.overLimit)
    self.assertEqual(out.source, custom.LongitudinalPlanSP.TakeOverBraking.Source.factoryUnit)
    self.assertAlmostEqual(out.gap, 80., places=3)
    self.assertAlmostEqual(out.closingSpeed, 27., places=3)
    self.assertGreater(out.brakingNeeded, 0.3 * G)


class FakeSubMaster(dict):
  """Enough of a SubMaster for publish_longitudinal_plan_sp"""
  def all_checks(self, service_list=None):
    return True


class TestTheAlert(unittest.TestCase):
  def test_shown_whether_or_not_engaged_loud_and_changes_nothing(self):
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP, ET, Priority, AudibleAlert, VisualAlert, \
      AlertStatus, AlertSize
    alerts = EVENTS_SP[EventNameSP.takeOverBraking]
    self.assertEqual(list(alerts), [ET.PERMANENT])  # an alert only: no disengagement, no blocked engagement
    a = alerts[ET.PERMANENT]
    self.assertEqual(a.alert_text_1, "BRAKE NOW")
    self.assertEqual((a.alert_status, a.alert_size, a.priority), (AlertStatus.critical, AlertSize.full, Priority.HIGHEST))
    self.assertEqual(a.audible_alert, AudibleAlert.warningImmediate)  # the loudest repeating sound
    self.assertEqual(a.visual_alert, VisualAlert.none)  # nothing new on the truck's dash


if __name__ == "__main__":
  unittest.main()
