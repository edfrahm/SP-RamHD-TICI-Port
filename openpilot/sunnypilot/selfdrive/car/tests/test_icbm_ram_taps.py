"""Intelligent Cruise Button Management with Custom Ram ACC Tuning (edfrahm/SP-RamHD-TICI-Port#11): taps instead of a
held button, a pause after the driver's pedals and buttons, and a wait when taps go unanswered.

The truck below answers buttons as the Phase 1 study measured them on the Ram 2500 (reports/cruise_buttons.md):
a press moves the set speed 1 mph after 60 ms; held on, it jumps to the next multiple of 5 0.72 s after that first
step and every 0.72 s after; a release changes nothing. The set speed reaches sunnypilot about 0.1 s after it changes.
On 2026-10-19 at 14:47:02 the truck took Intelligent Cruise Button Management's own frames exactly like this
(70 -> 71 -> 75 in 0.82 s).
"""
import unittest

from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management import controller as icbm_controller
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement

MPH = 0.44704
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
ButtonType = car.CarState.ButtonEvent.Type


class RamCruise:
  """The truck's cruise control as measured: tap = 1 mph; hold = 1 mph, then multiples of 5 every 0.72 s"""
  FIRST_S, JUMP_S, REPORT_S = 0.06, 0.72, 0.1

  def __init__(self, set_mph, accepts=True):
    self.set_mph = set_mph
    self.accepts = accepts
    self.t = 0.
    self.pressed_since = None
    self.button = SendButtonState.none
    self.steps = 0
    self.history = [(-1., set_mph)]  # (time, set speed) as it changes

  def step(self, button):
    self.t += DT_CTRL
    if button != self.button:
      self.pressed_since = self.t if button != SendButtonState.none else None
      self.button, self.steps = button, 0
    if self.pressed_since is None or not self.accepts:
      return
    held = self.t - self.pressed_since
    due = 0 if held < self.FIRST_S else 1 + int((held - self.FIRST_S) // self.JUMP_S)
    while self.steps < due:
      up = button == SendButtonState.increase
      if self.steps == 0:
        self.set_mph += 1 if up else -1
      else:
        self.set_mph = (self.set_mph // 5 + 1) * 5 if up else (self.set_mph - 1) // 5 * 5
      self.steps += 1
      self.history.append((self.t, self.set_mph))

  def reported_mph(self):
    """What sunnypilot sees: the set speed as it was REPORT_S ago"""
    return [v for t, v in self.history if t <= self.t - self.REPORT_S][-1]


class FakeParams:
  def __init__(self, on):
    self.on = on

  def get_bool(self, key):
    return key == "RamAccTuning" and self.on


class TestIcbmRamTaps(unittest.TestCase):
  def make(self, toggle=True, fingerprint="RAM_HD_5TH_GEN"):
    icbm = IntelligentCruiseButtonManagement(structs.CarParams(carFingerprint=fingerprint, pcmCruise=True),
                                             structs.CarParamsSP(pcmCruiseSpeed=False))
    icbm.cruise_button_timers = dict.fromkeys(icbm.cruise_button_timers, 0)  # its own, not the shared module dict
    icbm.read_params(FakeParams(toggle))
    return icbm

  def drive(self, icbm, truck, target_mph, seconds, gas=False, events_at=None):
    presses, pressed_prev, button_log = [], False, []
    for i in range(int(seconds / DT_CTRL)):
      CS = car.CarState(gasPressed=gas, cruiseState={"enabled": True, "speedCluster": truck.reported_mph() * MPH})
      if events_at is not None and i in events_at:
        CS.buttonEvents = [car.CarState.ButtonEvent(type=ButtonType.accelCruise, pressed=events_at[i])]
      CC = car.CarControl(enabled=True)
      CC.cruiseControl.override = gas
      LP_SP = custom.LongitudinalPlanSP(vTarget=target_mph * MPH)
      icbm.run(CS, CC, LP_SP, is_metric=False)
      truck.step(icbm.cruise_button)
      pressed = icbm.cruise_button != SendButtonState.none
      if pressed and not pressed_prev:
        presses.append([i * DT_CTRL, 0.])
      if pressed:
        presses[-1][1] += DT_CTRL
      pressed_prev = pressed
      button_log.append(icbm.cruise_button)
    return presses, button_log

  def test_held_button_hunts_today(self):
    """Toggle off (today): the held button never settles on 63"""
    icbm, truck = self.make(toggle=False), RamCruise(75)
    self.drive(icbm, truck, 63, 8.)
    self.assertGreater(len({v for t, v in truck.history if t > 3.}), 2)
    self.assertNotEqual(truck.set_mph, 63)

  def test_taps_settle_without_hunting(self):
    icbm, truck = self.make(), RamCruise(75)
    presses, _ = self.drive(icbm, truck, 63, 8.)
    self.assertEqual(truck.set_mph, 63)
    self.assertEqual([v for _, v in truck.history], list(range(75, 62, -1)))  # one mph at a time, never past 63
    self.assertLess(max(t for t, _ in truck.history), 4.)  # 12 taps at 4 per second, after the 0.4 s wait
    self.assertTrue(all(length <= icbm_controller.TAP_PRESS + 1e-6 for _, length in presses))
    gaps = [b[0] - (a[0] + a[1]) for a, b in zip(presses, presses[1:], strict=False)]
    self.assertGreaterEqual(min(gaps), icbm_controller.TAP_RELEASE - 1e-6)

  def test_taps_up_to_the_target(self):
    icbm, truck = self.make(), RamCruise(60)
    self.drive(icbm, truck, 64, 4.)
    self.assertEqual([v for _, v in truck.history], [60, 61, 62, 63, 64])

  def test_other_cars_keep_the_held_button(self):
    icbm = self.make(fingerprint="RAM_1500_5TH_GEN")
    self.assertFalse(icbm.ram_acc_tuning)

  def test_unanswered_taps_wait(self):
    """15:59:08 on 2026-10-19: 77 frames of - went out and the set speed did not move. After 4 unanswered taps,
    tapping waits 2 s instead of pressing on."""
    icbm, truck = self.make(), RamCruise(70, accepts=False)
    presses, _ = self.drive(icbm, truck, 60, 4.)
    starts = [t for t, _ in presses]
    self.assertEqual(len([t for t in starts if t < 2.]), 4)
    self.assertGreaterEqual(starts[4] - starts[3], icbm_controller.TAP_RETRY)

  def test_gas_pauses_for_3_s(self):
    icbm, truck = self.make(), RamCruise(70)
    presses, _ = self.drive(icbm, truck, 65, 1., gas=True)
    self.assertEqual(presses, [])
    presses, _ = self.drive(icbm, truck, 65, 5.)
    self.assertGreaterEqual(presses[0][0], icbm_controller.DRIVER_PAUSE)

  def test_driver_button_pauses_for_3_s(self):
    """The driver taps + at 0.5 s (released at 0.6 s); a lower target 1 s later waits until 3 s after the release"""
    icbm, truck = self.make(), RamCruise(70)
    presses, _ = self.drive(icbm, truck, 70, 1., events_at={50: True, 60: False})
    self.assertEqual(presses, [])
    presses, _ = self.drive(icbm, truck, 65, 5.)
    self.assertGreaterEqual(1. + presses[0][0], 0.6 + icbm_controller.DRIVER_PAUSE)

  def test_toggle_off_takes_effect(self):
    icbm, truck = self.make(), RamCruise(75)
    icbm.read_params(FakeParams(False))
    self.assertFalse(icbm.ram_acc_tuning)
    _, log = self.drive(icbm, truck, 70, 1.5)
    self.assertGreater(sum(b == SendButtonState.decrease for b in log), 50)  # held, as today


if __name__ == "__main__":
  unittest.main()
