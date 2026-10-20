"""Intelligent Cruise Button Management on a car whose own cruise control reads the steering-wheel buttons (pcmCruise):
the driver's set speed is the car's set speed after the driver's presses, not a separate count of them.

Replays route 00000783--e3bae4dd6c at 14:47:01 (2026-10-19, Ram 2500): + held for 1.2 s; the car's set speed went
65 -> 66 -> 70 mph, sunnypilot's own count went 65 -> 70 -> 75, and ICBM pressed + until the car was at 75.
"""
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import VCruiseHelper

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
MPH = CV.MPH_TO_MS


class TestIcbmDriverSetSpeed(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.params.put_bool("CustomAccIncrementsEnabled", False, block=True)
    self.helper = VCruiseHelper(car.CarParams(pcmCruise=True), custom.CarParamsSP(pcmCruiseSpeed=False))

  def step(self, car_set_mph, events=()):
    CS = car.CarState(cruiseState={"available": True, "enabled": True, "speed": car_set_mph * MPH})
    CS.buttonEvents = list(events)
    self.helper.update_v_cruise(CS, enabled=True, is_metric=False)

  def v_cruise_mph(self):
    return self.helper.v_cruise_kph * CV.KPH_TO_MPH

  def engage_at(self, mph):
    for _ in range(100):
      self.step(mph)
    assert abs(self.v_cruise_mph() - mph) < 0.5

  def hold_accel_like_ram(self, start_mph):
    """+ held 1.2 s: the car steps +1 at 0.2 s and to the next multiple of 5 at 0.9 s, then settles"""
    self.step(start_mph, [ButtonEvent(type=ButtonType.accelCruise, pressed=True)])
    for frame in range(1, 120):
      self.step(start_mph if frame < 20 else start_mph + 1 if frame < 90 else start_mph + 5)
    self.step(start_mph + 5, [ButtonEvent(type=ButtonType.accelCruise, pressed=False)])
    for _ in range(200):
      self.step(start_mph + 5)

  def test_long_press_follows_the_car(self):
    self.engage_at(65)
    self.hold_accel_like_ram(65)
    assert abs(self.v_cruise_mph() - 70) < 0.5, self.v_cruise_mph()

  def test_car_set_speed_changes_without_driver_presses_are_not_adopted(self):
    """ICBM's own presses (lowering for a curve) change the car's set speed; the driver's choice must stay"""
    self.engage_at(70)
    for _ in range(300):
      self.step(55)
    assert abs(self.v_cruise_mph() - 70) < 0.5

  def test_custom_increments_keep_sunnypilot_count(self):
    self.params.put_bool("CustomAccIncrementsEnabled", True, block=True)
    self.params.put("CustomAccShortPressIncrement", 1, block=True)
    self.params.put("CustomAccLongPressIncrement", 5, block=True)
    self.helper.read_custom_set_speed_params()
    self.engage_at(65)
    self.hold_accel_like_ram(65)
    assert self.v_cruise_mph() > 72, "with custom increments sunnypilot's own count stands"
