"""The Ram Heavy Duty weight rows in Settings > Vehicle (brands/chrysler.py), built and updated without a display.

On 2026-10-19 the comma 3's screen process crashed three times (routes 00000781 and 00000784, and parked) with
AttributeError: 'ChryslerSettings' object has no attribute '_estimate': the Weight Estimate row reads its value as
soon as it is created, before the page had set up its stored estimate. The manager does not restart a crashed
screen process, so the screen stayed on the boot logo while everything else kept running.

The widget builders are replaced by stand-ins that read their values the way the real widgets do (TextAction reads
its text when it is created), so the page's own code runs as it does on the device.
"""
from unittest import mock

import openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.chrysler as chrysler
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.ramweightd.settings import RAM_HD_PLATFORM


def resolve(value):
  return value() if callable(value) else value


class FakeAction:
  def __init__(self, value=None):
    self.value = resolve(value)

  def set_value(self, value):
    self.value = resolve(value)


class FakeItem:
  def __init__(self, title, value=None):
    self.title = resolve(title)
    self.action_item = FakeAction(value)
    self.visible = True

  def set_visible(self, visible):
    self.visible = visible


class FakeParams:
  def __init__(self, values):
    self.values = values

  def get(self, key, return_default=False):
    return self.values.get(key)


class FakeUIState:
  def __init__(self, values):
    self.params = FakeParams(values)
    self.CP = None


def build(values):
  ui = FakeUIState(values)
  with mock.patch.multiple(chrysler, ui_state=ui,
                           text_item=lambda title, value, description=None, callback=None, enabled=True: FakeItem(title, value),
                           button_item_sp=lambda title, button_text, description=None, callback=None, enabled=True: FakeItem(title),
                           option_item_sp=lambda title, param, min_value, max_value, description=None, **kw: FakeItem(title)):
    page = chrysler.ChryslerSettings()
    page.update_settings()
  return page


class TestSettingsScreen(OpenpilotTestCase):
  def test_builds_with_no_estimate(self):
    page = build({"CarPlatformBundle": {"platform": RAM_HD_PLATFORM}})
    assert page._estimate_item.action_item.value == "No estimate yet"

  def test_builds_with_eric_settings(self):
    page = build({"CarPlatformBundle": {"platform": RAM_HD_PLATFORM}, "CruiseBaseWeightLb": 8400., "CruiseEmptyWeightLb": 8040.,
                  "RamWeightGradeState": {"mass_kg": 3810., "mass_std_kg": 50.}})
    assert page._estimate_item.action_item.value.startswith("8,400 lb")
    assert page._base_weight_item.action_item.value == "8,400 lb"
    assert page._empty_weight_item.action_item.value == "8,040 lb"
    assert all(item.visible for item in page.items)

  def test_malformed_estimate_and_other_trucks(self):
    page = build({"CarPlatformBundle": {"platform": "RAM_1500_5TH_GEN"}, "RamWeightGradeState": {"mass_kg": "x"}})
    assert page._estimate_item.action_item.value == "No estimate yet"
    assert not any(item.visible for item in page.items)
