"""
Tests for the Take-Over-Braking Alert toggle in Settings > Cruise (edfrahm/SP-RamHD-TICI-Port#28):
- it sits right below Custom Ram ACC Tuning and is shown only on a Ram Heavy Duty truck;
- it does not depend on Intelligent Cruise Button Management (it is a warning only);
- its text says what it does, that it never brakes, steers or disengages, that braking does not silence it, and when
  it works;
- its setting exists, is backed up, defaults to on, and the alert's code reads the same setting.

The screens need a graphics window, which a test machine does not have, so these tests read the screen code itself
with Python's own parser, as test_ram_acc_tuning.py does.
"""
import ast
import re
import unittest

from openpilot.selfdrive.ui.sunnypilot.tests.test_ram_acc_tuning import CRUISE, PARAMS_KEYS, items_list, module_strings, \
  toggle_calls
from openpilot.sunnypilot.selfdrive.controls.lib.take_over_braking import PARAM, PLATFORM
from openpilot.sunnypilot.selfdrive.ramweightd.settings import RAM_HD_PLATFORM


class TestTakeOverBrakingToggle(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    with open(CRUISE) as f:
      cls.source = f.read()
    cls.tree = ast.parse(cls.source)
    cls.call = toggle_calls(cls.tree)["ram_take_over_braking_toggle"]
    cls.kw = {k.arg: k.value for k in cls.call.keywords}

  def test_toggle_in_cruise_panel(self):
    self.assertEqual(self.kw["param"].value, PARAM)
    self.assertEqual(self.kw["title"].args[0].value, "Take-Over-Braking Alert")
    items = items_list(self.tree)
    self.assertEqual(items.index("ram_take_over_braking_toggle"), items.index("ram_acc_tuning_toggle") + 1)

  def test_ram_hd_only_and_not_tied_to_icbm(self):
    self.assertIn("self.ram_take_over_braking_toggle.set_visible(self._is_ram_hd())", self.source)
    self.assertNotIn("enabled", self.kw)
    self.assertEqual(RAM_HD_PLATFORM, PLATFORM)

  def test_description_says_what_it_does(self):
    desc = module_strings(self.tree)["RAM_TAKE_OVER_BRAKING_DESC"]
    for words in ("loud repeating alarm", "BRAKE NOW", "about 0.3 g", "never brakes, steers or disengages anything",
                  "your braking does not silence it", "40 mph", "within 60 m"):
      self.assertIn(words, desc)

  def test_setting_defaults_on(self):
    with open(PARAMS_KEYS) as f:
      keys = f.read()
    m = re.search(r'\{"RamTakeOverBrakingAlert", \{([^}]*)\}\}', keys)
    self.assertIsNotNone(m)
    self.assertEqual([a.strip() for a in m.group(1).split(",")], ["PERSISTENT | BACKUP", "BOOL", '"1"'])


if __name__ == "__main__":
  unittest.main()
