"""
Tests for the Custom Ram ACC Tuning toggle in Settings > Cruise (edfrahm/SP-RamHD-TICI-Port#10, #11):
- it sits right below Intelligent Cruise Button Management, with the other speed-control options;
- it can be switched on only on a Ram Heavy Duty truck with Intelligent Cruise Button Management on, which it works
  through, and its description says what is built (the planned-speed target, #11) and what is not yet;
- its setting exists, is backed up, and defaults to off, and the code that uses it reads the same setting.

The screens need a graphics window, which a test machine does not have, so these tests read the screen code
itself with Python's own parser.
"""
import ast
import os
import re
import unittest

from openpilot.common.basedir import BASEDIR
from openpilot.sunnypilot.selfdrive.controls.lib.planned_speed import RAM_ACC_TUNING_PARAM, RAM_ACC_TUNING_PLATFORM
from openpilot.sunnypilot.selfdrive.ramweightd.settings import RAM_HD_PLATFORM

CRUISE = os.path.join(BASEDIR, "openpilot", "selfdrive", "ui", "sunnypilot", "layouts", "settings", "cruise.py")
PARAMS_KEYS = os.path.join(BASEDIR, "openpilot", "common", "params_keys.h")


def toggle_calls(tree: ast.Module) -> dict[str, ast.Call]:
  out = {}
  for node in ast.walk(tree):
    if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call) and getattr(node.value.func, "id", None) == "toggle_item_sp":
      target = node.targets[0]
      if isinstance(target, ast.Attribute):
        out[target.attr] = node.value
  return out


def items_list(tree: ast.Module) -> list[str]:
  for node in ast.walk(tree):
    if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "items" for t in node.targets) \
       and isinstance(node.value, ast.List):
      return [e.attr for e in node.value.elts if isinstance(e, ast.Attribute)]
  raise AssertionError("items list not found")


def joined(node: ast.expr) -> str:
  """ "a" + "b" + ... in source order"""
  if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
    return joined(node.left) + joined(node.right)
  assert isinstance(node, ast.Constant) and isinstance(node.value, str)
  return node.value


def module_strings(tree: ast.Module) -> dict[str, str]:
  """Module-level NAME = tr_noop("..." + "...") strings"""
  out = {}
  for node in tree.body:
    if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and isinstance(node.value, ast.Call) \
       and getattr(node.value.func, "id", None) == "tr_noop":
      out[node.targets[0].id] = joined(node.value.args[0])
  return out


class TestRamAccTuningToggle(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    with open(CRUISE) as f:
      cls.source = f.read()
    cls.tree = ast.parse(cls.source)
    cls.call = toggle_calls(cls.tree)["ram_acc_tuning_toggle"]
    cls.kw = {k.arg: k.value for k in cls.call.keywords}

  def test_toggle_in_cruise_panel(self):
    self.assertIsInstance(self.kw["param"], ast.Constant)
    self.assertEqual(self.kw["param"].value, "RamAccTuning")
    title = self.kw["title"]
    self.assertTrue(isinstance(title, ast.Call) and title.args[0].value == "Custom Ram ACC Tuning")
    items = items_list(self.tree)
    self.assertEqual(items.index("ram_acc_tuning_toggle"), items.index("icbm_toggle") + 1)

  def test_enabled_only_on_ram_hd_with_icbm(self):
    enabled = self.kw["enabled"]
    self.assertIsInstance(enabled, ast.Lambda, "decided while the screen runs, not fixed")
    names = {n.attr for n in ast.walk(enabled) if isinstance(n, ast.Attribute)}
    self.assertIn("_is_ram_hd", names)
    self.assertIn("has_icbm", names)
    self.assertIn("carFingerprint == RAM_HD_PLATFORM", self.source)
    self.assertEqual(RAM_HD_PLATFORM, RAM_ACC_TUNING_PLATFORM)

  def test_description_says_what_is_built(self):
    strings = module_strings(self.tree)
    self.assertNotIn("RAM_ACC_TUNING_NOT_BUILT", strings)
    self.assertNotIn("Until it is", self.source)
    self.assertIn("coasts first", strings["RAM_ACC_TUNING_DESC"])
    self.assertIn("still does all following and braking", strings["RAM_ACC_TUNING_DESC"])
    self.assertIn("pauses it for 3 seconds", strings["RAM_ACC_TUNING_DESC"])
    self.assertTrue(strings["RAM_ACC_TUNING_LATER"].startswith("Not built yet: paced speed recovery"))
    self.assertIn("Intelligent Cruise Button Management", strings["RAM_ACC_TUNING_NEEDS_ICBM"])

  def test_setting_defaults_off(self):
    with open(PARAMS_KEYS) as f:
      keys = f.read()
    m = re.search(r'\{"RamAccTuning", \{([^}]*)\}\}', keys)
    self.assertIsNotNone(m)
    attrs = [a.strip() for a in m.group(1).split(",")]
    self.assertEqual(attrs, ["PERSISTENT | BACKUP", "BOOL", '"0"'])
    self.assertEqual(RAM_ACC_TUNING_PARAM, "RamAccTuning")


if __name__ == "__main__":
  unittest.main()
