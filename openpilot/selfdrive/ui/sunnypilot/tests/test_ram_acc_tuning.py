"""
Tests for the Custom Ram ACC Tuning toggle in Settings > Cruise (edfrahm/SP-RamHD-TICI-Port#10):
- it sits right below Intelligent Cruise Button Management, with the other speed-control options;
- it is created disabled, so it cannot be turned on before the Stage 2 smoothing it controls is built;
- its setting exists, is backed up, and defaults to off.

The screens need a graphics window, which a test machine does not have, so these tests read the screen code
itself with Python's own parser.
"""
import ast
import os
import re

from openpilot.common.basedir import BASEDIR

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


def test_toggle_in_cruise_panel():
  with open(CRUISE) as f:
    tree = ast.parse(f.read())
  call = toggle_calls(tree)["ram_acc_tuning_toggle"]
  kw = {k.arg: k.value for k in call.keywords}
  assert isinstance(kw["param"], ast.Constant) and kw["param"].value == "RamAccTuning"
  assert isinstance(kw["enabled"], ast.Constant) and kw["enabled"].value is False, "cannot be turned on until Stage 2 is built"
  title = kw["title"]
  assert isinstance(title, ast.Call) and title.args[0].value == "Custom Ram ACC Tuning"

  items = items_list(tree)
  assert items.index("ram_acc_tuning_toggle") == items.index("icbm_toggle") + 1


def test_setting_defaults_off():
  with open(PARAMS_KEYS) as f:
    keys = f.read()
  m = re.search(r'\{"RamAccTuning", \{([^}]*)\}\}', keys)
  assert m is not None
  attrs = [a.strip() for a in m.group(1).split(",")]
  assert attrs == ["PERSISTENT | BACKUP", "BOOL", '"0"']
