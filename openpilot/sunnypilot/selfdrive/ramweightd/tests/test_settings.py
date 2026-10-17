"""Settings helpers. The settings screen itself (vehicle panel, brands/chrysler.py) needs the user interface's
messaging and a display, so it is only checked here by reading its source; it runs on the comma 3."""
import ast
import os

from openpilot.common.params import Params, ParamKeyType
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.ramweightd import settings as S

PANEL = os.path.join(os.path.dirname(__file__), "../../../../selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/chrysler.py")


class TestSettings(OpenpilotTestCase):
  def test_param_keys(self):
    params = Params()
    for key, kind in (("CruiseBaseWeightLb", ParamKeyType.FLOAT), ("CruiseKnownExtraLb", ParamKeyType.FLOAT),
                      ("RamWeightGradeState", ParamKeyType.JSON)):
      assert params.get_type(key) == kind
    # the option control writes floats, which a FLOAT setting accepts
    for value in S.KNOWN_EXTRA_STEPS.values():
      params.put("CruiseKnownExtraLb", value, block=True)
    assert params.get("CruiseKnownExtraLb") == 2000.
    params.put("RamWeightGradeState", {"mass_kg": 3810., "mass_std_kg": 50.}, block=True)
    assert abs(S.estimate_lb(params.get("RamWeightGradeState"))[0] - 8399.6) < 0.1

  def test_known_extra_steps(self):
    assert list(S.KNOWN_EXTRA_STEPS.values()) == [float(x) for x in range(0, 2001, 10)]
    assert list(S.KNOWN_EXTRA_STEPS) == list(range(201))

  def test_parse_weight(self):
    assert S.parse_weight_lb("8400") == 8400.
    assert S.parse_weight_lb(" 8,400 lb ") == 8400.
    assert S.parse_weight_lb("0") == 0.
    for bad in ("", "abc", "-5", "40001", "8.4.0", None):
      assert S.parse_weight_lb(bad) is None, bad

  def test_estimate(self):
    lb, std = S.estimate_lb({"mass_kg": 3810., "mass_std_kg": 45.})
    assert abs(lb - 8399.6) < 0.1 and abs(std - 99.2) < 0.1
    for bad in (None, {}, {"mass_kg": "x", "mass_std_kg": 1.}, []):
      assert S.estimate_lb(bad) is None

  def test_panel_source(self):
    with open(PANEL) as f:
      source = f.read()
    ast.parse(source)
    for needed in ('"CruiseKnownExtraLb"', '"CruiseBaseWeightLb"', '"RamWeightGradeState"', "RAM_HD_PLATFORM", "set_visible"):
      assert needed in source, needed
