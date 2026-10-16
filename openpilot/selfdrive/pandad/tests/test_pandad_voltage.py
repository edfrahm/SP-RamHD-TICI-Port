import re
import unittest
from pathlib import Path

OPENPILOT = Path(__file__).resolve().parents[3]


class TestPandadVoltage(unittest.TestCase):
  # the comma 3's panda (dos) doesn't measure the car's voltage reliably (it reads 0.2-29 V on a 12 V car),
  # so on tici peripheralState.voltage/current must come from the SoM's INA231, which hardwared's power
  # monitoring uses for the low-voltage shutdown

  def test_same_sensor_as_python(self):
    cc = (OPENPILOT / "common/hardware/comma/hardware.h").read_text()
    py = (OPENPILOT / "common/hardware/comma/hardware.py").read_text()
    for name, node in (("get_voltage", "in1_input"), ("get_current", "curr1_input")):
      with self.subTest(name=name):
        path = f"/sys/class/hwmon/hwmon1/{node}"
        self.assertRegex(cc, rf"static int {name}\(\) \{{[^}}]*\"{re.escape(path)}\"")
        self.assertRegex(py, rf"def {name}\(self\):\n\s+with open\(\"{re.escape(path)}\"\)")

  def test_tici_uses_hwmon(self):
    src = (OPENPILOT / "selfdrive/pandad/pandad.cc").read_text()
    self.assertRegex(src, r"bool use_hwmon\(\) \{\n\s+return Hardware::get_device_type\(\) == cereal::InitData::DeviceType::TICI;")
    self.assertRegex(src, r"if \(use_hwmon\(\)\) \{\n\s+hardware_thread = std::thread\(hwmon_thread\);")
    fn = re.search(r"void send_peripheral_state\(.*?\n\}\n", src, re.S).group(0)
    self.assertIn("if (hwmon && !hwmon_state.initialized.load()) {\n    return;", fn)
    self.assertRegex(fn, r"if \(hwmon\) \{\n\s+ps\.setVoltage\(hwmon_state\.voltage\.load\(\)\);\n\s+ps\.setCurrent\(hwmon_state\.current\.load\(\)\);")


if __name__ == "__main__":
  unittest.main()
