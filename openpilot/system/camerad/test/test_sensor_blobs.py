import re
import unittest
from pathlib import Path

from openpilot.cereal import log

CAMERAD = Path(__file__).resolve().parents[1]
ImageSensor = log.FrameData.ImageSensor


def bps_blobs() -> dict[str, tuple[int, list[bytes | None]]]:
  src = (CAMERAD / "cameras/bps_blobs.h").read_text()
  blobs = {}
  for m in re.finditer(r"unsigned char (\w+)\[4\]\[(\w+)\] = \{(.*?)\n\};", src, re.S):
    entries = re.findall(r"\{([^{}]*)\}", m.group(3))
    blobs[m.group(1)] = (int(m.group(2), 0), [None if "placeholder" in e else bytes(int(x, 16) for x in re.findall(r"0x[0-9a-fA-F]+", e))
                                              for e in entries])
  return blobs


def probed_sensors() -> list[str]:
  src = (CAMERAD / "cameras/spectra.cc").read_text()
  return re.findall(r"init_sensor_lambda\(new (\w+)\)", src)


class TestSensorBlobs(unittest.TestCase):
  def test_ar0231_probed(self):
    # comma 3
    self.assertIn("AR0231", probed_sensors())

  def test_probed_sensors_have_bps_blobs(self):
    # the cabin camera is BPS-processed, so every sensor camerad can bring up needs its BPS blobs
    blobs = bps_blobs()
    self.assertEqual(set(blobs), {"bps_cfg", "bps_striping_output", "bps_settings"})
    for sensor in probed_sensors():
      idx = ImageSensor.schema.enumerants[sensor.lower()]
      for name, (size, entries) in blobs.items():
        with self.subTest(sensor=sensor, blob=name):
          self.assertIsNotNone(entries[idx], f"{name} has no blob for {sensor}")
          self.assertLessEqual(len(entries[idx]), size)

  def test_ar0231_full_res(self):
    # the AR0231 BPS blobs are for full resolution output (no BPS downscale)
    src = (CAMERAD / "sensors/ar0231.cc").read_text()
    self.assertNotIn("out_scale", src)
    self.assertIn("frame_width = 1928;", src)
    self.assertIn("frame_height = 1208;", src)


if __name__ == "__main__":
  unittest.main()
