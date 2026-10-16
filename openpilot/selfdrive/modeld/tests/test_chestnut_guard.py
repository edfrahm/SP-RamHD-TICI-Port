import tempfile
import unittest
from pathlib import Path
from unittest import mock

import openpilot.cereal.messaging as messaging
from openpilot.common.hardware import HARDWARE
from openpilot.common.hardware.usb import CHESTNUT_USB_IDS, CHESTNUT_USB_PRODUCT, set_usb_state
from openpilot.selfdrive.modeld import helpers


def fake_chestnut(devices_path: Path) -> None:
  d = devices_path / "2-1"
  d.mkdir(parents=True)
  vid, pid = CHESTNUT_USB_IDS[0]
  (d / "idVendor").write_text(f"{vid:04x}\n")
  (d / "idProduct").write_text(f"{pid:04x}\n")
  (d / "product").write_text(f"{CHESTNUT_USB_PRODUCT}\n")


class TestChestnutGuard(unittest.TestCase):
  # the comma 3 (tici) must never report chestnut: no big model list, no big model compile

  def test_chestnut_present(self):
    with tempfile.TemporaryDirectory() as tmp:
      fake_chestnut(Path(tmp))
      with mock.patch.object(helpers, "USB_DEVICES_PATH", Path(tmp)):
        for device_type, expected in (("tici", False), ("tizi", True), ("mici", True)):
          with self.subTest(device_type=device_type), mock.patch.object(HARDWARE, "get_device_type", return_value=device_type):
            self.assertEqual(helpers.chestnut_present(), expected)

  def test_device_state(self):
    vid, pid = CHESTNUT_USB_IDS[0]
    chestnut = {"busnum": 2, "devnum": 2, "vendorId": vid, "productId": pid, "speedMbps": 5000,
                "manufacturer": "comma.ai", "product": CHESTNUT_USB_PRODUCT, "linkErrorCount": 0}
    for device_type, expected in (("tici", False), ("tizi", True)):
      with self.subTest(device_type=device_type), mock.patch.object(HARDWARE, "get_device_type", return_value=device_type):
        msg = messaging.new_message('deviceState')
        set_usb_state(msg.deviceState, [chestnut])
        self.assertEqual(msg.deviceState.chestnutPresent, expected)


if __name__ == "__main__":
  unittest.main()
