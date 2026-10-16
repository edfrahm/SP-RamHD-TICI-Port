import os
import tempfile
import unittest
from unittest import mock

from panda import McuType
from openpilot.selfdrive.pandad import pandad


class TestPandadMcu(unittest.TestCase):
  def test_expected_signature_follows_mcu(self):
    # the comma 3's F4 panda must be checked against panda.bin.signed, H7 pandas against panda_h7.bin.signed
    with tempfile.TemporaryDirectory() as fw_path:
      sigs = {}
      for i, mcu in enumerate(McuType):
        sigs[mcu] = bytes([i + 1]) * 128
        with open(os.path.join(fw_path, mcu.config.app_fn), "wb") as f:
          f.write(b"\x00" * 1024 + sigs[mcu])

      with mock.patch.object(pandad, "FW_PATH", fw_path):
        for mcu in McuType:
          panda = mock.Mock(get_mcu_type=mock.Mock(return_value=mcu))
          self.assertEqual(pandad.get_expected_signature(panda), sigs[mcu])


if __name__ == "__main__":
  unittest.main()
