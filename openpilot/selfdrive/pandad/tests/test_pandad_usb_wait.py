import unittest
from unittest import mock

from openpilot.selfdrive.pandad import pandad


class FakeClock:
  def __init__(self, t):
    self.t = t

  def monotonic(self):
    return self.t

  def sleep(self, dt):
    self.t += dt


class TestWaitForUsbPanda(unittest.TestCase):
  # the comma 3's internal panda is on USB and needs time to (re-)enumerate after a reset

  def _wait(self, start, appears_at=None):
    clock = FakeClock(start)

    def usb(*args, **kwargs):
      return ["serial"] if appears_at is not None and clock.t >= appears_at else []

    with mock.patch.object(pandad.time, "monotonic", clock.monotonic), mock.patch.object(pandad.time, "sleep", clock.sleep), \
         mock.patch.object(pandad.Panda, "list", side_effect=usb), mock.patch.object(pandad.PandaDFU, "usb_list", return_value=[]):
      pandad.wait_for_usb_panda()
    return clock.t

  def test_returns_when_panda_enumerates(self):
    self.assertAlmostEqual(self._wait(start=100., appears_at=102.5), 102.5, delta=0.2)

  def test_gives_up_after_5s(self):
    self.assertAlmostEqual(self._wait(start=100.), 106., delta=0.2)

  def test_waits_for_usb_after_boot(self):
    # USB takes up to a minute to come up after boot
    self.assertAlmostEqual(self._wait(start=10.), 60., delta=0.2)
    self.assertAlmostEqual(self._wait(start=10., appears_at=40.), 40., delta=0.2)


if __name__ == "__main__":
  unittest.main()
