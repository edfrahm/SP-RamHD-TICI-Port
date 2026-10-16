import os
import shutil
import stat
import subprocess
import tempfile
import unittest

UPDATER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../updater")


def stub(path: str, body: str) -> None:
  with open(path, "w") as f:
    f.write(f"#!/usr/bin/env bash\n{body}\n")
  os.chmod(path, os.stat(path).st_mode | stat.S_IEXEC)


class TestUpdaterWrapper(unittest.TestCase):
  # comma 3s coming from AGNOS < 14 run weston and need the Qt updater; if no updater UI can run, flash without it

  def setUp(self):
    self.tmp = tempfile.mkdtemp()
    self.addCleanup(shutil.rmtree, self.tmp)
    self.log = os.path.join(self.tmp, "log")
    self.bin = os.path.join(self.tmp, "bin")
    os.mkdir(self.bin)
    shutil.copy(UPDATER, os.path.join(self.tmp, "updater"))
    for ui in ("updater_weston", "updater_magic"):
      stub(os.path.join(self.tmp, ui), f'echo "{ui} $*" >> {self.log}; exit 1')
    self.agnos_py = os.path.join(self.tmp, "agnos.py")
    stub(self.agnos_py, f'echo "agnos.py $*" >> {self.log}')
    self.manifest = os.path.join(self.tmp, "agnos.json")
    open(self.manifest, "w").close()
    stub(os.path.join(self.bin, "sudo"), f'echo "sudo $*" >> {self.log}')
    self.failures = os.path.join(self.tmp, "failures")

  def run_updater(self, weston: bool) -> list[str]:
    stub(os.path.join(self.bin, "systemctl"), f"exit {0 if weston else 3}")
    env = dict(os.environ, PATH=f"{self.bin}:{os.environ['PATH']}", AGNOS_UPDATER_FAILURES=self.failures)
    subprocess.run([os.path.join(self.tmp, "updater"), self.agnos_py, self.manifest], env=env, timeout=30, check=False)
    with open(self.log) as f:
      lines = f.read().splitlines()
    os.remove(self.log)
    return lines

  def test_weston_uses_qt_updater(self):
    self.assertEqual(self.run_updater(weston=True), [f"updater_weston {self.agnos_py} {self.manifest}"])

  def test_no_weston_uses_raylib_updater(self):
    self.assertEqual(self.run_updater(weston=False), [f"updater_magic {self.agnos_py} {self.manifest}"])

  def test_flashes_without_ui_after_three_failures(self):
    for _ in range(3):
      self.assertEqual(len(self.run_updater(weston=True)), 1)
    self.assertEqual(self.run_updater(weston=True), [f"agnos.py --swap {self.manifest}", "sudo reboot"])


if __name__ == "__main__":
  unittest.main()
