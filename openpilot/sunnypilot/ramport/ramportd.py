#!/usr/bin/env python3
"""
ramportd: the Ram port shared data upload process (edfrahm/SP-RamHD-TICI-Port#9). Runs only on Ram Heavy Duty trucks.

Two parts:
1. Writer, while driving: 10 times per second it takes the newest values from carState, modelV2, ramWeightGrade and
   the controller area network (CAN) messages DAS_3, DAS_4 (bus 2) and ECM_5 (bus 0), and hands one row to the drive
   writer, which trims the first and last 2 minutes and writes one gzip file per drive, split every hour, into the
   queue folder. A drive is recorded only if the toggle "SunnyPilot/Ram Port data Upload" (setting
   EnableSunnylinkUploader) is on when the drive starts. Switching it off during the drive throws away the rest.
2. Uploader, in a background thread: sends the queued files when the toggle is on and the network is unmetered.
   While driving it sends only if the existing setting OnroadUploads allows uploads while driving.

Command-line use (for tests and the local end-to-end test):
  python -m openpilot.sunnypilot.ramport.ramportd --make-test-file <folder> [--minutes 10]
  python -m openpilot.sunnypilot.ramport.ramportd --upload-file <path> --url <service address>
      [--key <private key file>] [--serial <serial>] [--imei <modem ID>]
"""
import argparse
import json
import math
import os
import re
import sys
import threading
import time

from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.ramport.drive_format import COLUMN_NAMES, format_value, wall_time
from openpilot.sunnypilot.ramport.drive_writer import DriveWriter, enforce_queue_limit, recover_leftovers
from openpilot.sunnypilot.ramport.sampler import CanSource, sample_fields
from openpilot.sunnypilot.ramport.uploader import QueueUploader, Reaction, api_host, key_from_private_pem, load_device_key, upload_file_once

SAMPLE_HZ = 10
TOGGLE_CHECK_S = 1.0
# A message older than this is treated as not available
MAX_AGE_NS = {"carState": int(0.5e9), "modelV2": int(0.5e9), "ramWeightGrade": int(1.0e9)}
PART_NUMBER_RE = re.compile(r"[^A-Za-z0-9]")


def _part_number(fw_version: bytes) -> str:
  return PART_NUMBER_RE.sub("", fw_version.decode("ascii", errors="ignore"))[:32]


def static_metadata_from_car_params(CP) -> dict:
  """Truck platform name and the engine and transmission module part numbers the truck reported at startup."""
  parts: dict[str, str] = {"engine": "", "transmission": ""}
  for fw in sorted(CP.carFw, key=lambda f: (f.logging, f.address)):
    ecu = str(fw.ecu)
    if ecu in parts and not parts[ecu]:
      parts[ecu] = _part_number(bytes(fw.fwVersion))
  return {"truck": str(CP.carFingerprint), "engine_part": parts["engine"], "transmission_part": parts["transmission"]}


def software_version() -> str:
  """sunnypilot version and short git commit, for example "2026.003.000 12038f7b0"."""
  try:
    from openpilot.common.version import get_build_metadata
    md = get_build_metadata()
    return f"{md.openpilot.short_version} {md.openpilot.git_commit[:9]}".strip()
  except Exception:
    cloudlog.exception("ramport: could not read the software version")
    return ""


def read_static_metadata(params) -> dict:
  meta = {"truck": "", "engine_part": "", "transmission_part": ""}
  try:
    from openpilot.cereal import messaging
    from opendbc.car.structs import car
    cp_bytes = params.get("CarParams") or params.get("CarParamsPersistent")
    if cp_bytes:
      meta.update(static_metadata_from_car_params(messaging.log_from_bytes(cp_bytes, car.CarParams)))
  except Exception:
    cloudlog.exception("ramport: could not read the car parameters")
  meta["software"] = software_version()
  return meta


class RamPortDaemon:
  """The writer half: notices drive start and end, and feeds samples to the drive writer."""

  def __init__(self, params, root: str | None = None, meta_provider=None):
    self.params = params
    self.root = root or Paths.ramport_root()
    self._meta: dict | None = None
    self._meta_provider = meta_provider or (lambda: read_static_metadata(self.params))
    self.can = CanSource()
    self.writer: DriveWriter | None = None
    self.in_drive = False
    self.next_toggle_check = 0.0
    os.makedirs(self.root, exist_ok=True)

  def static_meta(self) -> dict:
    if self._meta is None or not self._meta.get("truck"):
      self._meta = self._meta_provider()
    return self._meta

  def recover(self) -> list[str]:
    return recover_leftovers(self.root, self.static_meta())

  def update_can(self, can_strings) -> None:
    if self.writer is not None:
      self.can.update_strings(can_strings)

  def step(self, started: bool, mono: float, wall: float, car_state=None, model=None, rwg=None, now_nanos: int | None = None) -> None:
    """Called about 10 times per second. Message arguments are None when that message is not available."""
    if started and not self.in_drive:
      self.in_drive = True
      self.can = CanSource()
      if self.params.get_bool("EnableSunnylinkUploader"):
        self.writer = DriveWriter(self.root, self.static_meta)
        self.next_toggle_check = mono + TOGGLE_CHECK_S
        cloudlog.info("ramport: drive started, recording")
      else:
        self.writer = None
        cloudlog.info("ramport: drive started, upload toggle off, not recording")
    elif not started and self.in_drive:
      self.in_drive = False
      self.end_drive()

    if self.writer is None:
      return

    if mono >= self.next_toggle_check:
      self.next_toggle_check = mono + TOGGLE_CHECK_S
      if not self.params.get_bool("EnableSunnylinkUploader"):
        cloudlog.info("ramport: upload toggle switched off during the drive, discarding the rest of it")
        self.writer.abandon()
        self.writer = None
        return

    try:
      self.writer.add_sample(mono, wall, sample_fields(car_state, model, rwg, self.can, now_nanos))
    except OSError:
      cloudlog.exception("ramport: could not write, stopping this drive")
      self.writer.abandon()
      self.writer = None

  def end_drive(self) -> list[str]:
    queued: list[str] = []
    if self.writer is not None:
      try:
        queued = self.writer.end_drive()
        cloudlog.info(f"ramport: drive ended, {len(queued)} file(s) queued")
      except OSError:
        cloudlog.exception("ramport: could not finish the drive")
        self.writer.abandon()
      self.writer = None
    return queued


class SharedState:
  def __init__(self):
    self.onroad = False


def uploader_loop(uploader: QueueUploader, state: SharedState, exit_event: threading.Event) -> None:
  while not exit_event.is_set():
    try:
      wait = uploader.run_once(onroad=state.onroad)
    except Exception:
      cloudlog.exception("ramport: uploader step failed")
      wait = 60.0
    exit_event.wait(max(wait, 1.0))


def _fresh(sm, name: str, now_ns: int):
  if not sm.seen[name] or not sm.valid[name] or now_ns - sm.logMonoTime[name] > MAX_AGE_NS[name]:
    return None
  return sm[name]


def main() -> None:
  import openpilot.cereal.messaging as messaging
  from openpilot.common.realtime import Ratekeeper

  params = Params()
  daemon = RamPortDaemon(params)
  try:
    for path in daemon.recover():
      cloudlog.info(f"ramport: finished leftover part {os.path.basename(path)}")
    enforce_queue_limit(daemon.root)
  except Exception:
    cloudlog.exception("ramport: could not finish leftover parts")

  state = SharedState()
  exit_event = threading.Event()
  uploader = QueueUploader(daemon.root, Params())
  thread = threading.Thread(target=uploader_loop, args=(uploader, state, exit_event), daemon=True)
  thread.start()

  sm = messaging.SubMaster(["deviceState", "carState", "modelV2", "ramWeightGrade"])
  can_sock = messaging.sub_sock("can", timeout=0)
  rk = Ratekeeper(SAMPLE_HZ, print_delay_threshold=None)
  try:
    while True:
      sm.update(0)
      can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=False)
      started = sm["deviceState"].started
      state.onroad = started
      daemon.update_can(can_strings)
      now_ns = time.monotonic_ns()
      daemon.step(started, now_ns / 1e9, wall_time(), _fresh(sm, "carState", now_ns), _fresh(sm, "modelV2", now_ns),
                  _fresh(sm, "ramWeightGrade", now_ns), now_ns)
      rk.keep_time()
  finally:
    exit_event.set()
    daemon.end_drive()


# --- command-line tools ---

def synthetic_fields(i: int) -> list[str]:
  """Made-up but realistic values for sample number i, for test files only (clearly synthetic data)."""
  t = i / SAMPLE_HZ
  speed = 25.0 + 3.0 * math.sin(t / 30.0)
  values = {
    "speed_mps": speed, "accel_mps2": 0.1 * math.cos(t / 30.0), "accel_pedal_pct": 18.0 + 4.0 * math.sin(t / 7.0),
    "engine_rpm": 1500.0 + 100.0 * math.sin(t / 11.0), "engine_torque_nm": 600.0 + 80.0 * math.sin(t / 5.0),
    "gear_actual": 6, "gear_desired": 6, "converter_locked": 1, "shift_pending": 0,
    "trans_input_rpm": 1500.0, "trans_output_rpm": 1200.0, "brake_pressed": 0, "service_brake_on": 0, "exhaust_brake_on": 0,
    "tow_haul_on": 0, "trailer_connected": 0, "fuel_level_pct": 80.0 - t / 600.0, "grade": 0.02 * math.sin(t / 60.0),
    "grade_std": 0.005, "mass_kg": 3810.0, "mass_std_kg": 150.0, "drag_n_per_mps2": 0.75, "cruise_engaged": 1,
    "set_speed_mps": 26.8, "acc_engine_torque_req_nm": 610.0, "acc_brake_req": 0, "acc_decel_req_mps2": 4.0,
    "acc_distance_setting": 2, "lead_prob": 0.9, "lead_dist_m": 45.0 + 5.0 * math.sin(t / 13.0), "lead_speed_mps": speed - 0.5,
    "lead_accel_mps2": 0.0,
  }
  return [format_value(c, values.get(c)) for c in COLUMN_NAMES[1:]]


def make_test_file(out_root: str, minutes: float) -> list[str]:
  """Write a synthetic drive (made-up values) through the real drive writer. Returns the queued files."""
  meta = {"truck": "RAM_HD_5TH_GEN", "engine_part": "52370231AF", "transmission_part": "", "software": software_version()}
  writer = DriveWriter(out_root, lambda: meta)
  start_wall = wall_time() - minutes * 60.0
  for i in range(int(minutes * 60 * SAMPLE_HZ)):
    writer.add_sample(1000.0 + i / SAMPLE_HZ, start_wall + i / SAMPLE_HZ, synthetic_fields(i))
  return writer.end_drive()


def _cli(argv: list[str]) -> int:
  parser = argparse.ArgumentParser(description="Ram port shared data upload tools")
  parser.add_argument("--upload-file", help="send one drive file to the service once (registers first if needed)")
  parser.add_argument("--url", default=None, help="service address, for example http://localhost:7071 (default: the built-in address)")
  parser.add_argument("--key", default=None, help="private key file (PEM) to sign with instead of the device key")
  parser.add_argument("--serial", default=None, help="hardware serial number to register with (default: this device's)")
  parser.add_argument("--imei", default=None, help="cellular-modem ID to register with (default: this device's)")
  parser.add_argument("--make-test-file", metavar="FOLDER", help="write a synthetic drive file into FOLDER/queue")
  parser.add_argument("--minutes", type=float, default=10.0, help="length of the synthetic drive (default 10)")
  args = parser.parse_args(argv)

  if args.make_test_file:
    paths = make_test_file(args.make_test_file, args.minutes)
    print(json.dumps({"files": paths}))
    return 0 if paths else 1

  if args.upload_file:
    params = Params()
    if args.key:
      with open(args.key) as f:
        key = key_from_private_pem(f.read())
    else:
      key = load_device_key()
      if key is None:
        print("no device key found; pass --key", file=sys.stderr)
        return 2
    from openpilot.sunnypilot.ramport.uploader import default_hardware_ids

    def hardware_ids():
      serial, imei = ("", "") if (args.serial and args.imei) else default_hardware_ids(params)
      return (args.serial or serial), (args.imei or imei)

    host = (args.url or api_host()).rstrip("/")
    reaction, status, device_id = upload_file_once(args.upload_file, host, key, params, hardware_ids)
    print(json.dumps({"file": os.path.basename(args.upload_file), "status": status, "reaction": reaction.value,
                      "device_id": device_id, "note": "the device deletes the file on 'done'; this tool leaves it in place"}))
    return 0 if reaction == Reaction.DONE else 1

  parser.print_help()
  return 2


if __name__ == "__main__":
  if len(sys.argv) > 1:
    sys.exit(_cli(sys.argv[1:]))
  main()
