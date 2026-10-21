"""
Tests for the ramportd writer, with synthetic (made-up) message streams. No real drive data is used.
"""
import gzip
import json
import os
import re
import statistics

from opendbc.car.structs import car
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot import ramport
from openpilot.sunnypilot.ramport.drive_format import (COLUMN_NAMES, FILE_NAME_RE, LIMITS, METADATA_KEYS, format_value,
                                                       read_part_file)
from openpilot.sunnypilot.ramport.drive_writer import DriveWriter, building_dir, enforce_queue_limit, queue_dir, queued_files
from openpilot.sunnypilot.ramport.ramportd import RamPortDaemon, read_static_metadata
from openpilot.sunnypilot.ramport.sampler import RAM_WEIGHT_GRADE_FIELDS
from openpilot.sunnypilot.ramport.tests import synthetic
from openpilot.sunnypilot.ramport.tests.synthetic import FAKE_DONGLE_ID, FAKE_ROUTE, FAKE_VIN, START_WALL

COL = {name: i for i, name in enumerate(COLUMN_NAMES)}
STATIC_META = {"truck": "RAM_HD_5TH_GEN", "engine_part": "52370231AF", "transmission_part": "", "software": "2026.003.000 abcdef123"}


def sample_index(row) -> int:
  return int(round(float(row[COL["speed_mps"]]) * 1000))


class RamPortTestCase(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.params = Params()
    self.params.put_bool("EnableSunnylinkUploader", True, block=True)
    self.params.put("CarParams", synthetic.car_params_bytes(), block=True)
    self.params.put("DongleId", FAKE_DONGLE_ID, block=True)
    self.params.put("HardwareSerial", "a1b2c3d4", block=True)
    self.root = Paths.ramport_root()

  def daemon(self, meta=None):
    return RamPortDaemon(self.params, self.root, meta_provider=(lambda: meta) if meta is not None else None)


class TestDriveWriter(RamPortTestCase):
  def test_queue_folder_is_under_media(self):
    assert self.root.endswith(os.path.join("media", "0", "ramport"))
    assert self.root.startswith(Paths.comma_home())  # on the Mac, inside this test's own folder

  def test_trims_both_ends_and_writes_exact_layout(self):
    d = self.daemon(STATIC_META)
    synthetic.drive(d, 600.0)  # 10 minutes, samples 0..5999

    files = queued_files(self.root)
    assert len(files) == 1
    name = os.path.basename(files[0])
    m = FILE_NAME_RE.match(name)
    assert m and m.group(2) == "01", name

    meta, header, rows = read_part_file(files[0])
    assert list(meta.keys()) == METADATA_KEYS
    assert meta["format"] == "ramport-drive" and meta["version"] == 1
    assert meta["truck"] == "RAM_HD_5TH_GEN" and meta["engine_part"] == "52370231AF" and meta["transmission_part"] == ""
    assert meta["drive_key"] == m.group(1) and re.fullmatch(r"[0-9a-f]{16}", meta["drive_key"])
    assert meta["part"] == 1 and meta["rows"] == len(rows) and meta["sample_hz"] == 10
    assert meta["trim_start_s"] == 120 and meta["trim_end_s"] == 120
    assert meta["date"] == "2026-10-14"
    assert header == COLUMN_NAMES

    first, last = sample_index(rows[0]), sample_index(rows[-1])
    assert first == 1200, "the first 2 minutes (samples 0..1199) must not be written"
    assert 5999 - last >= 1199, "the last 2 minutes must not be written"
    assert 5999 - last <= 1201
    indexes = [sample_index(r) for r in rows]
    assert indexes == list(range(first, last + 1)), "no sample in between may be lost"

    times = [float(r[0]) for r in rows]
    assert times[0] == 0.0
    gaps = [b - a for a, b in zip(times, times[1:], strict=False)]
    assert all(g > 0 for g in gaps), "t_s must strictly increase"
    assert 0.09 <= statistics.median(gaps) <= 0.11
    assert all(len(r) == len(COLUMN_NAMES) for r in rows)

    # nothing left behind in the building folder, nothing more in memory
    assert os.listdir(building_dir(self.root)) == []

  def test_values_and_empty_fields(self):
    d = self.daemon(STATIC_META)
    synthetic.drive(d, 400.0, available=("engineRpm", "massKg", "gradeStd", "tow_haul_on"), das4_until_s=200.0, with_model=False)
    meta, _, rows = read_part_file(queued_files(self.root)[0])
    row = rows[0]
    # carState
    assert row[COL["accel_mps2"]] == "0.25" and row[COL["cruise_engaged"]] == "1" and row[COL["set_speed_mps"]] == "26.822"
    assert row[COL["brake_pressed"]] in ("0", "1")
    # ramWeightGrade: only the listed weight fields (message or column spelling); the grade and the truck signals it
    # lists are ignored (it publishes only when the estimate changes, and no longer publishes a grade)
    assert row[COL["mass_kg"]] == "3810"
    for column in ("grade", "grade_std", "tow_haul_on", "exhaust_brake_on", "trailer_connected", "mass_std_kg", "drag_n_per_mps2"):
      assert row[COL[column]] == "", column
    # the engine, transmission, brake and fuel gauge columns come from the bus, decoded as the weight estimator does
    assert row[COL["engine_rpm"]] == "1450" and row[COL["engine_torque_nm"]] == "812.5"
    assert row[COL["gear_actual"]] == "6" and row[COL["gear_desired"]] == "6" and row[COL["shift_pending"]] == "0"
    assert row[COL["converter_locked"]] == "1" and row[COL["service_brake_on"]] == "0"
    assert row[COL["trans_input_rpm"]] == "1450" and row[COL["trans_output_rpm"]] == "1100"
    assert abs(float(row[COL["fuel_level_pct"]]) - 76.4) < 0.4
    # no modelV2: lead columns empty
    for column in ("lead_prob", "lead_dist_m", "lead_speed_mps", "lead_accel_mps2"):
      assert row[COL[column]] == "", column
    # controller area network messages, decoded with the Ram Heavy Duty database
    assert row[COL["accel_pedal_pct"]] == "22.4"
    assert row[COL["acc_engine_torque_req_nm"]] == "640.25"
    assert row[COL["acc_brake_req"]] == "0"
    assert abs(float(row[COL["acc_decel_req_mps2"]]) - 4.0) < 0.005
    assert row[COL["acc_distance_setting"]] == "2"
    # DAS_4 stopped at 200 s (into the drive): more than 1 s later its column is empty
    late = [r for r in rows if sample_index(r) >= 2020]
    assert late and all(r[COL["acc_distance_setting"]] == "" for r in late)
    assert all(r[COL["acc_engine_torque_req_nm"]] == "640.25" for r in late)

  def test_truck_columns_only_from_the_bus(self):
    # no engine, transmission, brake or fuel messages on the bus: those columns stay empty even when ramWeightGrade lists
    # them, because its values are snapshots from when the estimate last changed
    d = self.daemon(STATIC_META)
    synthetic.drive(d, 250.0, available=("engineRpm", "gearActual", "fuelLevelPct", "massKg"), truck=False)
    _, _, rows = read_part_file(queued_files(self.root)[0])
    for column in ("engine_rpm", "engine_torque_nm", "gear_actual", "gear_desired", "shift_pending", "converter_locked",
                   "trans_input_rpm", "trans_output_rpm", "service_brake_on", "fuel_level_pct"):
      assert all(r[COL[column]] == "" for r in rows), column
    assert rows[0][COL["mass_kg"]] == "3810"

  def test_model_lead_columns(self):
    d = self.daemon(STATIC_META)
    synthetic.drive(d, 250.0)
    _, _, rows = read_part_file(queued_files(self.root)[0])
    row = rows[0]
    assert row[COL["lead_prob"]] == "0.8" and row[COL["lead_speed_mps"]] == "24.5" and row[COL["lead_accel_mps2"]] == "-0.3"
    assert 40.0 <= float(row[COL["lead_dist_m"]]) <= 49.0

  def test_every_value_within_limits_and_plain_decimals(self):
    d = self.daemon(STATIC_META)
    synthetic.drive(d, 300.0, available=list(RAM_WEIGHT_GRADE_FIELDS.values()))
    _, _, rows = read_part_file(queued_files(self.root)[0])
    number = re.compile(r"^-?\d+(\.\d{1,4})?$")
    for row in rows:
      for name, text in zip(COLUMN_NAMES, row, strict=True):
        if text == "":
          continue
        assert number.match(text), (name, text)
        if name in LIMITS:
          lo, hi = LIMITS[name]
          assert lo <= float(text) <= hi, (name, text)

  def test_no_forbidden_fields(self):
    """No location, VIN, route name, dongle ID or serial number anywhere in the file, nor in the gzip header."""
    self.params.put("CarParams", synthetic.car_params_bytes(), block=True)
    d = RamPortDaemon(self.params, self.root)  # the real metadata path, from the CarParams setting
    synthetic.drive(d, 300.0)
    path = queued_files(self.root)[0]
    raw = open(path, "rb").read()
    assert raw[3] & 0x08 == 0, "gzip header must not carry a file name"
    assert raw[4:8] == b"\x00\x00\x00\x00", "gzip header must not carry a time"
    text = gzip.decompress(raw).decode()
    lowered = text.lower()
    for word in ("latitude", "longitude", "altitude", "lat", "lon", "gps", "heading", "bearing", "vin", "dongle", "route", "serial",
                 "imei", "location"):
      assert not re.search(rf"\b{word}", lowered.replace("_", " ")), word
    for value in (FAKE_VIN, FAKE_DONGLE_ID, FAKE_ROUTE, "a1b2c3d4"):
      assert value not in text
    assert not re.search(r"\b[A-HJ-NPR-Z0-9]{17}\b", text), "nothing that looks like a VIN"
    assert not re.search(r"\b\d{15}\b", text), "nothing that looks like a modem ID"
    meta = json.loads(text.split("\n", 1)[0][len("#RAMPORT "):])
    assert meta["engine_part"] == "52370231AF"

  def test_hour_split(self):
    w = DriveWriter(self.root, lambda: STATIC_META)
    total = 75 * 60 * 10  # 75 minutes
    for i in range(total):
      w.add_sample(100.0 + i * 0.1, START_WALL + i * 0.1, [str(i)] + [""] * (len(COLUMN_NAMES) - 2))
    paths = w.end_drive()
    assert [os.path.basename(p)[-10:] for p in paths] == ["p01.csv.gz", "p02.csv.gz"]
    m1, _, r1 = read_part_file(paths[0])
    m2, _, r2 = read_part_file(paths[1])
    assert m1["rows"] == len(r1) == 36000
    assert (m1["trim_start_s"], m1["trim_end_s"]) == (120, 0)
    assert (m2["trim_start_s"], m2["trim_end_s"]) == (0, 120)
    assert m1["drive_key"] == m2["drive_key"] and m2["part"] == 2
    assert float(r1[-1][0]) < 3600.0 and float(r2[0][0]) == 0.0
    assert int(r2[0][1]) == int(r1[-1][1]) + 1, "parts follow each other with no lost sample"
    assert m2["rows"] == len(r2)
    assert abs(m2["rows"] - (total - 2 * 1200 - 36000 - 1)) <= 1

  def test_full_part_at_drive_end_is_the_last_part(self):
    w = DriveWriter(self.root, lambda: STATIC_META)
    n = 1200 + 36000 + 1201  # trimmed start, exactly one hour, trimmed end
    for i in range(n):
      w.add_sample(100.0 + i * 0.1, START_WALL + i * 0.1, [str(i)] + [""] * (len(COLUMN_NAMES) - 2))
    paths = w.end_drive()
    assert len(paths) == 1
    meta, _, rows = read_part_file(paths[0])
    assert meta["rows"] == 36000 and meta["trim_end_s"] == 120

  def test_short_drive_writes_nothing(self):
    synthetic.drive(self.daemon(STATIC_META), 239.0)
    assert queued_files(self.root) == []
    assert os.listdir(building_dir(self.root)) == []

  def test_part_date_is_utc_date_of_its_first_row(self):
    w = DriveWriter(self.root, lambda: STATIC_META)
    midnight = 1_792_108_800.0  # 2026-10-16 00:00:00 UTC
    start_wall = midnight - 120.0 - 30.0  # first written row 30 s before midnight
    for i in range(3000):
      w.add_sample(i * 0.1, start_wall + i * 0.1, ["1"] + [""] * (len(COLUMN_NAMES) - 2))
    meta, _, _ = read_part_file(w.end_drive()[0])
    assert meta["date"] == "2026-10-15"

  def test_toggle_off_at_drive_start_writes_nothing(self):
    self.params.put_bool("EnableSunnylinkUploader", False, block=True)
    d = self.daemon(STATIC_META)
    can = synthetic.CanStream()
    for i in range(4000):
      if i == 100:
        self.params.put_bool("EnableSunnylinkUploader", True, block=True)  # switched on during the drive: still not recorded
      mono = 10.0 + i * 0.1
      d.update_can([can.message_bytes(int(mono * 1e9))])
      d.step(True, mono, START_WALL + i * 0.1, synthetic.car_state(i), synthetic.model(i), None, int(mono * 1e9))
    d.step(False, 500.0, START_WALL + 500.0)
    assert queued_files(self.root) == []

  def test_toggle_switched_off_during_drive_discards_it(self):
    d = self.daemon(STATIC_META)
    for i in range(5000):
      if i == 3000:
        self.params.put_bool("EnableSunnylinkUploader", False, block=True)
      d.step(True, 10.0 + i * 0.1, START_WALL + i * 0.1, synthetic.car_state(i))
    d.step(False, 600.0, START_WALL + 600.0)
    assert queued_files(self.root) == []
    assert os.listdir(building_dir(self.root)) == []

  def test_next_drive_gets_a_new_key(self):
    d = self.daemon(STATIC_META)
    synthetic.drive(d, 300.0)
    synthetic.drive(d, 300.0, start_index=10000)
    files = queued_files(self.root)
    assert len(files) == 2
    keys = {read_part_file(p)[0]["drive_key"] for p in files}
    assert len(keys) == 2

  def test_leftover_part_is_finished_after_a_stop(self):
    d = self.daemon(STATIC_META)
    synthetic.drive(d, 400.0, started_after=True)  # the process stops mid-drive: no drive end
    d.writer.part.file.flush()
    del d
    assert queued_files(self.root) == []
    d2 = self.daemon(STATIC_META)
    recovered = d2.recover()
    assert len(recovered) == 1
    meta, _, rows = read_part_file(recovered[0])
    assert meta["trim_start_s"] == 120 and meta["trim_end_s"] == 120 and meta["rows"] == len(rows) > 0
    assert sample_index(rows[-1]) <= 3999 - 1199, "rows written before the stop already exclude its last 2 minutes"
    assert os.listdir(building_dir(self.root)) == []

  def test_queue_limit_deletes_oldest_first(self):
    os.makedirs(queue_dir(self.root), exist_ok=True)
    paths = []
    for k in range(5):
      p = os.path.join(queue_dir(self.root), f"{k:016x}_p01.csv.gz")
      with open(p, "wb") as f:
        f.write(b"x" * 1000)
      os.utime(p, (1000 + k, 1000 + k))
      paths.append(p)
    deleted = enforce_queue_limit(self.root, 3000)
    assert deleted == paths[:2]
    assert queued_files(self.root) == paths[2:]


class TestFormat(OpenpilotTestCase):
  def test_number_format(self):
    assert format_value("speed_mps", 12.0) == "12"
    assert format_value("speed_mps", 12.34567) == "12.346"
    assert format_value("grade", 1e-7) == "0"
    assert format_value("grade", -1e-7) == "0"
    assert format_value("grade_std", 0.00012) == "0.0001"
    assert "e" not in format_value("lead_accel_mps2", -3.2e-5)
    assert format_value("speed_mps", float("nan")) == ""
    assert format_value("speed_mps", None) == ""
    assert format_value("cruise_engaged", True) == "1" and format_value("cruise_engaged", False) == "0"
    assert format_value("gear_actual", 6) == "6"

  def test_limits(self):
    assert format_value("acc_decel_req_mps2", 4.0042) == "4", "the encoding's top value is set to the limit"
    assert format_value("speed_mps", -0.004) == "0"
    assert format_value("accel_mps2", 50.0) == "", "far outside: not available"
    assert format_value("mass_kg", 0.0) == "", "an estimator that has not started yet"
    assert format_value("gear_actual", -1) == ""


class TestRamHdOnly(OpenpilotTestCase):
  def test_is_ram_hd(self):
    params = Params()
    cp = car.CarParams.new_message()
    cp.carFingerprint = "RAM_HD_5TH_GEN"
    assert ramport.is_ram_hd(params, cp)
    cp.carFingerprint = "RAM_1500_5TH_GEN"
    assert not ramport.is_ram_hd(params, cp)

  def test_is_ram_hd_from_last_drive_when_parked(self):
    params = Params()
    params.put("CarParamsPersistent", synthetic.car_params_bytes(), block=True)
    ramport._persistent_fingerprint = None
    try:
      assert ramport.is_ram_hd(params, car.CarParams.new_message())
      ramport._persistent_fingerprint = None
      params.put("CarParamsPersistent", synthetic.car_params_bytes("TOYOTA_RAV4"), block=True)
      assert not ramport.is_ram_hd(params, car.CarParams.new_message())
    finally:
      ramport._persistent_fingerprint = None

  def test_process_is_registered(self):
    from openpilot.system.manager.process_config import managed_processes
    proc = managed_processes["ramportd"]
    assert proc.module == "openpilot.sunnypilot.ramport.ramportd"
    cp = car.CarParams.new_message()
    cp.carFingerprint = "RAM_HD_5TH_GEN"
    assert proc.should_run(False, Params(), cp) and proc.should_run(True, Params(), cp)
    cp.carFingerprint = "HONDA_CIVIC"
    assert not proc.should_run(True, Params(), cp)

  def test_static_metadata(self):
    params = Params()
    params.put("CarParams", synthetic.car_params_bytes(with_transmission=True), block=True)
    meta = read_static_metadata(params)
    assert meta["truck"] == "RAM_HD_5TH_GEN" and meta["engine_part"] == "52370231AF" and meta["transmission_part"] == "68376876AC"
    assert re.fullmatch(r"\d{4}\.\d{3}\.\d{3} [0-9a-f]{9}", meta["software"]), meta["software"]
