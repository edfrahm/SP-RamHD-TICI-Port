"""
Signal adapter and runner tests, on SYNTHETIC CAN frames packed with the Ram Heavy Duty database.
"""
import os
import tempfile

from opendbc.can.dbc import DBC
from opendbc.can.packer import CANPacker
from openpilot.cereal import log, messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.ramweightd import ram_signals as RS
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import LB_TO_KG
from openpilot.sunnypilot.selfdrive.ramweightd.runner import WeightGradeRunner
from openpilot.sunnypilot.selfdrive.ramweightd.tests.test_start_weight import car_params, vin

RAM_HD_DBC = "chrysler_ram_hd_generated"
DECODED_FIELDS = {"engineRpm", "engineTorqueNm", "gearActual", "gearDesired", "converterLocked", "shiftPending", "transInputRpm",
                  "transOutputRpm", "serviceBrakeOn", "exhaustBrakeOn", "towHaulOn", "trailerConnected", "fuelLevelPct"}


def can_event(t_nanos: int, frames):
  msg = messaging.new_message("can", len(frames))
  msg.logMonoTime = t_nanos
  for i, (addr, dat, bus) in enumerate(frames):
    msg.can[i].address = addr
    msg.can[i].dat = dat
    msg.can[i].src = bus
  return msg


def truck_frames(packer: CANPacker, transmission: bool = True, rpm=1500., torque=400., brake=0, gear_code=4):
  frames = [
    packer.make_can_msg("ECM_1", 0, {"ENGINE_RPM": rpm, "ENGINE_TORQUE": torque}),
    packer.make_can_msg("ESP_6", 0, {f"WHEEL_SPEED_{w}": 400. for w in ("FL", "FR", "RL", "RR")}),
    packer.make_can_msg("ESP_8", 0, {"Vehicle_Speed": 72., "BRK_PRESSURE": 0.}),
    packer.make_can_msg("ESP_1", 0, {"Brake_Pedal_State": brake}),
    packer.make_can_msg("ECM_5", 0, {"Accelerator_Position": 20.}),
    packer.make_can_msg("DAS_3", 2, {"ACC_ACTIVE": 0, "ENGINE_TORQUE_REQUEST": 0.}),
    (0x7FF, bytes(8), 0),
    (0x108, bytes(8), 1),  # ECM_1's address on a bus we do not read: must be ignored
  ]
  if transmission:
    frames.append(packer.make_can_msg("TCM_1", 0, {"TC_LOCKED": 1, "ACTUAL_GEAR": gear_code, "DESIRED_GEAR": gear_code,
                                                   "INPUT_SPEED": 1500., "OUTPUT_SPEED": 1500.}))
    frames.append(packer.make_can_msg("ESP_4", 0, {"Acceleration": 0.4}))
  return frames


def car_state_reader(v=20., gear="drive"):
  msg = messaging.new_message("carState")
  msg.carState.vEgoRaw = v
  msg.carState.gearShifter = gear
  return msg.carState


class TestMappingTable(OpenpilotTestCase):
  def test_table_is_well_formed(self):
    for name, spec in RS.RAM_SIGNALS.items():
      assert spec.status in (RS.KNOWN, RS.UNCHECKED, RS.PENDING, RS.MISSING), name
      assert spec.bus in (RS.BUS_TRUCK, RS.BUS_CAMERA), name
      assert (spec.message is None) == (spec.status == RS.MISSING), name
    for field, needs in RS.PUBLISHED_FIELDS.items():
      assert field in DECODED_FIELDS and all(n in RS.RAM_SIGNALS for n in needs)

  def test_known_signals_are_in_the_database_and_missing_ones_are_not(self):
    dbc = DBC(RAM_HD_DBC)
    for name, spec in RS.RAM_SIGNALS.items():
      in_db = spec.message in dbc.name_to_msg and spec.signal in dbc.name_to_msg[spec.message].sigs
      if spec.status in (RS.KNOWN, RS.UNCHECKED):
        assert in_db, f"{name}: {spec.message}.{spec.signal} should be in {RAM_HD_DBC}"
      else:
        assert not in_db, f"{name} is in the database now: mark it known once verified (#5)"
    reader = RS.RamSignalReader(RAM_HD_DBC)
    assert set(reader.not_in_database) == {n for n, s in RS.RAM_SIGNALS.items() if s.status in (RS.PENDING, RS.MISSING)}
    # verified in #5
    for name in ("converter_locked", "gear_code_actual", "gear_code_desired", "trans_input_rpm", "stability_accel_mps2"):
      assert reader.in_database(name), name

  def test_only_the_needed_messages_are_parsed(self):
    reader = RS.RamSignalReader(RAM_HD_DBC)
    dbc = DBC(RAM_HD_DBC)
    needed = {dbc.name_to_msg[s.message].address for n, s in RS.RAM_SIGNALS.items() if n in reader.specs}
    parsed = set().union(*(set(p.message_states) for p in reader.parsers.values()))
    assert parsed == needed
    assert len(parsed) <= 10
    assert set(reader.parsers) == {RS.BUS_TRUCK, RS.BUS_CAMERA}

  def test_gear_codes(self):
    assert [RS.gear_from_code(c) for c in (1, 2, 9, 3, 4, 5)] == [1, 2, 3, 4, 5, 6]
    assert RS.gear_from_code(0) == 0 and RS.gear_from_code(11) == 0 and RS.gear_from_code(7) == 0
    assert RS.gear_from_code(None) is None


class TestReader(OpenpilotTestCase):
  def test_decode_and_staleness(self):
    reader = RS.RamSignalReader(RAM_HD_DBC)
    packer = CANPacker(RAM_HD_DBC)
    t = 1_000_000_000
    for i in range(5):
      reader.update_from_events([can_event(t + i * 20_000_000, truck_frames(packer, rpm=1500. + i))])
    now = t + 4 * 20_000_000
    assert reader.get("engine_rpm", now) == 1504.
    assert reader.get("engine_torque_nm", now) == 400.
    assert reader.get("wheel_rpm_rl", now) == 400.
    assert reader.get("brake_pedal", now) == 0.
    assert reader.get("acc_active", now) == 0.
    assert reader.get("converter_locked", now) == 1. and reader.get("gear_code_actual", now) == 4.
    assert reader.get("fuel_level_pct", now) is None      # not in the database
    assert reader.get("engine_rpm", now + 2_000_000_000) is None  # too old
    assert reader.published_fields(now) == ["converterLocked", "engineRpm", "engineTorqueNm", "gearActual", "gearDesired", "serviceBrakeOn",
                                            "shiftPending", "transInputRpm", "transOutputRpm"]

  def test_frames_on_other_buses_are_ignored(self):
    reader = RS.RamSignalReader(RAM_HD_DBC)
    packer = CANPacker(RAM_HD_DBC)
    frames = [packer.make_can_msg("ECM_1", 1, {"ENGINE_RPM": 900.}), packer.make_can_msg("DAS_3", 0, {"ACC_ACTIVE": 1})]
    reader.update_from_events([can_event(1_000_000_000, frames)])
    assert reader.get("engine_rpm") is None and reader.get("acc_active") is None


class TestRunner(OpenpilotTestCase):
  def runner(self, dbc=RAM_HD_DBC, **kw):
    return WeightGradeRunner(car_params(vin()), dbc_name=dbc, **kw)

  def feed(self, runner, dbc=RAM_HD_DBC, transmission=True, n=30, pitch=False, t0=1_000_000_000):
    packer = CANPacker(dbc)
    out = None
    for i in range(n):
      t = t0 + i * 100_000_000
      runner.handle_can_events([can_event(t, truck_frames(packer, transmission))])
      runner.handle_car_state(car_state_reader())
      if pitch:
        dm = messaging.new_message("deviceMotion")
        dm.deviceMotion.orientationNED.valid = True
        dm.deviceMotion.orientationNED.yStd = 0.003
        runner.handle_device_motion(dm.deviceMotion, True)
        ec = messaging.new_message("extrinsicsCalibration")
        ec.extrinsicsCalibration.rpyCalib = [0., 0.02, 0.]
        ec.extrinsicsCalibration.calStatus = log.ExtrinsicsCalibration.Status.calibrated
        runner.handle_extrinsics(ec.extrinsicsCalibration)
      out = runner.step(t)
    return out, t

  def message(self, runner, t):
    msg = messaging.new_message("ramWeightGrade")
    runner.fill(msg.ramWeightGrade, t)
    return msg

  def test_available_signals_without_transmission_messages(self):
    r = self.runner()
    assert r.available_fields(0) == []  # the filter has not run yet
    out, t = self.feed(r, transmission=False)
    fields = set(self.message(r, t).ramWeightGrade.availableSignals)
    # estimates: weight and drag once the filter runs; no grade measurement yet
    assert {"massKg", "massStdKg", "dragNPerMps2"} <= fields
    assert "grade" not in fields and "gradeStd" not in fields
    # decoded signals: only those really decoded; converterLocked from the fallback rule is not listed
    assert fields & DECODED_FIELDS == {"engineRpm", "engineTorqueNm", "serviceBrakeOn"}
    assert out.converter_lock_rule == "ratioFallback" and out.converter_locked

  def test_available_signals_today(self):
    r = self.runner()
    out, t = self.feed(r, pitch=True)
    rwg = self.message(r, t).ramWeightGrade
    fields = set(rwg.availableSignals)
    assert {"massKg", "massStdKg", "dragNPerMps2", "grade", "gradeStd"} <= fields
    assert fields & DECODED_FIELDS == {"engineRpm", "engineTorqueNm", "serviceBrakeOn", "converterLocked", "gearActual", "gearDesired",
                                       "shiftPending", "transInputRpm", "transOutputRpm"}
    assert out.converter_lock_rule == "signal" and rwg.converterLocked
    assert rwg.gearActual == 5 and rwg.gearDesired == 5 and not rwg.shiftPending  # TCM_1 code 4 is fifth gear
    assert out.grade_source == "sensorOnly" and str(rwg.gradeSource) == "sensorOnly"  # ESP_4; the pitch is not used

  def test_shift_from_gears(self):
    r = self.runner()
    packer = CANPacker(RAM_HD_DBC)
    t0 = 1_000_000_000
    shifts = []
    for i in range(40):
      t = t0 + i * 100_000_000
      actual, desired = (3, 3) if i < 10 else (3, 4) if i < 13 else (4, 4)
      frames = [f for f in truck_frames(packer) if f[0] != 0x144]
      frames.append(packer.make_can_msg("TCM_1", 0, {"TC_LOCKED": 1, "ACTUAL_GEAR": actual, "DESIRED_GEAR": desired, "INPUT_SPEED": 1500.}))
      r.handle_can_events([can_event(t, frames)])
      r.handle_car_state(car_state_reader())
      r.step(t)
      shifts.append(r.last_inputs.shift_pending)
    assert not any(shifts[1:10])
    assert all(shifts[10:13])          # desired gear differs from the actual one
    assert all(shifts[13:13 + 10])     # the gear just changed: one second
    assert not any(shifts[24:])

  def test_message_contents(self):
    r = self.runner(base_weight_lb=8400., known_extra_lb=250.)
    out, t = self.feed(r)
    msg = self.message(r, t)
    rwg = msg.ramWeightGrade
    assert str(rwg.startSource) == "setting"
    assert abs(rwg.knownExtraKg - 250. * LB_TO_KG) < 1e-3
    assert abs(rwg.startMassKg - (8400. - 0.5 * r.start.tank_gal * r.start.lb_per_gal) * LB_TO_KG) < 0.1
    assert rwg.engineRpm == 1500. and rwg.engineTorqueNm == 400.
    assert rwg.configuration == r.configuration_text
    assert str(rwg.converterLockRule) == "signal"
    # the known extra load is published only: the estimator does not know it
    r2 = self.runner(base_weight_lb=8400., known_extra_lb=0.)
    out2, _ = self.feed(r2)
    assert out2.mass_kg == out.mass_kg and out2.mass_std_kg == out.mass_std_kg
    # the VIN is never in the message
    data = msg.to_bytes()
    assert vin().encode() not in data and vin()[-6:].encode() not in data

  def test_kept_estimate_only_for_the_same_truck(self):
    r = self.runner()
    self.feed(r)
    state = r.state()
    assert state["platform"] == "RAM_HD_5TH_GEN" and "configuration" in state
    assert vin() not in str(state)
    assert self.runner(persisted=state).persisted_used
    assert not self.runner(persisted={**state, "configuration": "another truck"}).persisted_used
    assert not self.runner(persisted={**state, "platform": "RAM_1500_5TH_GEN"}).persisted_used
    assert not self.runner(persisted={"garbage": 1}).persisted_used
    assert not self.runner(persisted=None).persisted_used

  def test_settings_change_moves_only_the_reference(self):
    r = self.runner()
    self.feed(r)
    mass = r.estimator.mass_kg
    assert r.start.source == "fallback"
    r.set_settings(8400., 100.)
    assert r.start.source == "setting" and r.known_extra_lb == 100.
    assert r.estimator.mass_kg == mass


class TestReplayHelpers(OpenpilotTestCase):
  def test_find_route_segments_in_order(self):
    from openpilot.sunnypilot.selfdrive.ramweightd.replay import find_log_files
    root = tempfile.mkdtemp(prefix="ramweightd_root_")
    for seg in (0, 1, 2, 10):
      os.makedirs(os.path.join(root, f"0000abcd--1234567890--{seg}"))
      open(os.path.join(root, f"0000abcd--1234567890--{seg}", "rlog.zst"), "w").close()
    files = find_log_files(["0000abcd--1234567890"], root)
    assert [int(f.split("--")[-1].split("/")[0]) for f in files] == [0, 1, 2, 10]
    assert find_log_files([os.path.join(root, "0000abcd--1234567890--2")], root) == [files[2]]
    with self.assertRaises(FileNotFoundError):
      find_log_files(["nothere"], root)
