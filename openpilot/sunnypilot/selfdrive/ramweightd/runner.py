"""
Turns openpilot messages into estimator samples and estimator results into the ramWeightGrade message.

Shared by the on-device service (ramweightd.py) and the offline tools (replay.py, calibrate.py), so a recorded drive is
processed exactly as on the truck. Inputs:
- can: the Ram signals of ram_signals.RAM_SIGNALS (only those messages are decoded)
- carState: speed from wheel speed (vEgoRaw), shifter position, doors
- deviceMotion: openpilot's location filter (locationd); its orientation is the Comma's pose in the world
- extrinsicsCalibration: the Comma's mounting angles in the truck (calibrationd), used to turn that pose into the
  truck's pitch (the same PoseCalibrator controlsd uses)
"""
import math

from opendbc.car import Bus
from opendbc.car.chrysler.values import DBC
from openpilot.selfdrive.locationd.helpers import Pose, PoseCalibrator
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import Inputs, LB_TO_KG, Output, WeightGradeEstimator, valid_persisted_state
from openpilot.sunnypilot.selfdrive.ramweightd.ram_signals import RamSignalReader
from openpilot.sunnypilot.selfdrive.ramweightd.start_weight import (UNKNOWN_FUEL_FRACTION, Configuration, StartWeight, WeightTable,
                                                                    choose_start_weight, identify_configuration, load_weight_table)

RAM_HD_BRAND = "chrysler"
RAM_HD_PLATFORM = "RAM_HD_5TH_GEN"
STEP_HZ = 10.
STEP_NANOS = int(1e9 / STEP_HZ)
ESTIMATE_FIELDS = ["massKg", "massStdKg", "dragNPerMps2"]
GRADE_FIELDS = ["grade", "gradeStd"]


def is_ram_hd(CP) -> bool:
  return CP.brand == RAM_HD_BRAND and CP.carFingerprint == RAM_HD_PLATFORM


def _bool(x: float | None) -> bool | None:
  return None if x is None else x > 0.5


class WeightGradeRunner:
  def __init__(self, CP, base_weight_lb: float = 0., known_extra_lb: float = 0., persisted: dict | None = None,
               table: WeightTable | None = None, dbc_name: str | None = None, record_windows: bool = False,
               table_path: str | None = None):
    self.CP = CP
    self.reader = RamSignalReader(dbc_name or DBC[CP.carFingerprint][Bus.pt])
    self.table = table if table is not None else load_weight_table(table_path)
    self.configuration: Configuration = identify_configuration(CP, self.table)
    self.configuration_text = self.configuration.text
    self.base_weight_lb = base_weight_lb
    self.known_extra_lb = known_extra_lb
    self.start: StartWeight = self._choose_start(None)

    # the kept estimate is used only when it belongs to this configuration
    self.persisted_used = bool(persisted) and valid_persisted_state(persisted) and \
      persisted.get("configuration") == self.configuration_text and persisted.get("platform") == CP.carFingerprint
    assumed_level = None if self.start.source == "fallback" else UNKNOWN_FUEL_FRACTION * 100.
    self.estimator = WeightGradeEstimator(self.start.mass_kg, self.start.std_kg, self.start.tank_gal, self.start.lb_per_gal,
                                          persisted if self.persisted_used else None, record_windows=record_windows,
                                          start_fuel_level_pct=assumed_level)

    self.calibrator = PoseCalibrator()
    self.car_state = None
    self.device_motion = None
    self.device_motion_valid = False
    self.last_output: Output | None = None
    self.last_inputs: Inputs | None = None

  def _choose_start(self, fuel_level_pct: float | None) -> StartWeight:
    return choose_start_weight(self.configuration, self.table, self.base_weight_lb, fuel_level_pct, self.CP.mass)

  def set_settings(self, base_weight_lb: float, known_extra_lb: float) -> None:
    """Settings may change between drives or during one. The base weight only changes the published starting weight
    (the reference for measuring accuracy); the estimate itself is not restarted."""
    self.known_extra_lb = known_extra_lb
    if base_weight_lb != self.base_weight_lb:
      self.base_weight_lb = base_weight_lb
      self.start = self._choose_start(self.estimator.fuel.first_level_pct)

  # ---- message input ----
  def handle_can_events(self, events) -> None:
    self.reader.update_from_events(events)

  def handle_car_state(self, car_state) -> None:
    self.car_state = car_state

  def handle_device_motion(self, device_motion, valid: bool) -> None:
    self.device_motion = device_motion
    self.device_motion_valid = valid and device_motion.orientationNED.valid

  def handle_extrinsics(self, extrinsics) -> None:
    self.calibrator.feed_extrinsics_calibration(extrinsics)

  def handle(self, which: str, msg, valid: bool = True) -> None:
    """msg is the message body (for example evt.carState), except for can, which takes the whole event."""
    if which == "can":
      self.handle_can_events([msg])
    elif which == "carState":
      self.handle_car_state(msg)
    elif which == "deviceMotion":
      self.handle_device_motion(msg, valid)
    elif which == "extrinsicsCalibration":
      self.handle_extrinsics(msg)

  # ---- one 10 Hz step ----
  def inputs(self, now_nanos: int) -> Inputs:
    r = self.reader

    def get(name):
      return r.get(name, now_nanos)

    cs = self.car_state
    v = float(cs.vEgoRaw) if cs is not None else 0.
    rl, rr = get("wheel_rpm_rl"), get("wheel_rpm_rr")
    rear = 0.5 * (rl + rr) if rl is not None and rr is not None else None

    pitch = pitch_std = lateral = None
    if self.device_motion is not None and self.device_motion_valid and self.calibrator.calib_valid:
      pose = self.calibrator.build_calibrated_pose(Pose.from_device_motion(self.device_motion))
      pitch = float(pose.orientation.pitch)
      pitch_std = float(self.device_motion.orientationNED.yStd)
      lateral = float(pose.angular_velocity.yaw) * v

    return Inputs(
      t=now_nanos * 1e-9,
      v=v,
      gear=str(cs.gearShifter) if cs is not None else "unknown",
      door_open=bool(cs.doorOpen) if cs is not None else None,
      brake_pedal=_bool(get("brake_pedal")),
      brake_pressure=get("brake_pressure"),
      accel_pedal_pct=get("accel_pedal_pct"),
      acc_active=_bool(get("acc_active")),
      acc_torque_request_nm=get("acc_torque_request_nm"),
      engine_rpm=get("engine_rpm"),
      engine_torque_nm=get("engine_torque_nm"),
      rear_wheel_rpm=rear,
      traction_control=_bool(get("traction_control_active")),
      stability_accel=get("stability_accel_mps2"),
      converter_locked=_bool(get("converter_locked")),
      shift_pending=_bool(get("shift_pending")),
      exhaust_brake=_bool(get("exhaust_brake_on")),
      trailer_connected=_bool(get("trailer_connected")),
      fuel_level_pct=get("fuel_level_pct"),
      pitch=pitch,
      pitch_std=pitch_std,
      lateral_accel=lateral,
    )

  def step(self, now_nanos: int) -> Output:
    inp = self.inputs(now_nanos)
    first_level = self.estimator.fuel.first_level_pct
    out = self.estimator.step(inp)
    if first_level is None and self.estimator.fuel.first_level_pct is not None and self.start.source != "fallback":
      # the fuel level became known: the starting weight no longer needs the half-tank assumption
      self.start = self._choose_start(self.estimator.fuel.first_level_pct)
    self.last_inputs, self.last_output = inp, out
    return out

  def fill(self, rwg, now_nanos: int) -> None:
    """Fill a ramWeightGrade message from the last step."""
    out, inp, r = self.last_output, self.last_inputs, self.reader
    if out is None or inp is None:
      return

    def num(name, default=0.):
      x = r.get(name, now_nanos)
      return default if x is None else x

    rwg.massKg = float(out.mass_kg)
    rwg.massStdKg = float(out.mass_std_kg)
    rwg.dragNPerMps2 = float(out.drag)
    rwg.dragStdNPerMps2 = float(out.drag_std)
    rwg.grade = float(out.grade)
    rwg.gradeStd = float(out.grade_std)
    rwg.cleanMoment = out.clean
    rwg.cleanSamples = out.clean_samples
    rwg.fuelBurnedKg = float(out.fuel_burned_kg)
    rwg.startMassKg = float(self.start.mass_kg)
    rwg.startMassStdKg = float(self.start.std_kg)
    rwg.startSource = self.start.source
    rwg.knownExtraKg = float(self.known_extra_lb * LB_TO_KG)
    rwg.configuration = self.configuration_text

    rwg.engineRpm = float(num("engine_rpm"))
    rwg.engineTorqueNm = float(num("engine_torque_nm"))
    rwg.gearActual = int(num("gear_actual"))
    rwg.gearDesired = int(num("gear_desired"))
    rwg.converterLocked = bool(out.converter_locked)
    rwg.shiftPending = bool(inp.shift_pending)
    rwg.transInputRpm = float(num("trans_input_rpm"))
    rwg.transOutputRpm = float(num("trans_output_rpm"))
    rwg.serviceBrakeOn = bool(out.service_brake_on)
    rwg.exhaustBrakeOn = bool(inp.exhaust_brake)
    rwg.towHaulOn = bool(_bool(r.get("tow_haul_on", now_nanos)))
    rwg.trailerConnected = bool(inp.trailer_connected)
    rwg.fuelLevelPct = float(num("fuel_level_pct"))
    rwg.availableSignals = self.available_fields(now_nanos)

    rwg.converterLockRule = out.converter_lock_rule
    rwg.gradeSource = out.grade_source
    rwg.massUpdates = out.mass_updates
    rwg.rejectedUpdates = out.rejected_updates
    rwg.qualifyingStops = out.qualifying_stops
    corr = out.mass_drag_correlation
    rwg.massDragCorrelation = float(corr) if math.isfinite(corr) else 0.
    rwg.notCleanReasons = out.not_clean_reasons
    rwg.stabilitySensorOffsetMps2 = float(out.sensor_offset)

  def available_fields(self, now_nanos: int) -> list[str]:
    """availableSignals: the ramWeightGrade fields the shared upload (#9) may write. The estimates are listed once the
    filter has run (the grade once it has a measurement); a decoded truck signal only when it comes from a real
    signal on the bus (converterLocked from the fallback rule is not listed)."""
    fields = self.reader.published_fields(now_nanos)
    out = self.last_output
    if out is not None:
      fields += ESTIMATE_FIELDS
      if out.grade_source != "none":
        fields += GRADE_FIELDS
    return sorted(fields)

  def state(self) -> dict:
    """RamWeightGradeState: the estimate kept between drives, tagged with the configuration in words (no VIN)."""
    s = self.estimator.state()
    s["platform"] = self.CP.carFingerprint
    s["configuration"] = self.configuration_text
    return s
