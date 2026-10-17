"""
Turns the newest messages into one data row of the drive file (contract section 1.2).

Sources:
- carState: speed, acceleration, driver on the brake pedal, factory cruise control engaged and its set speed.
- modelV2: the Comma camera's estimate of the car ahead (the first entry of leadsV3).
- ramWeightGrade: the weight and grade estimator's message. A field is written only when the estimator lists it
  in availableSignals; otherwise the field is left empty.
- Controller area network (CAN) bus messages, decoded with opendbc's CANParser and the Ram Heavy Duty message
  database chrysler_ram_hd_generated:
  - DAS_3 and DAS_4 on bus 2 (the side of the factory windshield unit, which runs the factory adaptive cruise
    control): its engine torque request, brake request, braking amount and following-distance setting.
  - ECM_5 on bus 0 (the truck side): accelerator pedal position.

Nothing about location, video, audio, the vehicle identification number (VIN), the route name or the dongle ID
is read here.
"""
from openpilot.cereal import log
from opendbc.can.parser import CANParser

from openpilot.sunnypilot.ramport.drive_format import COLUMN_NAMES, format_value

RAM_HD_DBC = "chrysler_ram_hd_generated"
BUS_TRUCK = 0     # truck side
BUS_CAMERA = 2    # factory windshield unit side

# A value older than this is treated as not available (written as an empty field)
CAN_STALE_NS = int(1.0e9)

# ramWeightGrade field name (as in openpilot/cereal/custom.capnp) for each column it fills
RAM_WEIGHT_GRADE_FIELDS = {
  "engine_rpm": "engineRpm",
  "engine_torque_nm": "engineTorqueNm",
  "gear_actual": "gearActual",
  "gear_desired": "gearDesired",
  "converter_locked": "converterLocked",
  "shift_pending": "shiftPending",
  "trans_input_rpm": "transInputRpm",
  "trans_output_rpm": "transOutputRpm",
  "service_brake_on": "serviceBrakeOn",
  "exhaust_brake_on": "exhaustBrakeOn",
  "tow_haul_on": "towHaulOn",
  "trailer_connected": "trailerConnected",
  "fuel_level_pct": "fuelLevelPct",
  "grade": "grade",
  "grade_std": "gradeStd",
  "mass_kg": "massKg",
  "mass_std_kg": "massStdKg",
  "drag_n_per_mps2": "dragNPerMps2",
}

# (column, message, signal) read from the controller area network bus
CAN_COLUMNS = [
  ("accel_pedal_pct", "ECM_5", "Accelerator_Position"),
  ("acc_engine_torque_req_nm", "DAS_3", "ENGINE_TORQUE_REQUEST"),
  ("acc_brake_req", "DAS_3", "ACC_DECEL_REQ"),
  ("acc_decel_req_mps2", "DAS_3", "ACC_DECEL"),
  ("acc_distance_setting", "DAS_4", "ACC_DISTANCE_CONFIG_2"),
]


class CanSource:
  """Decodes only the few messages the drive file needs, from the raw 'can' messages."""

  def __init__(self):
    nan = float("nan")  # no expected rate: freshness is checked here with CAN_STALE_NS instead
    self.parsers = {
      BUS_TRUCK: CANParser(RAM_HD_DBC, [("ECM_5", nan)], BUS_TRUCK),
      BUS_CAMERA: CANParser(RAM_HD_DBC, [("DAS_3", nan), ("DAS_4", nan)], BUS_CAMERA),
    }
    self.wanted = {(bus, addr) for bus, parser in self.parsers.items() for addr in parser.addresses}
    self.message_bus = {"ECM_5": BUS_TRUCK, "DAS_3": BUS_CAMERA, "DAS_4": BUS_CAMERA}
    self.last_nanos = 0

  def update_frames(self, nanos: int, frames) -> None:
    """frames: list of (address, data bytes, bus). Only the wanted messages are passed to the parsers."""
    wanted = [(addr, dat, src) for addr, dat, src in frames if (src, addr) in self.wanted]
    for parser in self.parsers.values():
      parser.update([(nanos, wanted)])
    self.last_nanos = max(self.last_nanos, nanos)

  def update_strings(self, can_strings) -> None:
    """can_strings: serialized 'can' messages as received from the messaging socket."""
    for s in can_strings:
      with log.Event.from_bytes(s) as event:
        frames = [(f.address, f.dat, f.src) for f in event.can if (f.src, f.address) in self.wanted]
        self.update_frames(event.logMonoTime, frames)

  def value(self, message: str, signal: str, now_nanos: int | None = None):
    parser = self.parsers[self.message_bus[message]]
    seen = parser.ts_nanos[message][signal]
    now_nanos = self.last_nanos if now_nanos is None else now_nanos
    if seen == 0 or now_nanos - seen > CAN_STALE_NS:
      return None
    return parser.vl[message][signal]


def available_set(rwg) -> set[str]:
  """The ramWeightGrade fields marked available. Both the message field name ("engineRpm") and the drive file
  column name ("engine_rpm") are accepted, so the two programs do not have to agree on one spelling."""
  return {str(s) for s in rwg.availableSignals}


def sample_fields(car_state=None, model=None, rwg=None, can: CanSource | None = None, now_nanos: int | None = None) -> list[str]:
  """One data row without its first column (t_s), as text fields in the contract's column order.

  Each source is a message reader, or None when that message is not available right now (never received,
  too old, or marked not valid); its columns are then left empty.
  """
  values: dict[str, object] = {}

  if car_state is not None:
    values["speed_mps"] = car_state.vEgo
    values["accel_mps2"] = car_state.aEgo
    values["brake_pressed"] = car_state.brakePressed
    values["cruise_engaged"] = car_state.cruiseState.enabled
    values["set_speed_mps"] = car_state.cruiseState.speed

  if model is not None and len(model.leadsV3) > 0:
    lead = model.leadsV3[0]
    values["lead_prob"] = lead.prob
    if len(lead.x) > 0:
      values["lead_dist_m"] = lead.x[0]
    if len(lead.v) > 0:
      values["lead_speed_mps"] = lead.v[0]
    if len(lead.a) > 0:
      values["lead_accel_mps2"] = lead.a[0]

  if rwg is not None:
    available = available_set(rwg)
    for column, field in RAM_WEIGHT_GRADE_FIELDS.items():
      if field in available or column in available:
        values[column] = getattr(rwg, field)

  if can is not None:
    for column, message, signal in CAN_COLUMNS:
      values[column] = can.value(message, signal, now_nanos)

  return [format_value(column, values.get(column)) for column in COLUMN_NAMES[1:]]
