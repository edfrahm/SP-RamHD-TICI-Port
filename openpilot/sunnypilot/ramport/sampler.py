"""
Turns the newest messages into one data row of the drive file (contract section 1.2).

Sources:
- carState: speed, acceleration, driver on the brake pedal, factory cruise control engaged and its set speed.
- modelV2: the Comma camera's estimate of the car ahead (the first entry of leadsV3).
- ramWeightGrade: the weight estimator's message, for the weight and air drag. A field is written only when the
  estimator lists it in availableSignals; otherwise the field is left empty. The estimator publishes only when its
  estimate changes, so its truck signals are not used here, and it no longer publishes a grade (grade columns empty).
- Controller area network (CAN) bus messages, decoded with opendbc's CANParser and the Ram Heavy Duty message
  database chrysler_ram_hd_generated:
  - DAS_3 and DAS_4 on bus 2 (the side of the factory windshield unit, which runs the factory adaptive cruise
    control): its engine torque request, brake request, braking amount and following-distance setting.
  - ECM_5 on bus 0 (the truck side): accelerator pedal position.
  - The engine, transmission, brake and fuel gauge signals, decoded exactly as the weight estimator decodes them
    (ramweightd/ram_signals.py): ECM_1, TCM_1, ESP_1, ESP_8 and FUEL_ODO on bus 0.

Nothing about location, video, audio, the vehicle identification number (VIN), the route name or the dongle ID
is read here.
"""
from openpilot.cereal import log
from opendbc.can.parser import CANParser

from openpilot.sunnypilot.ramport.drive_format import COLUMN_NAMES, format_value
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import BRAKE_PRESSURE_ON
from openpilot.sunnypilot.selfdrive.ramweightd.ram_signals import RAM_SIGNALS, RamSignalReader, gear_from_code

RAM_HD_DBC = "chrysler_ram_hd_generated"
BUS_TRUCK = 0     # truck side
BUS_CAMERA = 2    # factory windshield unit side

# A value older than this is treated as not available (written as an empty field)
CAN_STALE_NS = int(1.0e9)

# ramWeightGrade field name (as in openpilot/cereal/custom.capnp) for each column it fills
RAM_WEIGHT_GRADE_FIELDS = {
  "mass_kg": "massKg",
  "mass_std_kg": "massStdKg",
  "drag_n_per_mps2": "dragNPerMps2",
}

# the weight estimator's signals (ram_signals.RAM_SIGNALS) this file reads from the bus itself, and the column each fills
TRUCK_SIGNAL_COLUMNS = {
  "engine_rpm": "engine_rpm",
  "engine_torque_nm": "engine_torque_nm",
  "trans_input_rpm": "trans_input_rpm",
  "trans_output_rpm": "trans_output_rpm",
  "converter_locked": "converter_locked",
  "fuel_level_pct": "fuel_level_pct",
}
TRUCK_SIGNALS = {name: RAM_SIGNALS[name] for name in
                 list(TRUCK_SIGNAL_COLUMNS) + ["gear_code_actual", "gear_code_desired", "brake_pedal", "brake_pressure"]}

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
    self.truck = RamSignalReader(RAM_HD_DBC, TRUCK_SIGNALS)
    self.wanted |= {(bus, addr) for addr, buses in self.truck.wanted.items() for bus in buses}
    self.last_nanos = 0

  def update_frames(self, nanos: int, frames) -> None:
    """frames: list of (address, data bytes, bus). Only the wanted messages are passed to the parsers."""
    wanted = [(addr, dat, src) for addr, dat, src in frames if (src, addr) in self.wanted]
    for parser in self.parsers.values():
      parser.update([(nanos, wanted)])
    self.truck.update([(nanos, wanted)])
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

  def truck_values(self, now_nanos: int | None = None) -> dict[str, object]:
    """The engine, transmission, brake and fuel columns; a column whose signal is not current is left out."""
    now_nanos = self.last_nanos if now_nanos is None else now_nanos
    get = self.truck.get
    values: dict[str, object] = {}
    for name, column in TRUCK_SIGNAL_COLUMNS.items():
      x = get(name, now_nanos)
      if x is not None:
        values[column] = bool(x > 0.5) if column == "converter_locked" else x
    actual, desired = gear_from_code(get("gear_code_actual", now_nanos)), gear_from_code(get("gear_code_desired", now_nanos))
    if actual is not None:
      values["gear_actual"] = actual
    if desired is not None:
      values["gear_desired"] = desired
    if actual is not None and desired is not None:
      values["shift_pending"] = actual != desired
    pedal, pressure = get("brake_pedal", now_nanos), get("brake_pressure", now_nanos)
    if pedal is not None or pressure is not None:
      values["service_brake_on"] = bool(pedal is not None and pedal > 0.5) or (pressure is not None and pressure > BRAKE_PRESSURE_ON)
    return values


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
    values.update(can.truck_values(now_nanos))

  return [format_value(column, values.get(column)) for column in COLUMN_NAMES[1:]]
