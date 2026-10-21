"""
Adapter between the Ram Heavy Duty truck's controller area network (CAN) bus and the weight and grade estimator.

Every truck signal the estimator reads from the CAN bus is listed once, in RAM_SIGNALS below: the message that
carries it, the signal's name in the message database (the DBC file, which says where each value sits inside a
message), and the bus it is read from. To add a signal once it is verified (edfrahm/SP-RamHD-TICI-Port#5), add it to
the database in opendbc and fix its message and signal name here; nothing else needs to change.

A signal whose message (or signal name) is not in the truck's database is reported unavailable, is never read, and
its ramWeightGrade field is left out of availableSignals. A signal that is in the database but has not been received
recently (for example because the truck does not send that message) is also unavailable at that moment.

Only the messages named here are parsed, with opendbc's CANParser, to keep the processor load low on the comma 3.
"""
from dataclasses import dataclass

from opendbc.can.dbc import DBC
from opendbc.can.parser import CANParser

BUS_TRUCK = 0   # the truck side of the harness
BUS_CAMERA = 2  # the factory windshield unit (camera and cruise computer) side of the harness

# How sure we are of each signal
KNOWN = "known"      # in the Ram Heavy Duty database and read by openpilot or seen in Eric's drives
UNCHECKED = "unchecked"  # in the shared database, but not yet checked on this truck
PENDING = "pending"  # proposed but not in the database until #5 verifies it
MISSING = "missing"  # not found on the bus yet (#5); no message to read


@dataclass(frozen=True)
class SignalSpec:
  message: str | None  # message name in the database, None when the message is not found yet
  signal: str          # signal name inside that message
  bus: int             # which bus the message is read from
  status: str          # KNOWN, UNCHECKED, PENDING or MISSING (see above)
  max_age_s: float = 0.5  # older than this and the value counts as unavailable


# The ONE table of every Ram signal the estimator reads from the CAN bus.
RAM_SIGNALS: dict[str, SignalSpec] = {
  # engine computer (ECM_1, 0x108; ECM_5, 0x22F)
  "engine_rpm": SignalSpec("ECM_1", "ENGINE_RPM", BUS_TRUCK, KNOWN),
  "engine_torque_nm": SignalSpec("ECM_1", "ENGINE_TORQUE", BUS_TRUCK, KNOWN),
  "accel_pedal_pct": SignalSpec("ECM_5", "Accelerator_Position", BUS_TRUCK, KNOWN),

  # wheels and brakes (ESP_6, 0x158: each wheel's speed in revolutions per minute; ESP_8, 0x11C; ESP_1, 0x140)
  "wheel_rpm_fl": SignalSpec("ESP_6", "WHEEL_SPEED_FL", BUS_TRUCK, KNOWN),
  "wheel_rpm_fr": SignalSpec("ESP_6", "WHEEL_SPEED_FR", BUS_TRUCK, KNOWN),
  "wheel_rpm_rl": SignalSpec("ESP_6", "WHEEL_SPEED_RL", BUS_TRUCK, KNOWN),
  "wheel_rpm_rr": SignalSpec("ESP_6", "WHEEL_SPEED_RR", BUS_TRUCK, KNOWN),
  "vehicle_speed_kph": SignalSpec("ESP_8", "Vehicle_Speed", BUS_TRUCK, KNOWN),
  "brake_pressure": SignalSpec("ESP_8", "BRK_PRESSURE", BUS_TRUCK, KNOWN),
  "brake_pedal": SignalSpec("ESP_1", "Brake_Pedal_State", BUS_TRUCK, KNOWN),
  "traction_control_active": SignalSpec("ESP_2", "TCS_ACTIVE", BUS_TRUCK, UNCHECKED),

  # the factory adaptive cruise computer, on the windshield unit side (DAS_3, 0x1F4)
  "acc_active": SignalSpec("DAS_3", "ACC_ACTIVE", BUS_CAMERA, KNOWN),
  "acc_torque_request_nm": SignalSpec("DAS_3", "ENGINE_TORQUE_REQUEST", BUS_CAMERA, KNOWN),

  # transmission computer (TCM_1, 0x144), verified in #5: converter locked when engine speed equals INPUT_SPEED
  "converter_locked": SignalSpec("TCM_1", "TC_LOCKED", BUS_TRUCK, KNOWN),
  "gear_code_actual": SignalSpec("TCM_1", "ACTUAL_GEAR", BUS_TRUCK, KNOWN),    # a code: see TCM_1_GEARS
  "gear_code_desired": SignalSpec("TCM_1", "DESIRED_GEAR", BUS_TRUCK, KNOWN),
  "trans_input_rpm": SignalSpec("TCM_1", "INPUT_SPEED", BUS_TRUCK, KNOWN),
  "trans_output_rpm": SignalSpec("TCM_1", "OUTPUT_SPEED", BUS_TRUCK, KNOWN),
  # no shift-pending signal (never set in 549 shifts in #5): a shift is the desired gear differing from the actual one,
  # or the actual gear changing (see shift_in_progress)

  # stability-control sensor (ESP_4, 0x14C), verified in #5: fore-and-aft acceleration in steps of 0.08 m/s2
  "stability_accel_mps2": SignalSpec("ESP_4", "Acceleration", BUS_TRUCK, KNOWN),

  # the fuel gauge (FUEL_ODO, 0x310), verified in #5: percent of a full tank (raw 0-255), about once per second; the
  # cluster already damps it (it falls steadily while driving and only rises at refueling)
  "fuel_level_pct": SignalSpec("FUEL_ODO", "FUEL_LEVEL", BUS_TRUCK, KNOWN, max_age_s=5.0),

  # not found or not verified yet (#5); hooks only. Candidates seen but not verified, not used:
  # range to empty 0x30E 1|10@0 (likely km), fuel rate 0x2A0 36|13@0 (likely 0.01 L/h per count), odometer 0x310 39|24@0.
  "exhaust_brake_on": SignalSpec(None, "", BUS_TRUCK, MISSING),
  "tow_haul_on": SignalSpec(None, "", BUS_TRUCK, MISSING),
  "trailer_connected": SignalSpec(None, "", BUS_TRUCK, MISSING, max_age_s=2.0),
  # the fuel tank's size in US gallons, if the truck reports it (31, 32, 28, or the optional 50); used for the
  # starting weight and fuel accounting instead of the published-weight table's assumption
  "fuel_tank_gal": SignalSpec(None, "", BUS_TRUCK, MISSING, max_age_s=60.0),
}

# Which ramWeightGrade field each decoded truck signal fills, for availableSignals. A field is listed only when every
# signal it needs is available. (converterLocked filled by the fallback rule is not a decoded signal and is not listed.)
PUBLISHED_FIELDS: dict[str, tuple[str, ...]] = {
  "engineRpm": ("engine_rpm",),
  "engineTorqueNm": ("engine_torque_nm",),
  "gearActual": ("gear_code_actual",),
  "gearDesired": ("gear_code_desired",),
  "converterLocked": ("converter_locked",),
  "shiftPending": ("gear_code_actual", "gear_code_desired"),  # derived from the two gears (no shift-pending signal)
  "transInputRpm": ("trans_input_rpm",),
  "transOutputRpm": ("trans_output_rpm",),
  "exhaustBrakeOn": ("exhaust_brake_on",),
  "towHaulOn": ("tow_haul_on",),
  "trailerConnected": ("trailer_connected",),
  "fuelLevelPct": ("fuel_level_pct",),
  # serviceBrakeOn is filled from either brake signal; it is listed when at least one of them is available
}
SERVICE_BRAKE_SIGNALS = ("brake_pedal", "brake_pressure")

# TCM_1 gear codes -> gear (68RFE: 3.23, 1.84, 1.41, 1.00, 0.82, 0.63; axle 3.73), measured in #5.
# 0 and 11: stopped, park or reverse; anything else unknown. Published as 0.
TCM_1_GEARS = {1: 1, 2: 2, 9: 3, 3: 4, 4: 5, 5: 6}


def gear_from_code(code: float | None) -> int | None:
  """The gear (1-6) for a TCM_1 gear code, 0 for stopped/park/reverse/unknown, None when the signal is unavailable."""
  if code is None:
    return None
  return TCM_1_GEARS.get(int(round(code)), 0)


class RamSignalReader:
  """Decodes the signals of RAM_SIGNALS that exist in the given database, and only those."""

  def __init__(self, dbc_name: str, signals: dict[str, SignalSpec] | None = None):
    signals = RAM_SIGNALS if signals is None else signals
    dbc = DBC(dbc_name)

    self.specs: dict[str, SignalSpec] = {}
    self.not_in_database: list[str] = []
    for name, spec in signals.items():
      msg = dbc.name_to_msg.get(spec.message) if spec.message else None
      if msg is not None and spec.signal in msg.sigs:
        self.specs[name] = spec
      else:
        self.not_in_database.append(name)

    # one parser per bus, each with only the messages it needs
    messages_by_bus: dict[int, set[str]] = {}
    for spec in self.specs.values():
      messages_by_bus.setdefault(spec.bus, set()).add(spec.message)
    self.parsers = {bus: CANParser(dbc_name, [(m, 0) for m in sorted(msgs)], bus) for bus, msgs in messages_by_bus.items()}

    # fast filter: message address -> buses we read it from
    self.wanted: dict[int, frozenset[int]] = {}
    for bus, msgs in messages_by_bus.items():
      for m in msgs:
        address = dbc.name_to_msg[m].address
        self.wanted[address] = self.wanted.get(address, frozenset()) | {bus}

    self.last_nanos = 0

  def in_database(self, name: str) -> bool:
    return name in self.specs

  def update(self, can_list: list[tuple[int, list[tuple[int, bytes, int]]]]) -> None:
    """can_list: [(nanoseconds, [(address, data, bus), ...]), ...], already filtered or not."""
    if not can_list:
      return
    for parser in self.parsers.values():
      parser.update(can_list)
    self.last_nanos = max(self.last_nanos, can_list[-1][0])

  def update_from_events(self, events) -> None:
    """events: log Event readers carrying `can`. Reads data only from the messages we need (cheap on the comma 3)."""
    wanted = self.wanted
    can_list = []
    for evt in events:
      frames = []
      for f in evt.can:
        address = f.address
        buses = wanted.get(address)
        if buses is not None:
          src = f.src
          if src in buses:
            frames.append((address, f.dat, src))
      can_list.append((evt.logMonoTime, frames))
    self.update(can_list)

  def get(self, name: str, now_nanos: int | None = None) -> float | None:
    """The latest value of a signal, or None when it is not in the database or not received recently."""
    spec = self.specs.get(name)
    if spec is None:
      return None
    parser = self.parsers[spec.bus]
    ts = parser.ts_nanos[spec.message][spec.signal]
    if ts == 0:
      return None
    now = self.last_nanos if now_nanos is None else now_nanos
    if (now - ts) * 1e-9 > spec.max_age_s:
      return None
    return parser.vl[spec.message][spec.signal]

  def available(self, now_nanos: int | None = None) -> set[str]:
    return {name for name in self.specs if self.get(name, now_nanos) is not None}

  def published_fields(self, now_nanos: int | None = None) -> list[str]:
    avail = self.available(now_nanos)
    fields = [field for field, needs in PUBLISHED_FIELDS.items() if all(n in avail for n in needs)]
    if any(n in avail for n in SERVICE_BRAKE_SIGNALS):
      fields.append("serviceBrakeOn")
    return sorted(fields)
