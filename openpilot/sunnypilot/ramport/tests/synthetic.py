"""
Synthetic (made-up) message streams for the ramportd tests. Nothing here comes from a real drive.

Each sample i carries its own number in the speed (speed_mps = i / 1000), so a test can tell exactly which samples
reached the file.
"""
import openpilot.cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from opendbc.car.structs import car

from openpilot.sunnypilot.ramport.sampler import RAM_HD_DBC, BUS_CAMERA, BUS_TRUCK

SAMPLE_DT = 0.1
START_MONO = 5000.0
START_WALL = 1_792_000_000.0  # 2026-10-14 17:46:40 UTC, any fixed time works
FAKE_VIN = "1C6SRFFT5NN123456"
FAKE_DONGLE_ID = "0123456789abcdef"
FAKE_ROUTE = "00000776--9a65ca67cd"
ENGINE_PART = "52370231AF"


def car_params_bytes(fingerprint: str = "RAM_HD_5TH_GEN", with_transmission: bool = False) -> bytes:
  CP = car.CarParams.new_message()
  CP.carFingerprint = fingerprint
  CP.carVin = FAKE_VIN
  fws = CP.init("carFw", 3 if with_transmission else 2)
  fws[0].ecu = "eps"
  fws[0].fwVersion = b"68507906AB"
  fws[0].address = 0x761
  fws[1].ecu = "engine"
  fws[1].fwVersion = (ENGINE_PART + " ").encode()
  fws[1].address = 0x7e0
  if with_transmission:
    fws[2].ecu = "transmission"
    fws[2].fwVersion = b"68376876AC"
    fws[2].address = 0x7e1
  return CP.to_bytes()


def car_state(i: int, engaged: bool = True):
  msg = messaging.new_message("carState")
  cs = msg.carState
  cs.vEgo = i / 1000.0
  cs.aEgo = 0.25
  cs.brakePressed = (i % 50) == 0
  cs.cruiseState.enabled = engaged
  cs.cruiseState.speed = 26.8224
  return cs


def model(i: int, prob: float = 0.8):
  msg = messaging.new_message("modelV2")
  leads = msg.modelV2.init("leadsV3", 3)
  leads[0].prob = prob
  leads[0].x = [40.0 + (i % 10), 41.0]
  leads[0].v = [24.5, 24.6]
  leads[0].a = [-0.3, -0.2]
  return msg.modelV2


def ram_weight_grade(available: list[str]):
  msg = messaging.new_message("ramWeightGrade")
  r = msg.ramWeightGrade
  r.massKg = 3810.0
  r.massStdKg = 140.0
  r.dragNPerMps2 = 0.71
  r.grade = 0.031
  r.gradeStd = 0.004
  r.engineRpm = 1450.0
  r.engineTorqueNm = 812.5
  r.gearActual = 6
  r.gearDesired = 6
  r.converterLocked = True
  r.shiftPending = False
  r.transInputRpm = 1450.0
  r.transOutputRpm = 1100.0
  r.serviceBrakeOn = False
  r.exhaustBrakeOn = True
  r.towHaulOn = True
  r.trailerConnected = False
  r.fuelLevelPct = 76.4
  r.availableSignals = available
  return r


class CanStream:
  """Packs DAS_3 and DAS_4 (bus 2) and ECM_5 (bus 0) with valid counters and checksums, plus unrelated traffic, and
  with `truck` the engine, transmission, brake and fuel gauge messages (bus 0)."""

  def __init__(self, truck: bool = True):
    self.packer = CANPacker(RAM_HD_DBC)
    self.truck = truck

  def frames(self, include_das4: bool = True) -> list[tuple[int, bytes, int]]:
    frames = [
      self.packer.make_can_msg("ECM_5", BUS_TRUCK, {"Accelerator_Position": 22.4}),
      self.packer.make_can_msg("DAS_3", BUS_CAMERA, {"ENGINE_TORQUE_REQUEST": 640.25, "ACC_DECEL_REQ": 0, "ACC_DECEL": 4.0,
                                                      "ACC_ACTIVE": 1, "ACC_AVAILABLE": 1}),
      (0x123, b"\x01\x02\x03\x04", BUS_TRUCK),  # unrelated message, must be ignored
      (0x1F4, b"\x00" * 8, BUS_TRUCK),          # DAS_3's address on the wrong bus, must be ignored
    ]
    if include_das4:
      frames.append(self.packer.make_can_msg("DAS_4", BUS_CAMERA, {"ACC_DISTANCE_CONFIG_2": 2, "ACC_SET_SPEED_KPH": 97}))
    if self.truck:
      frames += [
        self.packer.make_can_msg("ECM_1", BUS_TRUCK, {"ENGINE_RPM": 1450., "ENGINE_TORQUE": 812.5}),
        # TCM_1 gear code 5 is sixth gear (ramweightd ram_signals.TCM_1_GEARS)
        self.packer.make_can_msg("TCM_1", BUS_TRUCK, {"TC_LOCKED": 1, "ACTUAL_GEAR": 5, "DESIRED_GEAR": 5, "INPUT_SPEED": 1450.,
                                                      "OUTPUT_SPEED": 1100.}),
        self.packer.make_can_msg("ESP_1", BUS_TRUCK, {"Brake_Pedal_State": 0}),
        self.packer.make_can_msg("ESP_8", BUS_TRUCK, {"BRK_PRESSURE": 0.}),
        self.packer.make_can_msg("FUEL_ODO", BUS_TRUCK, {"FUEL_LEVEL": 76.4}),
      ]
    return frames

  def message_bytes(self, nanos: int, include_das4: bool = True) -> bytes:
    frames = self.frames(include_das4)
    msg = messaging.new_message("can", len(frames))
    msg.logMonoTime = nanos
    for f, (addr, dat, bus) in zip(msg.can, frames, strict=True):
      f.address = addr
      f.dat = dat
      f.src = bus
    return msg.to_bytes()


def drive(daemon, seconds: float, available=("engineRpm", "massKg", "grade"), start_index: int = 0, with_model: bool = True,
          das4_until_s: float | None = None, started_after: bool = False, truck: bool = True):
  """Feed a synthetic drive of `seconds` to a RamPortDaemon at 10 samples per second, then end it."""
  can = CanStream(truck)
  rwg = ram_weight_grade(list(available)) if available is not None else None
  n = int(round(seconds / SAMPLE_DT))
  for k in range(n):
    i = start_index + k
    mono = START_MONO + i * SAMPLE_DT
    nanos = int(round(mono * 1e9))
    include_das4 = das4_until_s is None or k * SAMPLE_DT < das4_until_s
    daemon.update_can([can.message_bytes(nanos, include_das4)])
    daemon.step(True, mono, START_WALL + i * SAMPLE_DT, car_state(i), model(i) if with_model else None, rwg, nanos)
  if not started_after:
    end_mono = START_MONO + (start_index + n) * SAMPLE_DT
    daemon.step(False, end_mono, START_WALL + (start_index + n) * SAMPLE_DT)
