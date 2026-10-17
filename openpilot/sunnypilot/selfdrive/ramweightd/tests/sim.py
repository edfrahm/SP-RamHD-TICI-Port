"""
SYNTHETIC test data: a simple physics simulation of a heavy-duty pickup, for the estimator's tests. Nothing here comes
from a real drive.

The truck has a known mass, air-drag factor, rolling resistance and driveline efficiency, a six-speed automatic with
a torque converter that locks in third gear and above (and slips by a varying amount when unlocked), and drives a
schedule of speed changes, cruising, grades and stops. The sensors get realistic noise, quantization (the signal
steps of the real messages) and delays. The simulation runs at 100 Hz; the estimator gets 10 Hz samples, and
write_rlog writes the same drive as an openpilot log (initData, carParams, carState, can, deviceMotion,
extrinsicsCalibration) for the replay tool.
"""
import math
import random
from dataclasses import dataclass, field

import numpy as np

from openpilot.sunnypilot.selfdrive.ramweightd.estimator import G, Inputs, LB_TO_KG

DT = 0.01
GEAR_RATIOS = (3.23, 1.84, 1.41, 1.00, 0.82, 0.63)  # 68RFE
TCM_1_CODES = {1: 1, 2: 2, 3: 9, 4: 3, 5: 4, 6: 5}  # gear -> TCM_1 ACTUAL_GEAR / DESIRED_GEAR code (measured in #5)
UPSHIFT_MPS = (4.5, 8.5, 13., 18., 23.)
DOWNSHIFT_MPS = (3.5, 7.5, 12., 17., 21.5)
T_MAX_NM = 1300.
IDLE_RPM = 700.

@dataclass
class Truck:
  mass_kg: float = 3900.
  drag: float = 1.05
  rolling_resistance: float = 0.009
  efficiency: float = 0.85
  engine_inertia: float = 1.5
  rotating_mass_kg: float = 120.
  tire_radius_m: float = 0.42
  axle_ratio: float = 3.73
  tank_gal: float = 32.
  lb_per_gal: float = 7.1
  fuel_pct: float = 80.
  converter_loss: float = 0.15   # extra power lost while the converter slips
  body_pitch_per_accel: float = 0.003  # the body squats (nose up) when speeding up, radians per m/s2


@dataclass
class Sensors:
  stability_sensor: bool = True    # ESP_4 frames
  transmission: bool = True        # TCM_1 frames (converter lock, gears)
  comma_accelerometer: bool = True  # the Comma's accelerometer events
  fuel_level: bool = False
  sensor_offset: float = -0.02     # ESP_4 offset, m/s2 (about -0.002 in grade on Eric's drives)
  comma_offset: float = 0.17       # the Comma's accelerometer offset, m/s2 (0.13-0.19 on Eric's drives)
  comma_tilt: float = 0.0038       # the Comma's extra tilt per m/s2 of acceleration (cab on its mounts)
  pitch_bias: float = 0.0          # left-over error of the Comma's pitch, radians
  pitch_noise: float = 0.002
  pitch_slow_error: float = 0.0    # slowly wandering error of the Comma's pitch (standard deviation, radians) ...
  pitch_slow_tau_s: float = 60.    # ... and how long it lasts
  mount_rpy: tuple = (0.0, 0.03, 0.015)  # the Comma's mounting angles (extrinsicsCalibration)


@dataclass
class Phase:
  kind: str                 # "drive" or "stop"
  duration: float
  speed: float = 0.         # drive: target speed
  accel: float = 1.0        # drive: largest acceleration toward the target
  cruise: bool = False      # drive: the factory cruise holds the speed (no pedal)
  park: bool = False        # stop: shifter in park during the stop
  door: bool = False        # stop: a door opens during the stop
  add_mass_kg: float = 0.   # stop: load added at the middle of the stop (a trailer, cargo)
  add_drag: float = 0.      # stop: drag added (a trailer)
  refuel_pct: float = 0.    # stop: fuel added


def hilly(x: float) -> float:
  """Road grade (rise over run) at distance x meters: rolling hills with long climbs and descents."""
  return 0.035 * math.sin(2 * math.pi * x / 2600.) + 0.015 * math.sin(2 * math.pi * x / 900. + 1.)


def flat(x: float) -> float:
  return 0.


def varied_schedule(cycles: int = 3) -> list[Phase]:
  """Strong and gentle accelerations, different speeds, cruising, on whatever road the grade function gives."""
  cycle = [
    Phase("drive", 35, speed=14., accel=1.5),
    Phase("drive", 40, speed=24., accel=0.8),
    Phase("drive", 70, speed=24., cruise=True),
    Phase("drive", 30, speed=17., accel=0.6),
    Phase("drive", 40, speed=29., accel=1.2),
    Phase("drive", 90, speed=29., cruise=True),
    Phase("drive", 30, speed=20., accel=0.8),
    Phase("drive", 30, speed=26., accel=0.5),
    Phase("drive", 60, speed=26., cruise=True),
  ]
  return [Phase("stop", 5)] + cycle * cycles


@dataclass
class SimLog:
  """Everything at 100 Hz."""
  t: list = field(default_factory=list)
  v: list = field(default_factory=list)
  a: list = field(default_factory=list)
  x: list = field(default_factory=list)
  sine: list = field(default_factory=list)
  rpm: list = field(default_factory=list)
  torque: list = field(default_factory=list)
  gear: list = field(default_factory=list)
  gear_desired: list = field(default_factory=list)
  locked: list = field(default_factory=list)
  shifting: list = field(default_factory=list)
  pedal: list = field(default_factory=list)
  acc_active: list = field(default_factory=list)
  acc_torque: list = field(default_factory=list)
  brake_pedal: list = field(default_factory=list)
  brake_pressure: list = field(default_factory=list)
  shifter: list = field(default_factory=list)
  door: list = field(default_factory=list)
  body_pitch: list = field(default_factory=list)
  fuel_pct: list = field(default_factory=list)
  mass_kg: list = field(default_factory=list)   # true combined mass, including fuel
  drag: list = field(default_factory=list)


class TruckSim:
  def __init__(self, truck: Truck | None = None, sensors: Sensors | None = None, grade_fn=hilly, seed: int = 0):
    self.truck = truck or Truck()
    self.sensors = sensors or Sensors()
    self.grade_fn = grade_fn
    self.rng = random.Random(seed)
    self.log = SimLog()
    self.v = 0.
    self.x = 0.
    self.t = 0.
    self.gear = 1
    self.last_shift_t = -10.
    self.prev_gear = 1
    self.omega_prev = IDLE_RPM * 2 * math.pi / 60
    self.slip_noise = 0.
    tk = self.truck
    self.fuel_kg = tk.fuel_pct / 100. * tk.tank_gal * tk.lb_per_gal * LB_TO_KG
    self.dry_mass = tk.mass_kg - self.fuel_kg
    self.drag = tk.drag

  @property
  def mass(self) -> float:
    return self.dry_mass + self.fuel_kg

  def start_moving(self, v: float) -> "TruckSim":
    """Start the drive already at speed v, in the gear the shift schedule would use, converter locked."""
    self.v = v
    self.gear = 1 + sum(v > u for u in UPSHIFT_MPS)
    self.last_shift_t = -10.
    self.omega_prev = v * GEAR_RATIOS[self.gear - 1] * self.truck.axle_ratio / self.truck.tire_radius_m
    return self

  def _gear_update(self) -> None:
    g = self.gear
    if g < 6 and self.v > UPSHIFT_MPS[g - 1]:
      self.gear += 1
    elif g > 1 and self.v < DOWNSHIFT_MPS[g - 2]:
      self.gear -= 1
    if self.gear != g:
      self.prev_gear = g
      self.last_shift_t = self.t

  def _step(self, a_cmd: float, cruise: bool, shifter: str, door: bool) -> None:
    tk = self.truck
    theta = math.atan(self.grade_fn(self.x))
    s, c = math.sin(theta), math.cos(theta)
    m = self.mass
    self._gear_update()
    shifting = self.t - self.last_shift_t < 0.6
    locked = self.gear >= 3 and self.t - self.last_shift_t > 1.2 and self.v > 8.
    ratio = GEAR_RATIOS[self.gear - 1]
    if shifting:
      f = (self.t - self.last_shift_t) / 0.6
      ratio = GEAR_RATIOS[self.prev_gear - 1] * (1 - f) + ratio * f
    omega_turbine = self.v * ratio * tk.axle_ratio / tk.tire_radius_m

    resist = m * G * (s + tk.rolling_resistance * c) + self.drag * self.v ** 2
    f_req = (m + tk.rotating_mass_kg) * a_cmd + resist
    load = min(max(f_req * max(self.v, 1.) / 250e3, 0.), 1.)
    if locked:
      omega = omega_turbine
    else:
      self.slip_noise += self.rng.gauss(0, 0.002) - 0.05 * self.slip_noise * DT * 10
      omega = max(omega_turbine * (1.04 + 0.08 * load + self.slip_noise), IDLE_RPM * 2 * math.pi / 60)
    omega_dot = (omega - self.omega_prev) / DT
    self.omega_prev = omega
    eff = tk.efficiency * (1. if locked else 1. - tk.converter_loss)

    brake_force = 0.
    if self.v <= 0.01 and a_cmd <= 0.:
      # standing still: the brakes hold the truck, the engine idles
      torque, wheel_force, a = 20., 0., 0.
      brake_force = -abs(resist) - 2000.
      self.v = 0.
    elif f_req > 0.:
      v_eff = max(self.v, 0.5)
      torque = (f_req * v_eff / eff + tk.engine_inertia * omega * omega_dot) / omega
      torque = min(max(torque, 0.), T_MAX_NM)
      wheel_force = eff * (torque * omega - tk.engine_inertia * omega * omega_dot) / v_eff
      a = (wheel_force - resist) / (m + tk.rotating_mass_kg)
    else:
      # pedal released: the engine drags, the brakes do the rest
      torque = -40.
      wheel_force = eff * torque * omega / max(self.v, 0.5) if self.v > 0.5 else 0.
      brake_force = min(f_req - wheel_force, 0.)
      a = (wheel_force + brake_force - resist) / (m + tk.rotating_mass_kg)
    self.v = max(self.v + a * DT, 0.)
    self.x += self.v * DT
    # fuel: 220 grams per kilowatt hour of engine power, plus idle
    p_kw = max(torque * omega, 0.) / 1000.
    self.fuel_kg = max(self.fuel_kg - (0.220 * p_kw / 3600. + 0.0003) * DT, 0.)

    L = self.log
    L.t.append(self.t)
    L.v.append(self.v)
    L.a.append(a)
    L.x.append(self.x)
    L.sine.append(s)
    L.rpm.append(omega * 60 / (2 * math.pi))
    L.torque.append(torque)
    in_shift = self.t - self.last_shift_t < 0.3
    L.gear.append(self.prev_gear if in_shift else self.gear)
    L.gear_desired.append(self.gear)
    L.locked.append(locked)
    L.shifting.append(shifting)
    L.pedal.append(0. if (cruise or torque <= 0) else 100. * torque / T_MAX_NM)
    L.acc_active.append(cruise)
    L.acc_torque.append(torque if cruise else 0.)
    L.brake_pedal.append(brake_force < -300. and not cruise)
    L.brake_pressure.append(min(-brake_force / 20., 4000.) if brake_force < -300. else 0.)
    L.shifter.append(shifter)
    L.door.append(door)
    L.body_pitch.append(tk.body_pitch_per_accel * a)
    L.fuel_pct.append(100. * self.fuel_kg / (tk.tank_gal * tk.lb_per_gal * LB_TO_KG))
    L.mass_kg.append(self.mass)
    L.drag.append(self.drag)
    self.t += DT

  def run(self, phases: list[Phase]) -> "TruckSim":
    for ph in phases:
      if ph.kind == "drive":
        for _ in range(int(round(ph.duration / DT))):
          err = ph.speed - self.v
          a_cmd = min(max(0.8 * err, -1.0), ph.accel)
          if ph.cruise:
            a_cmd = min(max(0.3 * err, -0.3), 0.3)
          self._step(a_cmd, ph.cruise, "drive", False)
      else:
        while self.v > 0.:
          self._step(-1.5, False, "drive", False)
        n = int(round(ph.duration / DT))
        for i in range(n):
          if i == n // 2:
            self.dry_mass += ph.add_mass_kg
            self.drag += ph.add_drag
            tk = self.truck
            self.fuel_kg = min(self.fuel_kg + ph.refuel_pct / 100. * tk.tank_gal * tk.lb_per_gal * LB_TO_KG,
                               tk.tank_gal * tk.lb_per_gal * LB_TO_KG)
          in_park = ph.park and n // 10 < i < n - n // 10
          door = ph.door and n // 4 < i < n // 2
          self._step(0., False, "park" if in_park else "drive", door)
    return self

  # ---- sensors ----
  def _measured(self, i: int, delay_steps: int, values: list) -> float:
    return values[max(i - delay_steps, 0)]

  def measure(self, i: int, rng: random.Random) -> dict:
    """Sensor readings at log index i, with noise, quantization and delays."""
    L, se, tk = self.log, self.sensors, self.truck
    v = L.v[i]
    q = 0.0078125 / 3.6
    v_raw = round((v + rng.gauss(0, 0.01)) / q) * q if v > 0 else 0.
    rpm = round(self._measured(i, 4, L.rpm) + rng.gauss(0, 2))
    torque = round((self._measured(i, 4, L.torque) * (1 + rng.gauss(0, 0.01)) + rng.gauss(0, 3)) / 0.25) * 0.25
    wheel_rpm = round(v / (2 * math.pi * tk.tire_radius_m) * 60 / 0.5) * 0.5
    j = max(i - 3, 0)
    true_sensor = L.a[j] + G * math.sin(math.asin(L.sine[j]) + L.body_pitch[j]) + se.sensor_offset
    sensor = round((true_sensor + rng.gauss(0, 0.05)) / 0.08) * 0.08
    k = max(i - 5, 0)
    pitch = math.asin(L.sine[k]) + L.body_pitch[k] + se.pitch_bias + self._pitch_slow(k) + rng.gauss(0, se.pitch_noise)
    true_comma = L.a[j] + G * math.sin(math.asin(L.sine[j]) + L.body_pitch[j] + se.comma_tilt * L.a[j]) + se.comma_offset
    comma = true_comma + rng.gauss(0, 0.03)
    slosh = 30. * L.a[i] / G + 40. * L.sine[i] + rng.gauss(0, 0.4)
    fuel = round((L.fuel_pct[i] + slosh) / 0.5) * 0.5
    return {"v_raw": v_raw, "rpm": max(rpm, 0.), "torque": torque, "wheel_rpm": wheel_rpm, "sensor": sensor, "pitch": pitch,
            "comma": comma, "fuel": min(max(fuel, 0.), 100.)}

  def _pitch_slow(self, i: int) -> float:
    """A slowly wandering pitch error (first-order random process), the same for every reading of the log."""
    se = self.sensors
    if se.pitch_slow_error <= 0.:
      return 0.
    if getattr(self, "_slow", None) is None or len(self._slow) != len(self.log.t):
      rng = np.random.default_rng(12345)
      k = DT / se.pitch_slow_tau_s
      e = np.zeros(len(self.log.t))
      w = rng.normal(0., se.pitch_slow_error * math.sqrt(2 * k), len(e))
      for n in range(1, len(e)):
        e[n] = e[n - 1] * (1 - k) + w[n]
      self._slow = e
    return float(self._slow[i])

  def inputs(self, every: int = 10, seed: int = 1) -> list[Inputs]:
    """10 Hz estimator samples."""
    rng = random.Random(seed)
    L, se = self.log, self.sensors
    out = []
    for i in range(0, len(L.t), every):
      m = self.measure(i, rng)
      out.append(Inputs(
        t=L.t[i], v=m["v_raw"], gear=L.shifter[i], door_open=L.door[i],
        brake_pedal=L.brake_pedal[i], brake_pressure=L.brake_pressure[i],
        accel_pedal_pct=L.pedal[i], acc_active=L.acc_active[i], acc_torque_request_nm=L.acc_torque[i],
        engine_rpm=m["rpm"], engine_torque_nm=m["torque"], rear_wheel_rpm=m["wheel_rpm"],
        traction_control=False,
        stability_accel=m["sensor"] if se.stability_sensor else None,
        comma_accel=m["comma"] if se.comma_accelerometer else None,
        converter_locked=L.locked[i] if se.transmission else None,
        shift_pending=(L.gear[i] != L.gear_desired[i] or L.shifting[i]) if se.transmission else None,
        fuel_level_pct=m["fuel"] if se.fuel_level else None,
        pitch=m["pitch"], pitch_std=0.003, lateral_accel=0.,
      ))
    return out

  def true_mass_lb(self, i: int = -1) -> float:
    return self.log.mass_kg[i] / LB_TO_KG

  # ---- openpilot log ----
  def write_rlog(self, path: str, dbc_name: str = "chrysler_ram_hd_generated", vin: str = "", car_fw: list | None = None,
                 can_hz: int = 50, seed: int = 2) -> None:
    """Write the drive as an uncompressed openpilot rlog (capnp Event messages serialized with to_bytes)."""
    from opendbc.can.packer import CANPacker
    from openpilot.cereal import log, messaging
    from openpilot.common.transformations.orientation import rot_from_euler, euler_from_rot

    rng = random.Random(seed)
    packer = CANPacker(dbc_name)
    L, se = self.log, self.sensors
    t0 = 1_000_000_000
    device_from_calib = rot_from_euler(list(se.mount_rpy))

    def event(which: str, t: float, size: int | None = None):
      msg = messaging.new_message(which, size)
      msg.logMonoTime = int(t0 + t * 1e9)
      msg.valid = True
      return msg

    events = []
    init = event("initData", 0.)
    init.initData.deviceType = "tici"
    events.append(init)
    cp = event("carParams", 0.)
    cp.carParams.brand = "chrysler"
    cp.carParams.carFingerprint = "RAM_HD_5TH_GEN"
    cp.carParams.mass = 3541.
    cp.carParams.carVin = vin
    fws = car_fw or []
    cp.carParams.init("carFw", len(fws))
    for k, (ecu, version) in enumerate(fws):
      cp.carParams.carFw[k].ecu = ecu
      cp.carParams.carFw[k].fwVersion = version
    events.append(cp)

    step_can = int(round(1 / (can_hz * DT)))
    for i in range(0, len(L.t), step_can):
      t = L.t[i]
      m = self.measure(i, rng)
      cs = event("carState", t)
      c = cs.carState
      c.vEgoRaw = m["v_raw"]
      c.vEgo = L.v[i]
      c.aEgo = L.a[i]
      c.standstill = L.v[i] < 0.01
      c.gearShifter = L.shifter[i]
      c.doorOpen = L.door[i]
      c.brakePressed = L.brake_pedal[i]
      c.gasPressed = L.pedal[i] > 0
      c.cruiseState.enabled = L.acc_active[i]
      events.append(cs)

      frames = [
        packer.make_can_msg("ECM_1", 0, {"ENGINE_RPM": m["rpm"], "ENGINE_TORQUE": m["torque"]}),
        packer.make_can_msg("ESP_6", 0, {f"WHEEL_SPEED_{w}": m["wheel_rpm"] for w in ("FL", "FR", "RL", "RR")}),
        packer.make_can_msg("ESP_8", 0, {"Vehicle_Speed": m["v_raw"] * 3.6, "BRK_PRESSURE": L.brake_pressure[i]}),
        packer.make_can_msg("ESP_1", 0, {"Brake_Pedal_State": 1 if L.brake_pedal[i] else 0}),
        packer.make_can_msg("ECM_5", 0, {"Accelerator_Position": L.pedal[i]}),
        packer.make_can_msg("DAS_3", 2, {"ACC_ACTIVE": 1 if L.acc_active[i] else 0, "ENGINE_TORQUE_REQUEST": L.acc_torque[i]}),
        (0x7FF, bytes(8), 0),   # an unrelated message the reader must skip
        (0x108, bytes(8), 1),   # ECM_1's address on a bus we do not read
      ]
      if se.transmission:
        frames.append(packer.make_can_msg("TCM_1", 0, {"TC_LOCKED": 1 if L.locked[i] else 0, "ACTUAL_GEAR": TCM_1_CODES[L.gear[i]],
                                                       "DESIRED_GEAR": TCM_1_CODES[L.gear_desired[i]], "INPUT_SPEED": L.rpm[i],
                                                       "OUTPUT_SPEED": 0}))
      if se.stability_sensor:
        frames.append(packer.make_can_msg("ESP_4", 0, {"Acceleration": m["sensor"]}))
      ce = event("can", t + 0.001, len(frames))
      for k, (addr, dat, bus) in enumerate(frames):
        ce.can[k].address = addr
        ce.can[k].dat = dat
        ce.can[k].src = bus
      events.append(ce)

      if se.comma_accelerometer:
        # the Comma's accelerometer: specific force in the truck frame -> device frame -> the sensor's own axes
        th = math.asin(L.sine[i]) + L.body_pitch[i] + se.comma_tilt * L.a[i]
        f_calib = np.array([m["comma"], 0., -G * math.cos(th)])
        f_dev = device_from_calib @ f_calib
        ae = event("accelerometer", t + 0.0015)
        ae.accelerometer.init("acceleration")
        ae.accelerometer.acceleration.v = [float(-f_dev[2]), float(-f_dev[1]), float(-f_dev[0])]
        events.append(ae)

      if i % 5 == 0:  # 20 Hz
        dm = event("deviceMotion", t + 0.002)
        ned_from_calib = rot_from_euler([0., m["pitch"], 0.3])
        rpy = euler_from_rot(ned_from_calib @ device_from_calib.T)
        o = dm.deviceMotion.orientationNED
        o.x, o.y, o.z = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
        o.xStd = o.yStd = o.zStd = 0.003
        o.valid = True
        for meas in (dm.deviceMotion.angularVelocityDevice, dm.deviceMotion.velocityDevice, dm.deviceMotion.accelerationDevice):
          meas.valid = True
          meas.xStd = meas.yStd = meas.zStd = 0.01
        events.append(dm)
      if i % 25 == 0:  # 4 Hz
        ec = event("extrinsicsCalibration", t + 0.003)
        ec.extrinsicsCalibration.rpyCalib = [float(x) for x in se.mount_rpy]
        ec.extrinsicsCalibration.calStatus = log.ExtrinsicsCalibration.Status.calibrated
        events.append(ec)

    with open(path, "wb") as f:
      for e in events:
        f.write(e.to_bytes())


def vin_with_check_digit(vin16: str) -> str:
  """Synthetic test VINs: put a valid check digit at position 9 of a 17-character string (position 9 given as '0')."""
  from openpilot.sunnypilot.selfdrive.ramweightd.start_weight import _VIN_VALUES, _VIN_WEIGHTS
  assert len(vin16) == 17
  total = sum(_VIN_VALUES[ch] * w for ch, w in zip(vin16, _VIN_WEIGHTS, strict=True))
  check = total % 11
  return vin16[:8] + ("X" if check == 10 else str(check)) + vin16[9:]


def np_rms(x) -> float:
  return float(np.sqrt(np.mean(np.square(x))))
