"""
Weight, air-drag and road-grade estimator for Ram Heavy Duty trucks (edfrahm/SP-RamHD-TICI-Port#8).

This file holds only arithmetic: no messaging, no settings, no CAN bus decoding. The service (ramweightd.py) and the
offline tools (replay.py, calibrate.py) feed it the same decoded samples, ten times per second, so a recorded drive
replays exactly as it ran on the truck. Nothing here steers, accelerates or brakes the truck.

Terms used below:
- Kalman filter: a standard way to combine noisy measurements with a model of how the estimated values change. It
  keeps a best estimate and its uncertainty (a covariance: the variances of the estimates and how their errors are
  linked). Each new measurement moves the estimate in proportion to how much it is trusted compared with what is
  already known, and narrows the uncertainty only in the directions that measurement actually informs.
- Innovation: the difference between a measurement and what the filter predicted it would be. Innovation gating
  rejects a measurement whose innovation is too large for the stated uncertainty (an outlier).
- Grade: road slope as rise over run (0.05 is a 5 percent climb), positive uphill. Internally the filter keeps the
  sine of the slope angle, which is what gravity acts through.
- Torque converter: the fluid coupling between engine and gearbox. When it is locked, engine speed and wheel speed
  are tied together and almost no power is lost in it.

The physics of the weight estimate (all forces along the road, newtons):
  driving force  F = efficiency * engine torque * engine angular speed / truck speed
                 (engine power reaching the wheels divided by speed; it needs no gear ratios)
  F = (mass + rotating parts) * acceleration + mass * g * (sine(grade) + rolling resistance * cosine(grade)) + c * speed^2
The unknowns are the combined mass and the air-drag factor c; both enter linearly, so a two-state Kalman filter fits
them together. Steady cruising on a flat road only tells the filter mass * g * rolling resistance + c * speed^2 (one
combination); accelerations, grades and different speeds separate the two.
"""
import math
from collections import deque
from dataclasses import dataclass, field

G = 9.81  # gravity, meters per second squared
LB_TO_KG = 0.45359237

# ---------------------------------------------------------------------------------------------------------------------
# Physical constants and starting values ("priors"). calibrate.py fits EFFICIENCY and ROLLING_RESISTANCE from drives
# with a known weight; put the values it prints here.
# ---------------------------------------------------------------------------------------------------------------------
# Share of engine power that reaches the tire contact patches with the converter locked: automatic gearbox about
# 0.92-0.95, axles about 0.95-0.97, minus fan, alternator and pumps. About 0.85 for a heavy-duty pickup.
EFFICIENCY = 0.85
# Rolling resistance coefficient: tire drag force divided by weight. Light-truck tires on asphalt are about
# 0.008-0.012; 0.009 is typical for load-range E all-season tires at proper pressure.
ROLLING_RESISTANCE = 0.009
# Rotating inertia of the engine, flexplate and locked torque converter (kilogram square meters). Power spent spinning
# them up is subtracted using the measured engine speed change, so no gear ratio is needed. A rough figure for a
# 6.7 L diesel with its converter; it matters mainly in the low gears.
ENGINE_INERTIA_KGM2 = 1.5
# The truck's body pitches (nose up) when it speeds up, which tilts both the Comma and the stability sensor, so both
# read a steeper climb than the road has. Radians of body pitch per meter per second squared of acceleration;
# subtracted from both grade measurements. Unknown until calibrate.py fits it from drives with a known weight.
BODY_PITCH_PER_ACCEL = 0.0
# Wheels, tires, brake rotors, axle shafts and drive shafts behave like this much extra mass when the truck speeds up:
# four wheel-and-tire sets of about 5 kg m2 each at a 0.42 m rolling radius, plus the shafts.
ROTATING_MASS_KG = 120.
# Air drag factor c (newtons per (meter per second) squared) = 0.5 * air density * drag coefficient * frontal area.
# Heavy-duty pickup: 0.5 * 1.2 kg/m3 * about 0.47 * about 3.6 m2 = about 1.0. A trailer can add 1 to 3.
DRAG_PRIOR = 1.0
DRAG_PRIOR_STD = 0.3
DRAG_TRAILER_STD = 1.0  # uncertainty used whenever a trailer may have been hitched
# Wind and changes of direction make c drift slowly while driving: this lets its uncertainty grow by about 0.1
# per 10 minutes. The mass gets no such growth while moving: weight only changes while stopped.
DRAG_PROCESS_VAR_PER_S = 0.1 ** 2 / 600.
DRAG_MIN, DRAG_MAX = 0.3, 8.0

# Combined weight limits. 40,000 lb is the design ceiling (truck plus the heaviest trailer considered).
CEILING_KG = 40000. * LB_TO_KG
MIN_MASS_KG = 2000.
MAX_MASS_KG = CEILING_KG * 1.15
# Used only when a verified trailer signal says no trailer is hitched: load in the bed and cab can still change.
PAYLOAD_STD_KG = 1500.

# ---------------------------------------------------------------------------------------------------------------------
# Clean moments: only these are used for the weight (see clean_reasons)
# ---------------------------------------------------------------------------------------------------------------------
MIN_SPEED_MPS = 5.0
MIN_ENGINE_TORQUE_NM = 60.
MIN_PEDAL_PCT = 2.0
BRAKE_PRESSURE_ON = 20.  # ESP_8 BRK_PRESSURE in its raw units; its resting value is still to be checked on drives (#5)
MAX_LATERAL_ACCEL_MPS2 = 1.5  # sharper turns add tire scrub drag the model does not include
MAX_GRADE_STD = 0.02
FORWARD_GEARS = ("drive", "low", "sport", "manumatic", "eco")
# Fallback rule for a locked converter while its own signal is not verified: engine speed divided by wheel speed
# stays within this fraction of its average for this long.
RATIO_WINDOW_S = 2.0
RATIO_TOLERANCE = 0.01

# ---------------------------------------------------------------------------------------------------------------------
# Weight updates
# ---------------------------------------------------------------------------------------------------------------------
# Both sides of the force equation are averaged over the same one-second window of clean samples (ten samples), so
# their timing matches exactly; the acceleration over the window is the speed change divided by the window length.
# Windows do not overlap, so their errors are close to independent.
WINDOW_SAMPLES = 10
FORCE_NOISE_N = 200.       # wind gusts, road roughness, signal steps
FORCE_REL_NOISE = 0.03     # error of the engine torque signal, as a share of the driving force
ACCEL_NOISE_MPS2 = 0.03    # error of the window's acceleration from wheel speed
# Grade errors last many seconds, so successive windows share them. Their variance is counted this many times over
# so the uncertainty does not narrow faster than the grade information really allows.
GRADE_ERROR_CORRELATION = 5.
GATE_SIGMA = 3.0           # innovations beyond this many standard deviations are rejected as outliers
# A window whose acceleration plus g * sine(grade) is within this many noise widths of zero (steady driving on a level
# road) tells nothing about the weight by itself: there, noise in the measured grade and acceleration would look like
# information and pull the weight down (the "errors in the input variables" bias). Such a window is used only for
# the combination mass * g * rolling resistance + c * speed^2; its small acceleration and grade terms are moved to the
# measured side using the current weight estimate.
EXCITATION_SIGMAS = 3.0

# ---------------------------------------------------------------------------------------------------------------------
# Stops
# ---------------------------------------------------------------------------------------------------------------------
STOP_SPEED_MPS = 0.3
# A stop is "qualifying" (the load may have changed) when the shifter is in park, a door opens, a verified trailer
# signal changes, or the truck stands still this long. 60 seconds is too short: long red lights would reset the
# learned weight several times a drive.
QUALIFYING_STOP_S = 120.

# ---------------------------------------------------------------------------------------------------------------------
# Grade filter
# ---------------------------------------------------------------------------------------------------------------------
GRADE_LPF_S = 0.5               # the same smoothing for wheel-speed acceleration and the sensor's acceleration
GRADE_VAR_PER_M = 4e-6          # grade can change about 0.02 per 100 m driven (vertical curves)
GRADE_VAR_PER_S = 1e-7
SENSOR_OFFSET_VAR_PER_S = 1e-6
GRADE_INIT_STD = 0.05
SENSOR_OFFSET_INIT_STD = 0.3    # meters per second squared: a mounting tilt of up to about 2 degrees
SENSOR_NOISE_MPS2 = 0.06        # steps of 0.08 m/s2, body bounce
SENSOR_NOISE_PER_ACCEL = 0.05   # the body pitches when speeding up or braking, tilting the sensor
SENSOR_GATE_SIGMA = 5.0
SENSOR_WARMUP_SAMPLES = 10
PITCH_NOISE_FLOOR = 0.004       # mounting and body-pitch error left after the Comma's calibration
PITCH_CORRELATION = 4.          # the Comma's pitch is itself a filtered estimate: successive samples share errors
PITCH_ONLY_STD_FLOOR = 0.006    # without the stability sensor, never claim a grade better than this
MAX_SINE_GRADE = 0.37           # rise over run 0.4

# ---------------------------------------------------------------------------------------------------------------------
# Fuel
# ---------------------------------------------------------------------------------------------------------------------
FUEL_LPF_MOVING_S = 300.  # fuel sloshes when speeding up, braking, turning and on grades: smooth heavily
FUEL_WARMUP_S = 300.      # the first level of a drive that starts moving: an average over the first 5 minutes
FUEL_DEADBAND_PCT = 0.5   # smoothed dips smaller than this are slosh, not fuel burned
FUEL_LPF_STOPPED_S = 10.  # at a stop the level is calm
FUEL_SETTLE_S = 20.       # wait this long after stopping before trusting the level
FUEL_STABLE_S = 10.       # ... and until it has held steady (within 1 percent) this long: refueling has finished
REFUEL_MIN_PCT = 5.       # a rise of at least this much at a stop is refueling

STATE_VERSION = 1


def lb(kg: float) -> float:
  return kg / LB_TO_KG


class LowPass:
  """First-order low-pass filter: smooths a signal with time constant tau seconds.

  Until tau seconds of input have been seen it is a plain average of everything so far, so its first value does
  not linger (a first reading taken while fuel sloshes would otherwise bias it for minutes).
  """

  def __init__(self, tau: float):
    self.tau = tau
    self.x: float | None = None
    self.elapsed = 0.

  def update(self, u: float, dt: float) -> float:
    if self.x is None:
      self.x = u
      self.elapsed = dt
    else:
      self.elapsed += dt
      self.x += max(dt / (self.tau + dt), dt / self.elapsed) * (u - self.x)
    return self.x

  def set(self, x: float) -> None:
    self.x = x
    self.elapsed = self.tau

  def reset(self) -> None:
    self.x = None
    self.elapsed = 0.


def _kf_update(x: list[float], P: list[list[float]], H: tuple[float, float], z: float, R: float,
               gate_sigma: float | None = None) -> tuple[bool, float]:
  """One measurement update of a two-state Kalman filter, in place. Returns (accepted, innovation in sigmas)."""
  ph0 = P[0][0] * H[0] + P[0][1] * H[1]
  ph1 = P[1][0] * H[0] + P[1][1] * H[1]
  S = H[0] * ph0 + H[1] * ph1 + R
  innovation = z - (H[0] * x[0] + H[1] * x[1])
  sigmas = abs(innovation) / math.sqrt(S)
  if gate_sigma is not None and sigmas > gate_sigma:
    return False, sigmas
  k0, k1 = ph0 / S, ph1 / S
  x[0] += k0 * innovation
  x[1] += k1 * innovation
  P[0][0] -= k0 * ph0
  P[0][1] -= k0 * ph1
  P[1][0] -= k1 * ph0
  P[1][1] -= k1 * ph1
  # keep the covariance symmetric and positive
  P[0][1] = P[1][0] = 0.5 * (P[0][1] + P[1][0])
  P[0][0] = max(P[0][0], 1e-12)
  P[1][1] = max(P[1][1], 1e-12)
  return True, sigmas


class GradeFilter:
  """Two-state Kalman filter for the road grade.

  States: the sine of the grade angle, and the offset of the stability-control sensor's acceleration reading.
  Measurements:
  - stability sensor: the sensor feels acceleration plus gravity along the slope, wheel speed gives the acceleration
    alone, so (sensor acceleration - wheel-speed acceleration) = g * sine(grade) + offset. Both are smoothed by the
    same low-pass filter so their timing matches.
  - the Comma's pitch: the calibrated pitch of the road ahead from openpilot's location filter (deviceMotion
    orientation, turned into the truck's frame with extrinsicsCalibration's mounting angles).
  With both, the Comma's pitch fixes the slow offset and the sensor follows quick changes. Without the sensor, the
  Comma's pitch is used alone and the published uncertainty is never below PITCH_ONLY_STD_FLOOR.
  """

  def __init__(self, body_pitch_per_accel: float = BODY_PITCH_PER_ACCEL):
    self.body_pitch_per_accel = body_pitch_per_accel
    self.x = [0., 0.]  # sine of grade, sensor offset (m/s2)
    self.P = [[GRADE_INIT_STD ** 2, 0.], [0., SENSOR_OFFSET_INIT_STD ** 2]]
    self.v_lpf = LowPass(GRADE_LPF_S)
    self.sensor_lpf = LowPass(GRADE_LPF_S)
    self.prev_v_f: float | None = None
    self.sensor_samples = 0
    self.wheel_accel = 0.
    self.source = "none"
    self.last_sensor_used = False
    self.last_pitch_used = False

  def reset_smoothing(self) -> None:
    self.v_lpf.reset()
    self.sensor_lpf.reset()
    self.prev_v_f = None
    self.sensor_samples = 0

  def step(self, dt: float, v: float, sensor_accel: float | None, pitch: float | None, pitch_std: float | None) -> None:
    # predict: grade drifts with distance driven, the sensor offset barely at all
    self.P[0][0] += GRADE_VAR_PER_M * abs(v) * dt + GRADE_VAR_PER_S * dt
    self.P[1][1] += SENSOR_OFFSET_VAR_PER_S * dt

    v_f = self.v_lpf.update(v, dt)
    self.wheel_accel = 0. if self.prev_v_f is None else (v_f - self.prev_v_f) / dt
    self.prev_v_f = v_f

    body_pitch = self.body_pitch_per_accel * self.wheel_accel
    used_sensor = False
    if sensor_accel is not None:
      a_f = self.sensor_lpf.update(sensor_accel, dt)
      self.sensor_samples += 1
      if self.sensor_samples > SENSOR_WARMUP_SAMPLES:
        R = (SENSOR_NOISE_MPS2 + SENSOR_NOISE_PER_ACCEL * abs(self.wheel_accel)) ** 2
        z = a_f - self.wheel_accel - G * body_pitch
        used_sensor, _ = _kf_update(self.x, self.P, (G, 1.), z, R, SENSOR_GATE_SIGMA)
    else:
      self.sensor_lpf.reset()
      self.sensor_samples = 0

    used_pitch = False
    if pitch is not None and math.isfinite(pitch):
      sd = max(pitch_std if (pitch_std is not None and math.isfinite(pitch_std)) else 0., 0.002)
      R = (sd ** 2 + PITCH_NOISE_FLOOR ** 2) * PITCH_CORRELATION
      used_pitch, _ = _kf_update(self.x, self.P, (1., 0.), math.sin(pitch - body_pitch), R)

    self.x[0] = min(max(self.x[0], -MAX_SINE_GRADE), MAX_SINE_GRADE)
    self.last_sensor_used, self.last_pitch_used = used_sensor, used_pitch
    if used_sensor and used_pitch:
      self.source = "sensorAndPitch"
    elif used_sensor:
      self.source = "sensorOnly"
    elif used_pitch:
      self.source = "pitchOnly"
    # else keep the last source (a sample without measurements only predicts)

  @property
  def sine(self) -> float:
    return self.x[0]

  @property
  def sine_std(self) -> float:
    std = math.sqrt(self.P[0][0])
    if self.source == "pitchOnly":
      std = max(std, PITCH_ONLY_STD_FLOOR)
    return std

  @property
  def grade(self) -> float:
    s = self.x[0]
    return s / math.sqrt(1. - s * s)

  @property
  def grade_std(self) -> float:
    c = math.sqrt(1. - self.x[0] ** 2)
    return self.sine_std / c ** 3

  @property
  def sensor_offset(self) -> float:
    return self.x[1]


class RatioSteadiness:
  """Engine speed divided by wheel speed, over the last RATIO_WINDOW_S seconds.

  In a fixed gear with the torque converter locked the ratio is constant (the gear ratio times the axle ratio);
  with the converter slipping or a shift under way it moves. "Steady" means every sample of the last two seconds is
  within RATIO_TOLERANCE (1 percent) of their average.
  """

  def __init__(self):
    self.samples: deque[tuple[float, float]] = deque()

  def reset(self) -> None:
    self.samples.clear()

  def update(self, t: float, engine_rpm: float | None, wheel_rpm: float | None, v: float) -> tuple[bool, bool]:
    """Returns (valid, steady)."""
    ratio = None
    if engine_rpm is not None and engine_rpm > 500.:
      if wheel_rpm is not None and wheel_rpm > 30.:
        ratio = engine_rpm / wheel_rpm
      elif wheel_rpm is None and v > 1.:
        ratio = engine_rpm / v
    if ratio is None:
      self.samples.clear()
      return False, False
    self.samples.append((t, ratio))
    while self.samples and t - self.samples[0][0] > RATIO_WINDOW_S + 1e-6:
      self.samples.popleft()
    if t - self.samples[0][0] < RATIO_WINDOW_S - 0.051:
      return True, False
    mean = sum(r for _, r in self.samples) / len(self.samples)
    steady = all(abs(r - mean) <= RATIO_TOLERANCE * mean for _, r in self.samples)
    return True, steady


@dataclass
class Inputs:
  """One decoded sample. None means the signal is unavailable right now."""
  t: float                                   # seconds, monotonic
  v: float                                   # truck speed from wheel speed, m/s (carState.vEgoRaw)
  gear: str = "unknown"                      # shifter position (carState.gearShifter)
  door_open: bool | None = None
  brake_pedal: bool | None = None            # ESP_1 Brake_Pedal_State
  brake_pressure: float | None = None        # ESP_8 BRK_PRESSURE
  accel_pedal_pct: float | None = None       # ECM_5 Accelerator_Position
  acc_active: bool | None = None             # DAS_3 ACC_ACTIVE (factory cruise engaged)
  acc_torque_request_nm: float | None = None  # DAS_3 ENGINE_TORQUE_REQUEST
  engine_rpm: float | None = None            # ECM_1
  engine_torque_nm: float | None = None      # ECM_1
  rear_wheel_rpm: float | None = None        # ESP_6, average of the two rear wheels
  traction_control: bool | None = None       # ESP_2 TCS_ACTIVE
  stability_accel: float | None = None       # ESP_4 (pending #5)
  converter_locked: bool | None = None       # TRANS_SPEED (pending #5)
  shift_pending: bool | None = None          # TRANS_SPEED (pending #5)
  exhaust_brake: bool | None = None          # not found yet (#5)
  trailer_connected: bool | None = None      # not found yet (#5)
  fuel_level_pct: float | None = None        # not found yet (#5)
  pitch: float | None = None                 # the Comma's calibrated pitch, radians, positive nose up
  pitch_std: float | None = None
  lateral_accel: float | None = None         # from the Comma's yaw rate times speed


def clean_reasons(inp: Inputs, converter_locked: bool, lock_rule: str, ratio_steady: bool, grade_std: float) -> list[str]:
  """Every reason this sample is NOT a clean moment for the weight, in words. An empty list means clean.

  Clean: speed above MIN_SPEED_MPS, a forward gear, torque converter locked, no service brake, no exhaust brake, no
  shift in progress, engine pulling (positive engine torque with the accelerator pressed or the factory cruise asking
  for torque), no traction control, no sharp turn, and a known grade.
  While a signal is not verified (#5): the exhaust brake only works with the accelerator released, so "engine
  pulling" already excludes it; a shift changes the engine-to-wheel speed ratio, so the steady-ratio rule excludes it.
  """
  reasons = []
  if inp.v < MIN_SPEED_MPS:
    reasons.append("speed below 5 m/s")
  if inp.gear not in FORWARD_GEARS:
    reasons.append("not in a forward gear")
  if inp.engine_rpm is None or inp.engine_torque_nm is None:
    reasons.append("engine signals missing")
  if lock_rule == "none":
    reasons.append("converter state unknown")
  elif not converter_locked:
    reasons.append("converter not locked")
  if inp.brake_pedal or (inp.brake_pressure is not None and inp.brake_pressure > BRAKE_PRESSURE_ON):
    reasons.append("service brake on")
  if inp.exhaust_brake:
    reasons.append("exhaust brake on")
  if inp.shift_pending is not None:
    if inp.shift_pending:
      reasons.append("shift in progress")
  elif not ratio_steady:
    reasons.append("possible shift (ratio not steady)")
  pulling_request = (inp.accel_pedal_pct is not None and inp.accel_pedal_pct > MIN_PEDAL_PCT) or \
                    (bool(inp.acc_active) and inp.acc_torque_request_nm is not None and inp.acc_torque_request_nm > MIN_ENGINE_TORQUE_NM)
  if inp.engine_torque_nm is not None and (inp.engine_torque_nm <= MIN_ENGINE_TORQUE_NM or not pulling_request):
    reasons.append("engine not pulling")
  if inp.traction_control:
    reasons.append("traction control active")
  if inp.lateral_accel is not None and abs(inp.lateral_accel) > MAX_LATERAL_ACCEL_MPS2:
    reasons.append("turning")
  if grade_std > MAX_GRADE_STD:
    reasons.append("grade unknown")
  return reasons


class MassDragFilter:
  """Two-state Kalman filter: combined mass (kg, at the current fuel reference) and air-drag factor c."""

  def __init__(self, mass_kg: float, drag: float, P: list[list[float]]):
    self.x = [mass_kg, drag]
    self.P = [list(P[0]), list(P[1])]
    self.learned_P = [list(P[0]), list(P[1])]  # the covariance before the last widening
    self.updated_since_widen = False

  def predict(self, dt: float, moving: bool) -> None:
    # no process noise on the mass, ever: it changes only through widening at qualifying stops
    if moving:
      self.P[1][1] += DRAG_PROCESS_VAR_PER_S * dt

  def update(self, H: tuple[float, float], z: float, R: float) -> tuple[bool, float]:
    accepted, sigmas = _kf_update(self.x, self.P, H, z, R, GATE_SIGMA)
    if accepted:
      self.x[0] = min(max(self.x[0], MIN_MASS_KG), MAX_MASS_KG)
      self.x[1] = min(max(self.x[1], DRAG_MIN), DRAG_MAX)
      self.updated_since_widen = True
    return accepted, sigmas

  def widen(self, trailer_possible: bool) -> None:
    """Keep the values, widen the uncertainty so any load change since can be learned.

    With a possible trailer: the 40,000 lb ceiling sits two standard deviations above the current mass, and the
    drag uncertainty grows to DRAG_TRAILER_STD. Without one (verified trailer signal says none): PAYLOAD_STD_KG.
    """
    if self.updated_since_widen:
      self.learned_P = [list(self.P[0]), list(self.P[1])]
    self.updated_since_widen = False
    if trailer_possible:
      mass_std = max((CEILING_KG - self.x[0]) / 2., PAYLOAD_STD_KG)
      self.P[1][1] = max(self.P[1][1], DRAG_TRAILER_STD ** 2)
    else:
      mass_std = PAYLOAD_STD_KG
    self.P[0][0] = max(self.P[0][0], mass_std ** 2)
    self.P[0][1] = self.P[1][0] = 0.

  @property
  def mass(self) -> float:
    return self.x[0]

  @property
  def drag(self) -> float:
    return self.x[1]

  @property
  def mass_std(self) -> float:
    return math.sqrt(self.P[0][0])

  @property
  def drag_std(self) -> float:
    return math.sqrt(self.P[1][1])

  @property
  def correlation(self) -> float:
    return self.P[0][1] / math.sqrt(self.P[0][0] * self.P[1][1])

  def saved_covariance(self) -> list[list[float]]:
    """What to keep between drives: the learned covariance, not a widening nothing has been learned from since."""
    return self.P if self.updated_since_widen else self.learned_P


@dataclass
class WindowSample:
  t: float
  v: float
  power_per_speed: float  # engine torque * engine angular speed / truck speed, newtons (before efficiency)
  omega: float            # engine angular speed, radians per second
  sine: float
  cosine: float


@dataclass
class Window:
  """One second of clean samples, averaged. Inputs to one weight update (and to calibrate.py)."""
  t: float
  duration: float
  v_mean: float
  accel: float          # speed change / duration
  engine_force: float   # mean engine power / speed, minus power spent spinning up the engine; newtons, before efficiency
  sine: float
  cosine: float
  v2: float             # mean of speed squared
  grade_std: float


def _trapezoid_mean(ts: list[float], xs: list[float]) -> float:
  total = 0.
  for i in range(len(ts) - 1):
    total += 0.5 * (xs[i] + xs[i + 1]) * (ts[i + 1] - ts[i])
  return total / (ts[-1] - ts[0])


class WindowAccumulator:
  def __init__(self):
    self.samples: list[WindowSample] = []

  def reset(self) -> None:
    self.samples = []

  def add(self, sample: WindowSample, grade_std: float) -> Window | None:
    self.samples.append(sample)
    if len(self.samples) < WINDOW_SAMPLES + 1:
      return None
    s = self.samples
    ts = [x.t for x in s]
    duration = ts[-1] - ts[0]
    v_mean = _trapezoid_mean(ts, [x.v for x in s])
    inertia_force = ENGINE_INERTIA_KGM2 * (s[-1].omega ** 2 - s[0].omega ** 2) / (2. * duration * max(v_mean, 1.))
    w = Window(
      t=ts[-1],
      duration=duration,
      v_mean=v_mean,
      accel=(s[-1].v - s[0].v) / duration,
      engine_force=_trapezoid_mean(ts, [x.power_per_speed for x in s]) - inertia_force,
      sine=_trapezoid_mean(ts, [x.sine for x in s]),
      cosine=_trapezoid_mean(ts, [x.cosine for x in s]),
      v2=_trapezoid_mean(ts, [x.v * x.v for x in s]),
      grade_std=grade_std,
    )
    self.samples = [s[-1]]  # windows touch but do not overlap
    return w


def window_measurement(w: Window, fuel_offset_kg: float, mass_guess_kg: float, mass_var: float = 0.,
                       efficiency: float = EFFICIENCY, rolling_resistance: float = ROLLING_RESISTANCE):
  """The weight update for one window: (H, z, R, informs_weight) of z = H . [mass at fuel reference, c] + noise.

  informs_weight is False for a steady, level window, which only informs the rolling-plus-air-drag combination.
  """
  h_level = G * rolling_resistance * w.cosine  # what h would be in steady driving on a level road
  excitation = w.accel + G * w.sine            # the part of h that separates the weight from the air drag
  h = h_level + excitation
  force = efficiency * w.engine_force
  # current mass = mass at the fuel reference - fuel burned since; move the known part to the measured side
  z = force - ROTATING_MASS_KG * w.accel + fuel_offset_kg * h
  R = FORCE_NOISE_N ** 2 + (FORCE_REL_NOISE * force) ** 2 + (mass_guess_kg * ACCEL_NOISE_MPS2) ** 2 + \
      GRADE_ERROR_CORRELATION * (mass_guess_kg * G * w.grade_std) ** 2
  noise_width = math.hypot(ACCEL_NOISE_MPS2, G * w.grade_std)
  if abs(excitation) < EXCITATION_SIGMAS * noise_width:
    z -= (mass_guess_kg - fuel_offset_kg) * excitation
    R += mass_var * excitation ** 2
    return (h_level, w.v2), z, R, False
  return (h, w.v2), z, R, True


class FuelTracker:
  """Fuel burned, from the fuel level. Inactive (all zero) while the fuel-level signal is not available.

  The level sloshes, so while moving it is smoothed over FUEL_LPF_MOVING_S and only decreases count: the level
  estimate only moves down, and only when the smoothed level is more than FUEL_DEADBAND_PCT below it. At a stop the
  level is calm: after FUEL_SETTLE_S, once it has held steady for FUEL_STABLE_S, a rise of at least REFUEL_MIN_PCT
  is refueling.
  """

  def __init__(self, tank_gal: float, lb_per_gal: float, last_drive_level_pct: float | None = None):
    self.kg_per_pct = tank_gal * lb_per_gal * LB_TO_KG / 100.
    self.moving_lpf = LowPass(FUEL_LPF_MOVING_S)
    self.calm_lpf = LowPass(FUEL_LPF_STOPPED_S)
    self.moving_s = 0.
    self.stable_s = 0.
    self.ref: float | None = None  # current level estimate, percent
    self.burned_kg = 0.            # this drive
    self.last_drive_level_pct = last_drive_level_pct
    self.first_level_pct: float | None = None

  @property
  def level_pct(self) -> float | None:
    return self.ref

  def _set_first(self, level: float) -> float:
    """First level of the drive. Returns the fuel mass change since the level the estimate assumed (refuel or use)."""
    self.ref = level
    self.first_level_pct = level
    if self.last_drive_level_pct is not None and abs(level - self.last_drive_level_pct) > 1.:
      return (level - self.last_drive_level_pct) * self.kg_per_pct
    return 0.

  def step(self, dt: float, level: float | None, moving: bool, stopped_s: float) -> float:
    """Returns the fuel mass added at this moment (refueling), in kilograms; usually 0."""
    if level is None:
      return 0.
    added = 0.
    if moving:
      self.calm_lpf.reset()
      self.stable_s = 0.
      self.moving_s += dt
      smoothed = self.moving_lpf.update(level, dt)
      if self.ref is None:
        if self.moving_s >= FUEL_WARMUP_S:
          added = self._set_first(smoothed)
      else:
        new_ref = min(self.ref, smoothed + FUEL_DEADBAND_PCT)
        self.burned_kg += (self.ref - new_ref) * self.kg_per_pct
        self.ref = new_ref
    elif stopped_s >= FUEL_SETTLE_S:
      calm = self.calm_lpf.update(level, dt)
      self.stable_s = self.stable_s + dt if abs(level - calm) < 1. else 0.
      if self.stable_s >= FUEL_STABLE_S:
        if self.ref is None:
          added = self._set_first(calm)
          self.moving_lpf.set(calm)
        elif calm > self.ref + REFUEL_MIN_PCT:
          added = (calm - self.ref) * self.kg_per_pct
          self.ref = calm
          self.moving_lpf.set(calm)
    return added


@dataclass
class Output:
  mass_kg: float
  mass_std_kg: float
  drag: float
  drag_std: float
  grade: float
  grade_std: float
  sensor_offset: float
  grade_source: str
  clean: bool
  not_clean_reasons: list[str]
  clean_samples: int
  mass_updates: int
  rejected_updates: int
  qualifying_stops: int
  mass_drag_correlation: float
  fuel_burned_kg: float
  fuel_level_pct: float | None
  converter_locked: bool
  converter_lock_rule: str
  service_brake_on: bool
  events: list[str] = field(default_factory=list)


class WeightGradeEstimator:
  def __init__(self, start_mass_kg: float, start_mass_std_kg: float, tank_gal: float, lb_per_gal: float,
               persisted: dict | None = None, record_windows: bool = False, start_fuel_level_pct: float | None = None,
               efficiency: float = EFFICIENCY, rolling_resistance: float = ROLLING_RESISTANCE,
               body_pitch_per_accel: float = BODY_PITCH_PER_ACCEL):
    # efficiency, rolling_resistance and body_pitch_per_accel can be overridden to try values from calibrate.py
    # start_fuel_level_pct: the fuel level the starting weight assumed (for example 50 when the level was unknown), so
    # the mass is corrected once the real level is read
    last_level = start_fuel_level_pct
    if persisted is not None:
      cov = persisted.get("mass_drag_cov", 0.)
      P = [[persisted["mass_std_kg"] ** 2, cov], [cov, persisted["drag_std"] ** 2]]
      self.mass = MassDragFilter(persisted["mass_kg"], persisted["drag"], P)
      last_level = persisted.get("fuel_level_pct")
    else:
      self.mass = MassDragFilter(start_mass_kg, DRAG_PRIOR, [[start_mass_std_kg ** 2, 0.], [0., DRAG_PRIOR_STD ** 2]])
    # every drive starts like a qualifying stop: a trailer may have been hitched while the truck was off
    self.mass.widen(trailer_possible=True)

    self.efficiency = efficiency
    self.rolling_resistance = rolling_resistance
    self.grade = GradeFilter(body_pitch_per_accel)
    self.ratio = RatioSteadiness()
    self.window = WindowAccumulator()
    self.fuel = FuelTracker(tank_gal, lb_per_gal, last_level)
    self.fuel_offset_kg = 0.   # fuel burned since the mass reference
    self.fuel_burned_seen = 0.

    self.t_prev: float | None = None
    self.stopped_s = 0.
    self.stop_qualified = False
    self.stop_trailer: bool | None = None
    self.prev_trailer: bool | None = None

    self.clean_samples = 0
    self.mass_updates = 0
    self.rejected_updates = 0
    self.qualifying_stops = 0
    self.record_windows = record_windows
    self.windows: list[tuple[Window, float, bool]] = []  # (window, fuel offset kg, accepted)

  @property
  def mass_kg(self) -> float:
    return self.mass.mass - self.fuel_offset_kg

  def _qualifying_stop(self, trailer_possible: bool, why: str, events: list[str]) -> None:
    self.stop_qualified = True
    self.qualifying_stops += 1
    self.mass.widen(trailer_possible)
    self.window.reset()
    events.append(f"qualifying stop ({why}): weight kept at {lb(self.mass_kg):.0f} lb, uncertainty widened")

  def step(self, inp: Inputs) -> Output:
    events: list[str] = []
    dt = 0.1 if self.t_prev is None else inp.t - self.t_prev
    if dt <= 0. or dt > 0.5:
      # first sample, a repeated timestamp or a gap in the data: start the smoothing and windows over
      self.grade.reset_smoothing()
      self.ratio.reset()
      self.window.reset()
      dt = 0.1
    self.t_prev = inp.t
    moving = inp.v > STOP_SPEED_MPS
    trailer_possible = inp.trailer_connected is None or bool(inp.trailer_connected)

    # stops
    if moving:
      self.stopped_s = 0.
      self.stop_qualified = False
    else:
      self.stopped_s += dt
      if not self.stop_qualified:
        if inp.gear == "park":
          self._qualifying_stop(trailer_possible, "park", events)
        elif inp.door_open:
          self._qualifying_stop(trailer_possible, "door opened", events)
        elif inp.trailer_connected is not None and self.prev_trailer is not None and inp.trailer_connected != self.prev_trailer:
          self._qualifying_stop(True, "trailer connection changed", events)
        elif self.stopped_s >= QUALIFYING_STOP_S:
          self._qualifying_stop(trailer_possible, f"stopped {QUALIFYING_STOP_S:.0f} s", events)
    if inp.trailer_connected is not None:
      self.prev_trailer = inp.trailer_connected

    # grade
    self.grade.step(dt, inp.v, inp.stability_accel, inp.pitch, inp.pitch_std)

    # fuel
    added_kg = self.fuel.step(dt, inp.fuel_level_pct, moving, self.stopped_s)
    burned_now = self.fuel.burned_kg - self.fuel_burned_seen
    self.fuel_burned_seen = self.fuel.burned_kg
    self.fuel_offset_kg += burned_now
    if added_kg != 0.:
      # at a stop (or the first level of the drive): move the fuel change into the mass itself
      self.mass.x[0] = min(max(self.mass.x[0] - self.fuel_offset_kg + added_kg, MIN_MASS_KG), MAX_MASS_KG)
      self.fuel_offset_kg = 0.
      events.append(f"fuel change at a stop: {lb(added_kg):+.0f} lb")

    # torque converter and clean moment
    ratio_valid, ratio_steady = self.ratio.update(inp.t, inp.engine_rpm, inp.rear_wheel_rpm, inp.v)
    if inp.converter_locked is not None:
      lock_rule, locked = "signal", bool(inp.converter_locked)
    elif ratio_valid:
      lock_rule, locked = "ratioFallback", ratio_steady
    else:
      lock_rule, locked = "none", False
    reasons = clean_reasons(inp, locked, lock_rule, ratio_steady, self.grade.grade_std)
    clean = not reasons

    # weight and drag
    self.mass.predict(dt, moving)
    if clean:
      self.clean_samples += 1
      omega = inp.engine_rpm * 2. * math.pi / 60.
      s = self.grade.sine
      sample = WindowSample(inp.t, inp.v, inp.engine_torque_nm * omega / inp.v, omega, s, math.sqrt(1. - s * s))
      w = self.window.add(sample, self.grade.grade_std)
      if w is not None:
        H, z, R, _ = window_measurement(w, self.fuel_offset_kg, self.mass.mass, self.mass.P[0][0], self.efficiency,
                                        self.rolling_resistance)
        accepted, _ = self.mass.update(H, z, R)
        if accepted:
          self.mass_updates += 1
        else:
          self.rejected_updates += 1
        if self.record_windows:
          self.windows.append((w, self.fuel_offset_kg, accepted))
    else:
      self.window.reset()

    service_brake = bool(inp.brake_pedal) or (inp.brake_pressure is not None and inp.brake_pressure > BRAKE_PRESSURE_ON)
    return Output(
      mass_kg=self.mass_kg,
      mass_std_kg=self.mass.mass_std,
      drag=self.mass.drag,
      drag_std=self.mass.drag_std,
      grade=self.grade.grade,
      grade_std=self.grade.grade_std,
      sensor_offset=self.grade.sensor_offset,
      grade_source=self.grade.source,
      clean=clean,
      not_clean_reasons=reasons,
      clean_samples=self.clean_samples,
      mass_updates=self.mass_updates,
      rejected_updates=self.rejected_updates,
      qualifying_stops=self.qualifying_stops,
      mass_drag_correlation=self.mass.correlation,
      fuel_burned_kg=self.fuel.burned_kg,
      fuel_level_pct=self.fuel.level_pct,
      converter_locked=locked,
      converter_lock_rule=lock_rule,
      service_brake_on=service_brake,
      events=events,
    )

  def state(self) -> dict:
    """What to keep in RamWeightGradeState between drives (no VIN, no location)."""
    P = self.mass.saved_covariance()
    return {
      "version": STATE_VERSION,
      "mass_kg": round(self.mass_kg, 1),
      "mass_std_kg": round(math.sqrt(P[0][0]), 1),
      "drag": round(self.mass.drag, 4),
      "drag_std": round(math.sqrt(P[1][1]), 4),
      "mass_drag_cov": round(P[0][1], 4),
      "fuel_level_pct": None if self.fuel.level_pct is None else round(self.fuel.level_pct, 2),
      "mass_updates": self.mass_updates,
    }


def valid_persisted_state(state) -> bool:
  if not isinstance(state, dict) or state.get("version") != STATE_VERSION:
    return False
  try:
    values = [float(state[k]) for k in ("mass_kg", "mass_std_kg", "drag", "drag_std")]
    cov = float(state.get("mass_drag_cov", 0.))
  except (KeyError, TypeError, ValueError):
    return False
  if not all(math.isfinite(x) for x in values + [cov]):
    return False
  mass, mass_std, drag, drag_std = values
  if not (MIN_MASS_KG <= mass <= MAX_MASS_KG and DRAG_MIN <= drag <= DRAG_MAX and mass_std > 0. and drag_std > 0.):
    return False
  return cov * cov < mass_std ** 2 * drag_std ** 2
