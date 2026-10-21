"""
When ramweightd reads the truck (edfrahm/SP-RamHD-TICI-Port#8).

The weight does not change while driving, and most of the service's processor time goes to reading the truck's CAN
messages (about 100 batches a second), not to the weight math. So the service reads the truck only while it has
something to learn, and publishes only when its answer changes:

- LEARN: from the start of each drive, and after a qualifying stop, until the estimate has settled (one standard
  deviation within SETTLE_FRACTION of the estimate, after at least MIN_UPDATES weight updates in this period), or until
  LEARN_LIMIT_S of driving have passed without settling.
- SLEEP: no CAN, no accelerometer, no pose. carState about once a second, only to notice a qualifying stop, with the
  same rules as the estimator (park, a door opened, or standing still estimator.QUALIFYING_STOP_S). A qualifying stop
  widens the uncertainty and starts a new LEARN period.
- FUEL: every FUEL_PERIOD_S asleep, read the truck again for FUEL_READ_S to take the fuel gauge; the fuel burned since
  comes off the published weight. If the estimate had not settled, a new LEARN period follows.

Why SETTLE_FRACTION is 10%: on Eric's empty truck only rolling resistance (about 0.0085 g of the 0.037 g coast-down at
50-72 mph) scales with weight; air drag and engine braking are fixed forces. A weight 20% off (+/-1,700 lb) makes a
coast-down predicted from the weight 13-19% off; 10% makes it 7-9% off. Highway drives settled at 10% in 3.6-6.7
minutes; mixed drives did not within 15, hence the limit.
"""
from dataclasses import dataclass

from openpilot.sunnypilot.selfdrive.ramweightd.estimator import QUALIFYING_STOP_S, STOP_SPEED_MPS, lb

LEARN = "learn"
SLEEP = "sleep"
FUEL = "fuel"

SETTLE_FRACTION = 0.10      # settled: one standard deviation within this fraction of the estimate...
MIN_UPDATES = 10            # ...after at least this many weight updates (3-second windows) in this learning period
LEARN_LIMIT_S = 20. * 60.   # stop learning after this much driving without settling; retried at the next fuel read
FUEL_PERIOD_S = 3600.       # asleep this long: read the fuel gauge
FUEL_READ_S = 10.           # how long a fuel read listens (the gauge message comes about once a second)
SLEEP_CHECK_S = 1.0         # asleep: look at carState this often
HEARTBEAT_S = 10.           # publish at least this often, so a reader knows the service is alive


@dataclass(frozen=True)
class Snapshot:
  """What a reader of ramWeightGrade cares about; a new message goes out when it changes."""
  mode: str
  mass_lb: int
  mass_std_lb: int
  qualifying_stops: int
  fuel_level_pct: float | None


class DutyCycle:
  def __init__(self, t: float):
    self.mode = LEARN
    self.mode_t = t
    self.learn_moving_s = 0.
    self.updates_at_period_start: int | None = None
    self.settled = False
    self.asleep_since = t
    # stop watching while asleep (the estimator does its own while it is stepped)
    self.stopped_s = 0.
    self.stop_qualified = False
    self.last_check_t: float | None = None
    # publishing
    self.last_snapshot: Snapshot | None = None
    self.last_publish_t: float | None = None

  @property
  def listening(self) -> bool:
    return self.mode != SLEEP

  def _set(self, mode: str, t: float, why: str, events: list[str]) -> None:
    events.append(f"{self.mode} -> {mode}: {why}")
    self.mode, self.mode_t = mode, t
    if mode == LEARN:
      self.learn_moving_s = 0.
      self.updates_at_period_start = None
    elif mode == SLEEP:
      self.asleep_since = t
      self.last_check_t = None  # the stop state carries over from the estimator (listening_step)

  def after_step(self, t: float, dt: float, moving: bool, mass_kg: float, mass_std_kg: float, mass_updates: int) -> list[str]:
    """Call after each estimator step while listening. Returns what changed, in words."""
    events: list[str] = []
    if self.updates_at_period_start is None:
      self.updates_at_period_start = mass_updates
    within = mass_std_kg <= SETTLE_FRACTION * mass_kg
    if self.mode == LEARN:
      self.settled = within and mass_updates - self.updates_at_period_start >= MIN_UPDATES
    else:
      self.settled = self.settled and within  # a fuel read keeps a settled estimate settled
    if self.mode == LEARN:
      if moving:
        self.learn_moving_s += dt
      if self.settled:
        self._set(SLEEP, t, f"settled at {lb(mass_kg):.0f} +/- {lb(mass_std_kg):.0f} lb after {self.learn_moving_s / 60.:.1f} min "
                  "of driving", events)
      elif self.learn_moving_s >= LEARN_LIMIT_S:
        self._set(SLEEP, t, f"not settled after {LEARN_LIMIT_S / 60.:.0f} min of driving ({lb(mass_kg):.0f} +/- "
                  f"{lb(mass_std_kg):.0f} lb); trying again at the next fuel read", events)
    elif self.mode == FUEL and t - self.mode_t >= FUEL_READ_S:
      if self.settled:
        self._set(SLEEP, t, "fuel read", events)
      else:
        self._set(LEARN, t, "fuel read; the estimate has not settled yet", events)
    return events

  def check_asleep(self, t: float, v: float, gear: str, door_open: bool | None) -> tuple[str | None, list[str]]:
    """Call with carState about every SLEEP_CHECK_S while asleep. Returns (a qualifying stop's reason or None, events).
    On a qualifying stop the caller widens the estimate (estimator.note_qualifying_stop) and the mode becomes LEARN."""
    events: list[str] = []
    dt = 0. if self.last_check_t is None else t - self.last_check_t
    self.last_check_t = t
    why = None
    if v > STOP_SPEED_MPS:
      self.stopped_s, self.stop_qualified = 0., False
    else:
      self.stopped_s += dt
      if not self.stop_qualified:
        if gear == "park":
          why = "park"
        elif door_open:
          why = "door opened"
        elif self.stopped_s >= QUALIFYING_STOP_S:
          why = f"stopped {QUALIFYING_STOP_S:.0f} s"
    if why is not None:
      self.stop_qualified = True
      self._set(LEARN, t, f"qualifying stop ({why})", events)
      self.settled = False
    elif t - self.asleep_since >= FUEL_PERIOD_S:
      self._set(FUEL, t, f"{FUEL_PERIOD_S / 60.:.0f} min asleep: read the fuel gauge", events)
    return why, events

  def should_publish(self, t: float, snapshot: Snapshot) -> bool:
    if self.last_snapshot != snapshot or self.last_publish_t is None or t - self.last_publish_t >= HEARTBEAT_S:
      self.last_snapshot, self.last_publish_t = snapshot, t
      return True
    return False


def listening_step(runner, duty: DutyCycle, now_nanos: int, dt: float) -> list[str]:
  """One estimator step while listening, then the duty cycle's decision. Shared by the service and replay.py."""
  out = runner.step(now_nanos)
  events = list(out.events) + duty.after_step(now_nanos * 1e-9, dt, runner.last_inputs.v > STOP_SPEED_MPS, out.mass_kg,
                                              out.mass_std_kg, out.mass_updates)
  if not duty.listening:
    # going to sleep: watch for stops from where the estimator is, so a stop it already counted is not counted again
    duty.stopped_s, duty.stop_qualified = runner.estimator.stopped_s, runner.estimator.stop_qualified
  return events


def asleep_check(runner, duty: DutyCycle, t: float, car_state) -> list[str]:
  """One look at carState while asleep. On a qualifying stop the estimate is widened; when listening resumes the runner
  drops its inputs from before the sleep (the old carState would make the first step look like driving). Shared by the
  service and replay.py."""
  why, events = duty.check_asleep(t, car_state.vEgoRaw, str(car_state.gearShifter), car_state.doorOpen)
  if why is not None:
    events += runner.estimator.note_qualifying_stop(why, duty.stopped_s)
  if duty.listening:
    runner.wake(car_state)
  return events
