"""
Estimator tests on SYNTHETIC drives from tests/sim.py (a physics simulation with known weight, air drag and grades,
with sensor noise, signal steps and delays). No real drive data is used here.
"""
import copy
import math
from functools import cache

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.ramweightd import estimator as E
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import Inputs, LB_TO_KG, WeightGradeEstimator
from openpilot.sunnypilot.selfdrive.ramweightd.tests.sim import Phase, Sensors, Truck, TruckSim, flat, hilly, varied_schedule

BODY_PITCH = Truck().body_pitch_per_accel  # the estimator is told the simulated body pitch (calibrate.py finds it)


def run(sim: TruckSim, start_kg: float = 3500., start_std: float = 600., persisted=None, **kw):
  est = WeightGradeEstimator(start_kg, start_std, sim.truck.tank_gal, sim.truck.lb_per_gal, persisted,
                             body_pitch_per_accel=kw.pop("body_pitch", BODY_PITCH), **kw)
  outs = [est.step(inp) for inp in sim.inputs()]
  return est, outs


@cache
def varied(sensor: bool, seed: int = 3):
  sim = TruckSim(Truck(), Sensors(stability_sensor=sensor, transmission=sensor), hilly, seed=seed).run(varied_schedule(3))
  return sim, *run(sim)


@cache
def steady_flat():
  sim = TruckSim(Truck(), Sensors(), flat, seed=5).start_moving(25.).run([Phase("drive", 600, speed=25., cruise=True)])
  return sim, *run(sim)


@cache
def with_trailer():
  sched = varied_schedule(2) + [Phase("stop", 180, park=True, door=True, add_mass_kg=4500., add_drag=1.5)] + varied_schedule(2)[1:]
  sim = TruckSim(Truck(), Sensors(stability_sensor=False, transmission=False), hilly, seed=6).run(sched)
  return sim, *run(sim)


@cache
def with_fuel():
  sched = varied_schedule(4) + [Phase("stop", 120, park=True, refuel_pct=40.)] + varied_schedule(1)[1:]
  sim = TruckSim(Truck(fuel_pct=70.), Sensors(fuel_level=True), hilly, seed=7).run(sched)
  return sim, *run(sim)


class TestConvergence(OpenpilotTestCase):
  def test_converges_with_stability_sensor(self):
    sim, est, outs = varied(True)
    err = outs[-1].mass_kg / sim.log.mass_kg[-1] - 1.
    assert abs(err) < 0.02, f"weight error {100 * err:+.2f}%"
    assert abs(outs[-1].mass_kg - sim.log.mass_kg[-1]) < 3. * outs[-1].mass_std_kg
    assert abs(outs[-1].drag - sim.truck.drag) < 0.15
    assert outs[-1].grade_source == "sensorAndPitch"
    assert abs(outs[-1].sensor_offset - sim.sensors.sensor_offset) < 0.05

  def test_converges_with_comma_pitch_only(self):
    # today's signals: no stability sensor, no transmission message (converter-lock fallback rule)
    sim, est, outs = varied(False)
    err = outs[-1].mass_kg / sim.log.mass_kg[-1] - 1.
    assert abs(err) < 0.025, f"weight error {100 * err:+.2f}%"
    assert abs(outs[-1].mass_kg - sim.log.mass_kg[-1]) < 3. * outs[-1].mass_std_kg
    assert outs[-1].grade_source == "pitchOnly"
    assert outs[-1].converter_lock_rule == "ratioFallback"
    assert outs[-1].mass_updates > 300

  def test_grade_tracks_the_road(self):
    for sensor in (True, False):
      sim, est, outs = varied(sensor)
      errs = [outs[k].grade - math.tan(math.asin(sim.log.sine[10 * k])) for k in range(200, len(outs))]
      rms = math.sqrt(sum(e * e for e in errs) / len(errs))
      assert rms < 0.004, f"sensor={sensor}: grade error {rms:.4f}"

  def test_pitch_only_grade_is_less_certain(self):
    std_sensor = varied(True)[2][-1].grade_std
    std_pitch = varied(False)[2][-1].grade_std
    assert std_pitch >= E.PITCH_ONLY_STD_FLOOR
    assert std_pitch > std_sensor

  def test_uncorrected_body_pitch_biases_low(self):
    # documents why calibrate.py fits the body pitch: without it, the simulated squat reads as extra climb
    sim = varied(False)[0]
    _, outs = run(sim, body_pitch=0.)
    err = outs[-1].mass_kg / sim.log.mass_kg[-1] - 1.
    assert -0.04 < err < -0.005


class TestUncertainty(OpenpilotTestCase):
  def test_steady_flat_cruising_informs_only_the_combination(self):
    sim, est, outs = steady_flat()
    assert outs[-1].mass_updates > 500
    # the weight stays very uncertain, perfectly traded against the air drag
    assert outs[-1].mass_std_kg > 3000.
    assert outs[-1].mass_drag_correlation < -0.95
    # but the combination rolling resistance + air drag at this speed is pinned down
    m, c = est.mass.x
    predicted = m * E.G * E.ROLLING_RESISTANCE + c * 25. ** 2
    true = sim.truck.mass_kg * E.G * sim.truck.rolling_resistance + sim.truck.drag * 25. ** 2
    assert abs(predicted / true - 1.) < 0.03

  def test_varied_driving_narrows_the_weight(self):
    sim, est, outs = varied(False)
    assert outs[-1].mass_std_kg < 100.
    assert outs[-1].mass_drag_correlation > -0.8
    # narrowing comes with the varied driving: the first minute is still wide
    assert outs[600].mass_std_kg > 2. * outs[-1].mass_std_kg


class TestStops(OpenpilotTestCase):
  def test_no_mass_change_while_moving(self):
    # no process noise on the weight: its uncertainty never grows while moving (only at qualifying stops)
    sim, est, outs = with_trailer()
    for k in range(1, len(outs)):
      if sim.log.v[10 * k] > E.STOP_SPEED_MPS and outs[k].qualifying_stops == outs[k - 1].qualifying_stops:
        assert outs[k].mass_std_kg <= outs[k - 1].mass_std_kg + 1e-6, f"weight uncertainty grew while moving at sample {k}"
    f = E.MassDragFilter(4000., 1., [[100. ** 2, 0.], [0., 0.1 ** 2]])
    f.predict(10., moving=True)
    assert f.P[0][0] == 100. ** 2
    assert f.P[1][1] > 0.1 ** 2

  def test_qualifying_stop_keeps_value_and_widens(self):
    sim, est, outs = with_trailer()
    k = next(k for k in range(len(outs)) if outs[k].qualifying_stops == 1)
    before, after = outs[k - 1], outs[k]
    assert "park" in after.events[0]
    assert after.mass_kg == before.mass_kg
    assert after.mass_std_kg >= (E.CEILING_KG - before.mass_kg) / 2. - 1.
    assert after.drag_std >= E.DRAG_TRAILER_STD - 1e-9
    # 40,000 lb is within two standard deviations, so a heavy trailer can be learned
    assert before.mass_kg + 2. * after.mass_std_kg >= E.CEILING_KG - 1.

  def test_trailer_is_learned_after_the_stop(self):
    sim, est, outs = with_trailer()
    err = outs[-1].mass_kg / sim.log.mass_kg[-1] - 1.
    assert sim.log.mass_kg[-1] > 8000.
    assert abs(err) < 0.03, f"weight error with trailer {100 * err:+.2f}%"
    assert abs(outs[-1].drag - sim.log.drag[-1]) < 0.4

  def _stop_inputs(self, seconds: float, gear: str = "drive", door_at: float | None = None):
    inputs = [Inputs(t=0.1 * i, v=20., gear="drive") for i in range(50)]
    t = inputs[-1].t
    for i in range(int(seconds * 10)):
      t += 0.1
      inputs.append(Inputs(t=t, v=0., gear=gear, door_open=door_at is not None and abs(i * 0.1 - door_at) < 1.))
    return inputs

  def _stops(self, inputs) -> int:
    est = WeightGradeEstimator(4000., 100., 32., 7.1)
    return [est.step(i) for i in inputs][-1].qualifying_stops

  def test_which_stops_qualify(self):
    assert self._stops(self._stop_inputs(30.)) == 0                  # a short stop at a light
    assert self._stops(self._stop_inputs(100.)) == 0                 # a long red light
    assert self._stops(self._stop_inputs(E.QUALIFYING_STOP_S + 1.)) == 1
    assert self._stops(self._stop_inputs(5., gear="park")) == 1
    assert self._stops(self._stop_inputs(20., door_at=10.)) == 1
    assert self._stops(self._stop_inputs(400., gear="park")) == 1    # once per stop

  def test_drive_start_widens(self):
    est = WeightGradeEstimator(4000., 50., 32., 7.1)
    assert est.mass.mass == 4000.
    assert est.mass.mass_std >= (E.CEILING_KG - 4000.) / 2. - 1.

  def test_widening_without_trailer(self):
    # once a verified trailer signal says "no trailer", only cargo and passengers can change
    f = E.MassDragFilter(4000., 1., [[50. ** 2, 0.], [0., 0.1 ** 2]])
    f.widen(trailer_possible=False)
    assert abs(f.mass_std - E.PAYLOAD_STD_KG) < 1e-6
    assert abs(f.drag_std - 0.1) < 1e-9


class TestPersistence(OpenpilotTestCase):
  def test_state_round_trip_and_widening(self):
    sim, est, outs = varied(False)
    state = est.state()
    assert E.valid_persisted_state(state)
    assert abs(state["mass_kg"] - outs[-1].mass_kg) < 0.1
    nxt = WeightGradeEstimator(3000., 600., 32., 7.1, persisted=state)
    assert abs(nxt.mass.mass - state["mass_kg"]) < 1e-6         # starts from the kept estimate
    assert abs(nxt.mass.drag - state["drag"]) < 1e-6
    assert nxt.mass.mass_std >= (E.CEILING_KG - state["mass_kg"]) / 2. - 1.  # widened like a qualifying stop
    assert nxt.state()["mass_std_kg"] == state["mass_std_kg"]   # nothing learned yet: keeps the learned uncertainty

  def test_saved_uncertainty_is_the_learned_one(self):
    # every drive ends in park; the kept estimate must not be the widened one
    est = copy.deepcopy(with_trailer()[1])
    t = est.t_prev
    for i in range(1, 40):
      est.step(Inputs(t=t + 0.1 * i, v=0., gear="drive" if i < 20 else "park"))
    assert est.mass.mass_std > 4000.  # widened at the park stop
    assert est.state()["mass_std_kg"] < 1000.

  def test_invalid_states_are_ignored(self):
    good = {"version": E.STATE_VERSION, "mass_kg": 4000., "mass_std_kg": 100., "drag": 1., "drag_std": 0.1, "mass_drag_cov": 0.}
    assert E.valid_persisted_state(good)
    for bad in (None, [], {}, {**good, "version": 0}, {**good, "mass_kg": float("nan")}, {**good, "mass_kg": 100.},
                {**good, "drag": -1.}, {**good, "mass_std_kg": 0.}, {**good, "mass_drag_cov": 1000.}, {**good, "drag": "x"}):
      assert not E.valid_persisted_state(bad), bad


class TestFuel(OpenpilotTestCase):
  def test_fuel_burned(self):
    sim, est, outs = with_fuel()
    k_refuel = next(k for k in range(len(outs)) if any("fuel change" in e for e in outs[k].events))
    # counting starts after the first FUEL_WARMUP_S of moving (the first unbiased level)
    t0 = int(E.FUEL_WARMUP_S * 100) + 500
    i_refuel = 10 * k_refuel
    i_last_move = max(i for i in range(i_refuel) if sim.log.v[i] > 1.)
    true_burned = (sim.log.mass_kg[t0] - sim.log.mass_kg[i_last_move])
    assert true_burned > 2.
    assert abs(outs[k_refuel].fuel_burned_kg - true_burned) < 0.3 * true_burned + 1.

  def test_refuel_at_a_stop(self):
    sim, est, outs = with_fuel()
    events = [e for o in outs for e in o.events if "fuel change" in e]
    assert len(events) == 1, events
    k = next(k for k in range(len(outs)) if outs[k].events and "fuel change" in outs[k].events[-1])
    added_lb = (outs[k].mass_kg - outs[k - 1].mass_kg) / LB_TO_KG
    m = sim.log.mass_kg
    true_added_lb = max(m[i + 1] - m[i] for i in range(len(m) - 1)) / LB_TO_KG
    assert true_added_lb > 50.
    assert abs(added_lb - true_added_lb) < 15., (added_lb, true_added_lb)
    # the fuel burned so far is kept; the weight follows the true weight after refueling
    assert outs[-1].fuel_burned_kg >= outs[k].fuel_burned_kg
    assert abs(outs[-1].mass_kg / sim.log.mass_kg[-1] - 1.) < 0.02

  def test_fuel_inactive_without_signal(self):
    sim, est, outs = varied(False)
    assert outs[-1].fuel_burned_kg == 0.
    assert outs[-1].fuel_level_pct is None

  def test_increase_while_moving_is_not_refueling(self):
    f = E.FuelTracker(32., 7.1)
    added = 0.
    for i in range(int(400 / 0.1)):
      added += f.step(0.1, 50. + (10. if i > 3500 else 0.), True, 0.)
    assert added == 0.
    assert f.level_pct is not None and f.level_pct < 51.


class TestRules(OpenpilotTestCase):
  def clean_inputs(self, **kw) -> Inputs:
    base = dict(t=0., v=20., gear="drive", brake_pedal=False, brake_pressure=0., accel_pedal_pct=20., acc_active=False,
                acc_torque_request_nm=0., engine_rpm=1500., engine_torque_nm=400., shift_pending=False, exhaust_brake=False,
                traction_control=False, lateral_accel=0.)
    base.update(kw)
    return Inputs(**base)

  def reasons(self, locked=True, rule="signal", steady=True, grade_std=0.005, **kw):
    return E.clean_reasons(self.clean_inputs(**kw), locked, rule, steady, grade_std)

  def test_clean(self):
    assert self.reasons() == []
    # the factory cruise asking for torque counts as pulling
    assert self.reasons(accel_pedal_pct=0., acc_active=True, acc_torque_request_nm=300.) == []

  def test_each_rule(self):
    assert "speed below 5 m/s" in self.reasons(v=4.)
    assert "not in a forward gear" in self.reasons(gear="reverse")
    assert "converter not locked" in self.reasons(locked=False)
    assert "converter state unknown" in self.reasons(rule="none", locked=False)
    assert "service brake on" in self.reasons(brake_pedal=True)
    assert "service brake on" in self.reasons(brake_pressure=E.BRAKE_PRESSURE_ON + 1.)
    assert "exhaust brake on" in self.reasons(exhaust_brake=True)
    assert "shift in progress" in self.reasons(shift_pending=True)
    assert "possible shift (ratio not steady)" in self.reasons(shift_pending=None, steady=False)
    assert "engine not pulling" in self.reasons(engine_torque_nm=20.)
    assert "engine not pulling" in self.reasons(accel_pedal_pct=0.)
    assert "engine not pulling" in self.reasons(accel_pedal_pct=None, acc_active=True, acc_torque_request_nm=0.)
    assert "traction control active" in self.reasons(traction_control=True)
    assert "turning" in self.reasons(lateral_accel=2.5)
    assert "grade unknown" in self.reasons(grade_std=0.05)
    assert "engine signals missing" in self.reasons(engine_rpm=None)

  def test_ratio_fallback(self):
    r = E.RatioSteadiness()
    steady = [r.update(0.1 * i, 1500. + (i % 3), 400., 20.) for i in range(25)]
    assert steady[-1] == (True, True)
    assert steady[10] == (True, False)  # not yet two seconds
    r = E.RatioSteadiness()
    slipping = [r.update(0.1 * i, 1500. * (1. + 0.03 * math.sin(i / 3.)), 400., 20.) for i in range(40)]
    assert slipping[-1] == (True, False)
    assert r.update(5., None, 400., 20.) == (False, False)
    # without wheel speeds the truck speed stands in
    r = E.RatioSteadiness()
    assert [r.update(0.1 * i, 1500., None, 20.) for i in range(25)][-1] == (True, True)
