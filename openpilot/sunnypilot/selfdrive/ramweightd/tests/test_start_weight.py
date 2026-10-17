"""
Starting weight and configuration tests.

- TestShippedTable uses the shipped ram_hd_weights.json (generated from the research table built from Stellantis
  documents) and VINs that are FAKE: 17 characters built around a configuration's model characters with a made-up
  serial number and a computed check digit, never a real truck's VIN.
- The other classes use SYNTHETIC_TABLE, whose VIN rules, part numbers and curb weights are made up for the test.
"""
import copy
import json
import os

from opendbc.car.structs import car
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.ramweightd import make_weight_table as M
from openpilot.sunnypilot.selfdrive.ramweightd import start_weight as S
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import LB_TO_KG
from openpilot.sunnypilot.selfdrive.ramweightd.tests.sim import vin_with_check_digit

ERIC = dict(S.ERIC_CONFIGURATION)
FULL_RESEARCH_TABLE = os.path.expanduser("~/.claude/jobs/f744baa9/tmp/research/ram_hd_specs.json")  # not committed
ALLOWANCES_LB = S.OPTIONS_ALLOWANCE_LB + S.DRIVER_ALLOWANCE_LB


def config(**kw) -> dict:
  c = {**ERIC, "bed_length_in": 76.3, "curb_weight_lb": None, "curb_weight_kind": "base", "fuel_tank_gal": None, "source": "test",
       "notes": "synthetic"}
  c.update(kw)
  return c


SYNTHETIC_TABLE = {
  "schema": 1, "retrieved": "test", "sources": [{"id": "test", "title": "synthetic test data", "url": ""}],
  "diesel_lb_per_gal": 7.1, "gasoline_lb_per_gal": 6.0,
  "configs": [
    config(curb_weight_lb=7000., fuel_tank_gal=32.),                                  # Eric's configuration
    config(bed="long", bed_length_in=98.3, curb_weight_lb=7200., fuel_tank_gal=32.),
    config(series="3500", rear_wheels="dual", bed="long", curb_weight_lb=7900., fuel_tank_gal=50.),
    config(engine="hemi_6.4", transmission="8HP75", curb_weight_lb=6400., fuel_tank_gal=32.),
    config(drive="4x2", curb_weight_lb=6800.),
  ],
  "vin": {
    "model_year_codes": {"N": 2022, "P": 2023},
    "rules": [  # synthetic positions and codes
      {"model_years": [2022, 2023], "position": 5, "code": "X", "meaning": {"drive": "4x4"}, "source": "test"},
      {"model_years": [2022, 2023], "position": 5, "code": "Y", "meaning": {"drive": "4x2"}, "source": "test"},
      {"model_years": [2022], "position": 6, "code": "2", "meaning": {"series": "2500", "rear_wheels": "single"}, "source": "test"},
      {"model_years": [2022], "position": 6, "code": "3", "meaning": {"series": "3500", "rear_wheels": "dual"}, "source": "test"},
      {"model_years": [2022], "position": 7, "code": "C", "meaning": {"cab": "crew"}, "source": "test"},
      {"model_years": [2022], "position": 8, "code": "D", "meaning": {"engine": "cummins_so"}, "source": "test"},
      {"model_years": [2022], "position": 8, "code": "H", "meaning": {"engine": "hemi_6.4", "transmission": "8HP75"}, "source": "test"},
      {"model_years": [2023], "position": 7, "code": "C", "meaning": {"cab": "mega"}, "source": "test"},
      {"model_years": [2022], "position": 4, "code": "Z", "meaning": {"cab": "mega"}, "source": "test"},
    ],
  },
  "engine_parts": [{"part_prefix_or_number": "11112222", "engine": "cummins_so", "transmission": "68RFE", "source": "test"},
                   {"part_prefix_or_number": "33334444", "engine": "cummins_ho", "source": "test"}],
  "transmission_parts": [{"part_prefix_or_number": "55556666", "transmission": "AS69RC", "source": "test"}],
}


def table(data=None) -> S.WeightTable:
  return S.parse_weight_table(copy.deepcopy(data or SYNTHETIC_TABLE))


def vin(year_code="N", p4="A", p5="X", p6="2", p7="C", p8="D") -> str:
  """A FAKE VIN for SYNTHETIC_TABLE."""
  return vin_with_check_digit(f"1AB{p4}{p5}{p6}{p7}{p8}0{year_code}G000001")


def fake_vin(model_chars: str, year_code: str = "N") -> str:
  """A FAKE VIN around real model characters (positions 1-8), for the shipped table: made-up plant and serial."""
  return vin_with_check_digit(f"{model_chars}0{year_code}G000000")


def car_params(vin_text: str = "", fw: list | None = None, mass: float = 3541.):
  CP = car.CarParams.new_message()
  CP.brand = "chrysler"
  CP.carFingerprint = "RAM_HD_5TH_GEN"
  CP.carVin = vin_text
  CP.mass = mass
  fws = fw or []
  CP.init("carFw", len(fws))
  for i, (ecu, version) in enumerate(fws):
    CP.carFw[i].ecu = ecu
    CP.carFw[i].fwVersion = version
  return CP


def lb(start: S.StartWeight) -> float:
  return start.mass_kg / LB_TO_KG


class TestShippedTable(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.t = S.load_weight_table()
    assert self.t is not None

  def eric_rows(self):
    return [c for c in self.t.configs if all(c[k] == v for k, v in ERIC.items()) and c["package"] is None]

  def test_table_facts(self):
    assert not self.t.placeholder and self.t.retrieved == "2026-10-16"
    assert self.t.diesel_lb_per_gal == 7.079 and self.t.gasoline_lb_per_gal == 6.25
    rows = {c["box_variant"]: c for c in self.eric_rows()}
    assert rows["standard"]["curb_weight_lb"] == 7511 and rows["rambox"]["curb_weight_lb"] == 7640
    assert rows["standard"]["fuel_tank_gal"] == 31 and rows["standard"]["fuel_tank_optional_gal"] is None
    # the options allowance constant is Eric's empty weight minus his configuration's published weight
    assert S.ERIC_EMPTY_LB - rows["standard"]["curb_weight_lb"] == S.OPTIONS_ALLOWANCE_LB == 529.

  def test_eric_model_characters(self):
    # Eric's VIN model characters 3C6 UR5GL, model year N, inside a FAKE VIN
    v = fake_vin("3C6UR5GL")
    fields, conflicts = S.decode_vin(v, self.t)
    assert not conflicts
    assert {k: fields[k] for k in ("model_year", "series", "rear_wheels", "drive", "cab", "bed")} == \
      {"model_year": 2022, "series": "2500", "rear_wheels": "single", "drive": "4x4", "cab": "crew", "bed": "short"}
    assert fields["engine"] == frozenset({"cummins_so", "cummins_ho"})  # the VIN only says Cummins
    for fw in ([], [("engine", b"52370231AK")]):
      conf = S.identify_configuration(car_params(v, fw), self.t)
      assert {k: conf.fields[k] for k in ERIC} == ERIC  # the table knows a 2500 Cummins is the standard output and 68RFE
      assert conf.text == "2022 Ram 2500, single rear wheels, Crew Cab, short bed, four-wheel drive, " + \
        "Cummins 6.7 L diesel, standard output, 68RFE six-speed automatic"
      assert v not in conf.text and "000000" not in conf.text

  def test_eric_start_weight(self):
    conf = S.identify_configuration(car_params(fake_vin("3C6UR5GL")), self.t)
    full = S.choose_start_weight(conf, self.t, 0., 100., 3541.)
    assert full.source == "table" and full.tank_gal == 31. and full.lb_per_gal == 7.079
    assert abs(lb(full) - (7511. + ALLOWANCES_LB)) < 1e-6  # 8,290 lb with a full tank
    unknown = S.choose_start_weight(conf, self.t, 0., None, 3541.)
    assert abs(lb(unknown) - (8290. - 0.5 * 31. * 7.079)) < 1e-6
    # the VIN cannot tell a RamBox: the standard box is used and its extra 129 lb widens the uncertainty
    assert "RamBox would add 129 lb" in full.detail
    assert full.std_kg > S.TABLE_STD_KG * 1.05
    setting = S.choose_start_weight(conf, self.t, 8400., None, 3541.)
    assert setting.source == "setting" and abs(lb(setting) - (8400. - 0.5 * 31. * 7.079)) < 1e-6

  def test_3500_cummins_gearbox(self):
    v = fake_vin("3C63R3GL")  # 3500 single rear wheels, Crew Cab long bed, four-wheel drive, Cummins
    conf = S.identify_configuration(car_params(v), self.t)
    assert conf.fields["series"] == "3500" and conf.fields["bed"] == "long"
    assert conf.fields["engine"] == frozenset({"cummins_so", "cummins_ho"}) and "transmission" not in conf.fields
    assert "Cummins 6.7 L diesel, standard or high output; not identified: engine, transmission" in conf.text
    avg = S.choose_start_weight(conf, self.t, 0., 100., 3541.)
    assert avg.source == "table" and "average of the standard and high output Cummins" in avg.detail
    # Crew Cab 8 ft box, 2022: the optional 50 gal tank is assumed (18 gal more than the published 32 gal tank)
    extra = 18. * 7.079
    assert avg.tank_gal == 50. and abs(lb(avg) - ((7974. + 7809.) / 2. + extra + ALLOWANCES_LB)) < 1e-6
    # the engine part number decides the output when the table knows it (here a made-up part number)
    t = S.load_weight_table()
    t.part_rules.append(("engine", {"part_prefix_or_number": "99990000", "engine": "cummins_ho"}))
    conf_ho = S.identify_configuration(car_params(v, [("engine", b"99990000AA")]), t)
    assert conf_ho.fields["engine"] == "cummins_ho" and conf_ho.fields["transmission"] == "AS69RC"
    ho = S.choose_start_weight(conf_ho, t, 0., 100., 3541.)
    assert abs(lb(ho) - (7974. + extra + ALLOWANCES_LB)) < 1e-6
    assert avg.std_kg > ho.std_kg

  def test_fuel_tank_rule(self):
    crew_long_2022 = S.identify_configuration(car_params(fake_vin("3C63R3GL")), self.t)
    assumed = S.choose_start_weight(crew_long_2022, self.t, 0., 100., 3541.)
    assert assumed.tank_gal == 50. and "assumed optional 50 gal tank (+127 lb" in assumed.detail
    # the truck reports a standard tank: no extra fuel, and less uncertainty
    reported = S.choose_start_weight(crew_long_2022, self.t, 0., 100., 3541., tank_gal_reported=32.)
    assert reported.tank_gal == 32. and abs(lb(assumed) - lb(reported) - 18. * 7.079) < 1e-6
    assert reported.std_kg < assumed.std_kg
    # half a tank of unknown level is half of the assumed 50 gal tank
    half = S.choose_start_weight(crew_long_2022, self.t, 0., None, 3541.)
    assert abs(lb(assumed) - lb(half) - 25. * 7.079) < 1e-6
    # 2019 had no optional tank
    crew_long_2019 = S.identify_configuration(car_params(fake_vin("3C63R3GL", "K")), self.t)
    s2019 = S.choose_start_weight(crew_long_2019, self.t, 0., 100., 3541.)
    assert s2019.tank_gal == 32. and "optional" not in s2019.detail
    # Eric's 6 ft 4 in box has no optional tank; a Regular Cab diesel has 28 gal
    assert S.choose_start_weight(S.identify_configuration(car_params(fake_vin("3C6UR5GL")), self.t), self.t, 0., 100., 3541.).tank_gal == 31.
    regular = S.identify_configuration(car_params(fake_vin("3C6UR5AL")), self.t)
    assert regular.fields["cab"] == "regular" and regular.fields["bed"] == "long"
    assert S.choose_start_weight(regular, self.t, 0., 100., 3541.).tank_gal == 28.

  def test_bed_from_vin_and_packages(self):
    mega = S.identify_configuration(car_params(fake_vin("3C6UR5ML")), self.t)
    assert mega.fields["cab"] == "mega" and mega.fields["bed"] == "short"
    assert S.choose_start_weight(mega, self.t, 0., 100., 3541.).source == "table"
    pw = S.identify_configuration(car_params(fake_vin("3C6UR5EJ")), self.t)  # Power Wagon, 6.4 L HEMI
    assert pw.fields["package"] == "power_wagon" and "Power Wagon" in pw.text
    s = S.choose_start_weight(pw, self.t, 0., 100., 3541.)
    assert s.source == "table" and s.lb_per_gal == 6.25
    # an unknown model code: the bed is not known, every other field is
    conf = S.Configuration({k: v for k, v in ERIC.items() if k != "bed"} | {"package": None})
    avg = S.choose_start_weight(conf, self.t, 0., 100., 3541.)
    assert avg.source == "tableBedAverage" and avg.std_kg > S.choose_start_weight(S.Configuration(dict(ERIC)), self.t, 0., 100., 3541.).std_kg


class TestTableLayout(OpenpilotTestCase):
  def test_malformed_tables_are_rejected(self):
    for change in (
      lambda d: d.update(schema=2),
      lambda d: d["configs"][0].update(cab="quad"),
      lambda d: d["configs"][0].pop("bed"),
      lambda d: d["configs"][0].update(curb_weight_lb=-5),
      lambda d: d["configs"][0].update(box_variant="flatbed"),
      lambda d: d["vin"]["rules"].append({"position": 30, "code": "A", "meaning": {}}),
      lambda d: d["vin"]["rules"].append({"position": "5-7", "code": "AB", "meaning": {}}),
      lambda d: d["vin"]["rules"].append({"position": 3, "code": "A", "meaning": {"color": "red"}}),
      lambda d: d["vin"]["rules"].append({"position": 3, "code": "A", "meaning": {"engine": ["cummins_so", "v12"]}}),
      lambda d: d["engine_parts"].append({"engine": "cummins_so"}),
    ):
      d = copy.deepcopy(SYNTHETIC_TABLE)
      change(d)
      with self.assertRaises(ValueError):
        S.parse_weight_table(d)

  def test_missing_file_falls_back(self):
    assert S.load_weight_table("/nonexistent/ram_hd_weights.json") is None


class TestVin(OpenpilotTestCase):
  def test_check_digit(self):
    v = vin()
    assert S.vin_is_valid(v)
    assert not S.vin_is_valid(v[:8] + ("1" if v[8] != "1" else "2") + v[9:])
    assert not S.vin_is_valid("0" * 17)
    assert not S.vin_is_valid("")
    assert not S.vin_is_valid(v[:16])
    assert not S.vin_is_valid(v[:5] + "I" + v[6:])  # I, O and Q are never used

  def test_rules(self):
    fields, conflicts = S.decode_vin(vin(), table())
    assert fields == {"model_year": 2022, "drive": "4x4", "series": "2500", "rear_wheels": "single", "cab": "crew", "engine": "cummins_so"}
    assert not conflicts
    # rules apply only to their model years
    fields, _ = S.decode_vin(vin(year_code="P"), table())
    assert fields == {"model_year": 2023, "drive": "4x4", "cab": "mega"}
    # two rules disagreeing about one field: the field is unknown and reported as conflicting
    fields, conflicts = S.decode_vin(vin(p4="Z"), table())
    assert "cab" not in fields and conflicts == {"cab"}
    # an unknown model-year code: nothing is decoded
    assert S.decode_vin(vin(year_code="A"), table()) == ({}, set())

  def test_configuration_text_never_contains_the_vin(self):
    v = vin()
    conf = S.identify_configuration(car_params(v), table())
    assert v not in conf.text and v[-6:] not in conf.text and v[:8] not in conf.text
    assert conf.text.startswith("2022 Ram 2500, single rear wheels, Crew Cab, four-wheel drive, Cummins 6.7 L diesel, standard output")
    assert "not identified: bed" in conf.text

  def test_part_numbers(self):
    CP = car_params(vin(), fw=[("engine", b"11112222AK"), ("abs", b"11112222AK"), ("transmission", b"99999999AA")])
    conf = S.identify_configuration(CP, table())
    assert conf.fields["engine"] == "cummins_so" and conf.fields["transmission"] == "68RFE"
    CP = car_params(vin(), fw=[("engine", b"33334444AB "), ("transmission", b"55556666AA")])
    conf = S.identify_configuration(CP, table())
    assert "engine" not in conf.fields and "engine" in conf.conflicts  # VIN says standard output, part says high output
    assert "conflicting: engine" in conf.text


class TestStartWeight(OpenpilotTestCase):
  def start(self, fields, base_lb=0., fuel=None, data=None):
    t = table(data) if data is not False else None
    return S.choose_start_weight(S.Configuration(fields), t, base_lb, fuel, 3541.)

  def test_setting(self):
    tank_lb = 32. * 7.1
    s = self.start(ERIC, base_lb=8400., fuel=100.)
    assert s.source == "setting" and abs(lb(s) - 8400.) < 1e-6
    s = self.start(ERIC, base_lb=8400., fuel=25.)
    assert abs(lb(s) - (8400. - 0.75 * tank_lb)) < 1e-6
    s_unknown = self.start(ERIC, base_lb=8400.)
    assert abs(lb(s_unknown) - (8400. - 0.5 * tank_lb)) < 1e-6
    assert s_unknown.std_kg > self.start(ERIC, base_lb=8400., fuel=50.).std_kg
    # the setting wins even when the configuration is unknown or there is no table
    assert self.start({}, base_lb=8400., data=False).source == "setting"

  def test_table(self):
    # published 7,000 lb + options allowance 529 lb + 250 lb, full tank
    s = self.start({**ERIC}, fuel=100.)
    assert s.source == "table" and abs(lb(s) - (7000. + ALLOWANCES_LB)) < 1e-6
    assert s.tank_gal == 32.
    # a 3500 dual-rear-wheel long bed: its 50-gallon tank half empty
    f = {**ERIC, "series": "3500", "rear_wheels": "dual", "bed": "long"}
    s = self.start(f, fuel=50.)
    assert s.source == "table" and s.tank_gal == 50.
    assert abs(lb(s) - (7900. + ALLOWANCES_LB - 0.5 * 50. * 7.1)) < 1e-6
    # a diesel long bed starts heavier than a gas short bed
    gas = self.start({**ERIC, "engine": "hemi_6.4", "transmission": "8HP75"}, fuel=100.)
    assert gas.source == "table" and gas.lb_per_gal == 6.0
    assert s.mass_kg > gas.mass_kg

  def test_bed_average(self):
    f = {k: v for k, v in ERIC.items() if k != "bed"}
    s = self.start(f, fuel=100.)
    assert s.source == "tableBedAverage"
    assert abs(lb(s) - (7100. + ALLOWANCES_LB)) < 1e-6
    assert s.std_kg > self.start(ERIC, fuel=100.).std_kg

  def test_fallback(self):
    # no curb weight in the table for this configuration
    d = copy.deepcopy(SYNTHETIC_TABLE)
    d["configs"][0]["curb_weight_lb"] = None
    s = self.start(dict(ERIC), data=d)
    assert s.source == "fallback" and s.mass_kg == 3541. and s.std_kg == S.FALLBACK_STD_KG
    # unknown configuration (two fields missing), or no table at all
    assert self.start({k: v for k, v in ERIC.items() if k not in ("bed", "drive")}).source == "fallback"
    assert self.start({}).source == "fallback"
    assert self.start(ERIC, data=False).source == "fallback"

  def test_from_car_params(self):
    CP = car_params(vin(), fw=[("engine", b"11112222AK")])
    t = table()
    conf = S.identify_configuration(CP, t)
    assert {k: conf.fields[k] for k in conf.fields if k != "package"} == {k: v for k, v in ERIC.items() if k != "bed"}
    assert S.choose_start_weight(conf, t, 0., None, CP.mass).source == "tableBedAverage"


FULL_SAMPLE = {
  "schema": 1, "retrieved": "2026-01-01",
  "sources": [{"id": "chart", "title": "a chart", "url": "u1", "publisher": "x"}, {"id": "vin_2022", "title": "vin guide", "url": "u2"},
              {"id": "vin_2023", "title": "vin guide", "url": "u3"}, {"id": "unused", "title": "unused", "url": "u4"},
              {"id": "tedb_ed40", "title": "energy data book", "url": "u5"}],
  "diesel_lb_per_gal": 7.079, "gasoline_lb_per_gal": 6.25,
  "configs": [{**ERIC, "bed_length_in": 76.3, "box_variant": "standard", "package": None, "recommended_curb_weight_lb": None,
               "curb_weight_lb": None, "secondary_curb_weight_lb": 7500, "curb_weight_kind": "base", "fuel_tank_gal": 31,
               "fuel_tank_optional_gal": None, "source": "chart", "trim_basis": "dropped", "variants": []}],
  "vin": {"model_year_codes": {"2022": "N", "2023": "P"}, "rules": [
    {"model_years": [2022, 2023], "position": "5-7", "code": "R5E", "meaning": {"series": "2500", "rear_wheels": "single", "drive": "4x4",
     "cab": "crew", "bed": "short", "bed_length_in": 76.3, "trim": "Power Wagon (2023-2024: Power Wagon / Rebel)"}, "source": "vin_2022..2023"},
    {"model_years": [2022], "position": "5-7", "code": "R5G", "meaning": {"series": "2500", "rear_wheels": "single", "drive": "4x4",
     "cab": "crew", "bed": "short", "trim": "Longhorn"}, "source": "vin_2022"},
    {"model_years": [2022], "position": 8, "code": "L", "meaning": {"engine": "cummins", "description": "6.7 L"}, "source": "vin_2022"},
    {"model_years": [2022], "position": 8, "code": "J", "meaning": {"engine": "hemi_6.4", "implies_transmission": "8HP75"}, "source": "vin_2022"},
    {"model_years": [2022], "position": 4, "code": "F", "meaning": {"gvwr_lb_min": 9001}, "source": "vin_2022"},
  ]},
  "engine_parts": [{"part_number": "52370231", "engine": "cummins_so", "transmission": "68RFE", "series": "2500", "source": "chart"},
                   {"part_number": "52370131", "engine": "unknown (likely Cummins)", "series": "unknown", "source": "chart"}],
}


class TestGenerator(OpenpilotTestCase):
  def test_build(self):
    slim = json.loads(M.dumps(M.build(copy.deepcopy(FULL_SAMPLE))))
    S.parse_weight_table(slim)
    assert slim["retrieved"] == "2026-01-01" and not slim["placeholder"]
    assert slim["vin"]["model_year_codes"] == {"N": 2022, "P": 2023}
    row = slim["configs"][0]
    assert row["recommended_curb_weight_lb"] == 7500 and "trim_basis" not in row and "variants" not in row
    rules = slim["vin"]["rules"]
    pw = [r for r in rules if r["code"] == "R5E"]
    assert [(r["model_years"], r["meaning"]["package"]) for r in pw] == [([2022], "power_wagon"), ([2023], ["power_wagon", "rebel"])]
    assert all("trim" not in r["meaning"] and "bed_length_in" not in r["meaning"] for r in rules)
    assert next(r for r in rules if r["code"] == "R5G")["meaning"]["package"] is None
    assert next(r for r in rules if r["code"] == "L")["meaning"] == {"engine": ["cummins_so", "cummins_ho"]}
    assert next(r for r in rules if r["code"] == "J")["meaning"] == {"engine": "hemi_6.4", "transmission": "8HP75"}
    assert not any(r["position"] == 4 for r in rules)
    assert slim["engine_parts"] == [{"part_prefix_or_number": "52370231", "engine": "cummins_so", "transmission": "68RFE", "series": "2500",
                                     "source": "chart"}]
    assert {s["id"] for s in slim["sources"]} == {"chart", "vin_2022", "vin_2023", "tedb_ed40"}

  def test_shipped_table_is_regenerated_exactly(self):
    if not os.path.exists(FULL_RESEARCH_TABLE):
      self.skipTest("the full research table is not on this machine")
    with open(FULL_RESEARCH_TABLE) as f:
      expected = M.dumps(M.build(json.load(f)))
    with open(S.DEFAULT_TABLE_PATH) as f:
      assert f.read() == expected


class TestWeightFloor(OpenpilotTestCase):
  """The floor: empty weight minus the fuel not in the tank (the whole tank when the level is not known)."""

  def setUp(self):
    super().setUp()
    self.t = S.load_weight_table()
    self.eric = S.identify_configuration(car_params(fake_vin("3C6UR5GL")), self.t)  # FAKE VIN around Eric's model characters

  def floor_lb(self, conf, empty_lb, level, tank=31., density=7.079, table=True):
    f = S.weight_floor(conf, self.t if table else None, empty_lb, level, tank, density)
    return None if f.mass_kg is None else f.mass_kg / LB_TO_KG, f.source

  def test_setting_both_fuel_cases(self):
    full_tank = 31. * 7.079  # 219 lb
    assert abs(self.floor_lb(self.eric, 8040., 100.)[0] - 8040.) < 1e-6
    lb_unknown, source = self.floor_lb(self.eric, 8040., None)
    assert source == "setting" and abs(lb_unknown - (8040. - full_tank)) < 1e-6  # 7,821 lb: as if the tank were empty
    assert abs(self.floor_lb(self.eric, 8040., 50.)[0] - (8040. - 0.5 * full_tank)) < 1e-6
    # the setting works without any table or identified configuration
    assert abs(self.floor_lb(S.Configuration(), 8040., 100., table=False)[0] - 8040.) < 1e-6

  def test_table_without_options_allowance(self):
    lb_full, source = self.floor_lb(self.eric, 0., 100.)
    assert source == "table" and abs(lb_full - 7511.) < 1e-6  # Ram's published curb weight, standard box, no 529 lb
    assert abs(self.floor_lb(self.eric, 0., None)[0] - (7511. - 31. * 7.079)) < 1e-6
    # bed not identified: the lighter bed; 3500 Cummins output not identified: the lighter engine
    no_bed = S.Configuration({k: v for k, v in self.eric.fields.items() if k != "bed"})
    rows = [c for c in self.t.configs if all(c[k] == v for k, v in no_bed.fields.items()) and c["box_variant"] == "standard"]
    assert abs(self.floor_lb(no_bed, 0., 100.)[0] - min(c["curb_weight_lb"] for c in rows)) < 1e-6
    crew_long_3500 = S.identify_configuration(car_params(fake_vin("3C63R3GL")), self.t)
    assert abs(self.floor_lb(crew_long_3500, 0., 100.)[0] - 7809.) < 1e-6  # standard output, lighter than the high output's 7,974

  def test_no_floor_without_configuration(self):
    assert self.floor_lb(S.Configuration(), 0., 100.) == (None, "none")
    assert self.floor_lb(S.Configuration({"model_year": 2022}), 0., None) == (None, "none")
    assert self.floor_lb(self.eric, 0., 100., table=False) == (None, "none")
