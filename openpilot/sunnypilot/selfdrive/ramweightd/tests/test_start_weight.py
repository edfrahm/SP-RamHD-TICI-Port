"""
Starting weight and configuration tests. The VIN rules, part numbers and curb weights in SYNTHETIC_TABLE are made up
for the test (they are not Ram's codes or weights); the placeholder table is the shipped ram_hd_weights.json.
"""
import copy

from opendbc.car.structs import car
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.ramweightd import start_weight as S
from openpilot.sunnypilot.selfdrive.ramweightd.estimator import LB_TO_KG
from openpilot.sunnypilot.selfdrive.ramweightd.tests.sim import vin_with_check_digit

ERIC = dict(S.ERIC_CONFIGURATION)


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
  return vin_with_check_digit(f"1AB{p4}{p5}{p6}{p7}{p8}0{year_code}G000001")


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


class TestTable(OpenpilotTestCase):
  def test_placeholder_loads(self):
    t = S.load_weight_table()
    assert t is not None and t.placeholder
    eric = [c for c in t.configs if all(c[k] == v for k, v in ERIC.items())]
    assert len(eric) == 1 and eric[0]["curb_weight_lb"] is None
    assert t.diesel_lb_per_gal == 7.1 and t.gasoline_lb_per_gal == 6.0
    assert t.model_year_codes["N"] == 2022

  def test_malformed_tables_are_rejected(self):
    for change in (
      lambda d: d.update(schema=2),
      lambda d: d["configs"][0].update(cab="quad"),
      lambda d: d["configs"][0].pop("bed"),
      lambda d: d["configs"][0].update(curb_weight_lb=-5),
      lambda d: d["vin"]["rules"].append({"position": 30, "code": "A", "meaning": {}}),
      lambda d: d["vin"]["rules"].append({"position": 3, "code": "A", "meaning": {"color": "red"}}),
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

  def test_placeholder_table_gives_only_the_model_year(self):
    fields, conflicts = S.decode_vin(vin(), S.load_weight_table())
    assert fields == {"model_year": 2022} and not conflicts
    assert S.decode_vin(vin(year_code="A"), S.load_weight_table())[0] == {}

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

  def test_configuration_text_never_contains_the_vin(self):
    v = vin()
    conf = S.identify_configuration(car_params(v), table())
    text = conf.text
    assert v not in text and v[-6:] not in text and v[:8] not in text
    assert text.startswith("2022 Ram 2500, single rear wheels, Crew Cab, four-wheel drive, Cummins 6.7 L diesel, standard output")
    assert "not identified: bed, transmission" in text

  def test_part_numbers(self):
    CP = car_params(vin(), fw=[("engine", b"11112222AK"), ("abs", b"11112222AK"), ("transmission", b"99999999AA")])
    conf = S.identify_configuration(CP, table())
    assert conf.fields["engine"] == "cummins_so" and conf.fields["transmission"] == "68RFE"
    CP = car_params(vin(), fw=[("engine", b"33334444AB "), ("transmission", b"55556666AA")])
    conf = S.identify_configuration(CP, table())
    assert "engine" not in conf.fields and "engine" in conf.conflicts  # VIN says standard output, part says high output
    assert conf.fields["transmission"] == "AS69RC"
    assert "conflicting: engine" in conf.text


class TestStartWeight(OpenpilotTestCase):
  def start(self, fields, base_lb=0., fuel=None, data=None):
    t = table(data) if data is not False else None
    return S.choose_start_weight(S.Configuration(fields), t, base_lb, fuel, 3541.)

  def test_setting(self):
    tank_lb = 32. * 7.1
    s = self.start(ERIC, base_lb=8400., fuel=100.)
    assert s.source == "setting" and abs(s.mass_kg / LB_TO_KG - 8400.) < 1e-6
    s = self.start(ERIC, base_lb=8400., fuel=25.)
    assert abs(s.mass_kg / LB_TO_KG - (8400. - 0.75 * tank_lb)) < 1e-6
    s_unknown = self.start(ERIC, base_lb=8400.)
    assert abs(s_unknown.mass_kg / LB_TO_KG - (8400. - 0.5 * tank_lb)) < 1e-6
    assert s_unknown.std_kg > self.start(ERIC, base_lb=8400., fuel=50.).std_kg
    # the setting wins even when the configuration is unknown or there is no table
    assert self.start({}, base_lb=8400., data=False).source == "setting"

  def test_table(self):
    # Eric's configuration: published 7,000 lb + allowance (8,040 - 7,000) + 250 = 8,290 lb with a full tank
    s = self.start({**ERIC}, fuel=100.)
    assert s.source == "table" and abs(s.mass_kg / LB_TO_KG - 8290.) < 1e-6
    assert s.tank_gal == 32.
    # a 3500 dual-rear-wheel long bed: 7,900 + 1,040 + 250, its 50-gallon tank half empty
    f = {**ERIC, "series": "3500", "rear_wheels": "dual", "bed": "long"}
    s = self.start(f, fuel=50.)
    assert s.source == "table" and s.tank_gal == 50.
    assert abs(s.mass_kg / LB_TO_KG - (7900. + 1040. + 250. - 0.5 * 50. * 7.1)) < 1e-6
    # a diesel long bed starts heavier than a gas short bed
    gas = self.start({**ERIC, "engine": "hemi_6.4", "transmission": "8HP75"}, fuel=100.)
    assert gas.source == "table" and gas.lb_per_gal == 6.0
    assert s.mass_kg > gas.mass_kg

  def test_bed_average(self):
    f = {k: v for k, v in ERIC.items() if k != "bed"}
    s = self.start(f, fuel=100.)
    assert s.source == "tableBedAverage"
    assert abs(s.mass_kg / LB_TO_KG - (7100. + 1040. + 250.)) < 1e-6
    assert s.std_kg > self.start(ERIC, fuel=100.).std_kg

  def test_fallback(self):
    # the placeholder table has no curb weights
    s = S.choose_start_weight(S.Configuration(dict(ERIC)), S.load_weight_table(), 0., None, 3541.)
    assert s.source == "fallback" and s.mass_kg == 3541. and s.std_kg == S.FALLBACK_STD_KG
    # unknown configuration (two fields missing), or no table at all
    assert self.start({k: v for k, v in ERIC.items() if k not in ("bed", "drive")}).source == "fallback"
    assert self.start({}).source == "fallback"
    assert self.start(ERIC, data=False).source == "fallback"
    # no allowance without Eric's published weight
    d = copy.deepcopy(SYNTHETIC_TABLE)
    d["configs"][0]["curb_weight_lb"] = None
    assert self.start({**ERIC, "series": "3500", "rear_wheels": "dual", "bed": "long"}, data=d).source == "fallback"

  def test_from_car_params(self):
    CP = car_params(vin(), fw=[("engine", b"11112222AK")])
    t = table()
    conf = S.identify_configuration(CP, t)
    assert conf.fields == {**{k: v for k, v in ERIC.items() if k != "bed"}}
    assert S.choose_start_weight(conf, t, 0., None, CP.mass).source == "tableBedAverage"
