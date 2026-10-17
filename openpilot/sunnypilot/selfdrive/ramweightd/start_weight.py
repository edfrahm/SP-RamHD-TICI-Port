"""
Starting weight for the weight estimator, and identification of the truck's configuration.

The configuration (model year, 2500 or 3500, single or dual rear wheels, cab, bed, two- or four-wheel drive, engine,
transmission) comes from the vehicle identification number (VIN, CarParams.carVin) and from the engine and
transmission module part numbers the truck reports at startup (CarParams.carFw). What each VIN character and part
number means comes ONLY from the published-weight table (ram_hd_weights.json); the only VIN position known here is
the model-year character at position 10, which every North American VIN has. The VIN itself is never logged, stored
or published: only the configuration in words.

Starting weight, in order:
1. the device setting CruiseBaseWeightLb when above 0 (Eric's truck: 8,400 lb with a full tank), minus missing fuel;
2. Ram's published curb weight for the identified configuration, plus the options allowance (Eric's empty weight,
   8,040 lb, minus Ram's published curb weight for Eric's configuration), plus 250 lb, minus missing fuel; when only
   the bed is unknown, the average over both beds with a wider uncertainty;
3. otherwise the platform's weight from CarParams, with a wide uncertainty.
Curb weights and Eric's figures include a full tank, hence "minus missing fuel". When the fuel level is not
available, the tank is assumed half full, with the uncertainty of not knowing it.
"""
import json
import math
import os
from dataclasses import dataclass, field

from openpilot.sunnypilot.selfdrive.ramweightd.estimator import LB_TO_KG

DEFAULT_TABLE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ram_hd_weights.json")
TABLE_SCHEMA = 1

CONFIG_FIELDS = ("model_year", "series", "rear_wheels", "cab", "bed", "drive", "engine", "transmission")
FIELD_VALUES = {
  "series": ("2500", "3500"),
  "rear_wheels": ("single", "dual"),
  "cab": ("regular", "crew", "mega"),
  "bed": ("short", "long"),
  "drive": ("4x2", "4x4"),
  "engine": ("hemi_6.4", "cummins_so", "cummins_ho"),
  "transmission": ("8HP75", "68RFE", "AS69RC"),
}

# Eric's truck (edfrahm/SP-RamHD-TICI-Port#8): defines the options allowance
ERIC_CONFIGURATION = {"model_year": 2022, "series": "2500", "rear_wheels": "single", "cab": "crew", "bed": "short",
                      "drive": "4x4", "engine": "cummins_so", "transmission": "68RFE"}
ERIC_EMPTY_LB = 8040.
DRIVER_ALLOWANCE_LB = 250.  # driver and everyday gear

DEFAULT_TANK_GAL = 32.  # assumed when the table gives no tank size (a typical Ram Heavy Duty standard tank)
DEFAULT_DIESEL_LB_PER_GAL = 7.1
DEFAULT_GASOLINE_LB_PER_GAL = 6.0
UNKNOWN_FUEL_FRACTION = 0.5  # fuel level unknown: assume half full

SETTING_STD_KG = 50.    # a weight the owner entered (for example from a scale)
TABLE_STD_KG = 150.     # options and equipment vary from truck to truck
FALLBACK_STD_KG = 600.  # configuration unknown: Ram Heavy Duty trucks range from about 6,000 to 9,000 lb

# VIN position 9 check digit (49 CFR 565): letter values and position weights
_VIN_VALUES = {**{str(d): d for d in range(10)},
               **dict(zip("ABCDEFGH", range(1, 9), strict=True)), **dict(zip("JKLMN", range(1, 6), strict=True)), "P": 7, "R": 9,
               **dict(zip("STUVWXYZ", range(2, 10), strict=True))}
_VIN_WEIGHTS = (8, 7, 6, 5, 4, 3, 2, 10, 0, 9, 8, 7, 6, 5, 4, 3, 2)


@dataclass
class WeightTable:
  configs: list[dict]
  model_year_codes: dict[str, int]
  vin_rules: list[dict]
  part_rules: list[tuple[str, dict]]  # (ecu kind "engine" or "transmission", rule)
  diesel_lb_per_gal: float = DEFAULT_DIESEL_LB_PER_GAL
  gasoline_lb_per_gal: float = DEFAULT_GASOLINE_LB_PER_GAL
  placeholder: bool = False


def parse_weight_table(data: dict) -> WeightTable:
  """Checks the table layout (schema 1) and returns it. Raises ValueError on a malformed table."""
  if not isinstance(data, dict) or data.get("schema") != TABLE_SCHEMA:
    raise ValueError(f"weight table: schema must be {TABLE_SCHEMA}")
  configs = data.get("configs", [])
  if not isinstance(configs, list):
    raise ValueError("weight table: configs must be a list")
  for i, c in enumerate(configs):
    if not isinstance(c, dict):
      raise ValueError(f"weight table: configs[{i}] is not an object")
    for k in CONFIG_FIELDS:
      if k not in c:
        raise ValueError(f"weight table: configs[{i}] lacks {k}")
      if k in FIELD_VALUES and c[k] not in FIELD_VALUES[k]:
        raise ValueError(f"weight table: configs[{i}].{k} = {c[k]!r} is not one of {FIELD_VALUES[k]}")
    for k in ("curb_weight_lb", "fuel_tank_gal"):
      if c.get(k) is not None and not (isinstance(c[k], (int, float)) and c[k] > 0):
        raise ValueError(f"weight table: configs[{i}].{k} must be a positive number or null")
  vin = data.get("vin", {}) or {}
  codes = {str(k): int(v) for k, v in (vin.get("model_year_codes", {}) or {}).items()}
  rules = vin.get("rules", []) or []
  for i, r in enumerate(rules):
    if not (isinstance(r, dict) and isinstance(r.get("position"), int) and 1 <= r["position"] <= 17
            and isinstance(r.get("code"), str) and isinstance(r.get("meaning"), dict)):
      raise ValueError(f"weight table: vin.rules[{i}] needs position (1-17), code and meaning")
    _check_meaning(r["meaning"], f"vin.rules[{i}]")
  part_rules = []
  for kind in ("engine", "transmission"):
    for i, r in enumerate(data.get(f"{kind}_parts", []) or []):
      if not (isinstance(r, dict) and isinstance(r.get("part_prefix_or_number"), str) and r["part_prefix_or_number"]):
        raise ValueError(f"weight table: {kind}_parts[{i}] needs part_prefix_or_number")
      meaning = {k: v for k, v in r.items() if k in CONFIG_FIELDS}
      _check_meaning(meaning, f"{kind}_parts[{i}]")
      part_rules.append((kind, r))
  return WeightTable(
    configs=configs,
    model_year_codes=codes,
    vin_rules=rules,
    part_rules=part_rules,
    diesel_lb_per_gal=float(data.get("diesel_lb_per_gal") or DEFAULT_DIESEL_LB_PER_GAL),
    gasoline_lb_per_gal=float(data.get("gasoline_lb_per_gal") or DEFAULT_GASOLINE_LB_PER_GAL),
    placeholder=bool(data.get("placeholder", False)),
  )


def _check_meaning(meaning: dict, where: str) -> None:
  for k, v in meaning.items():
    if k not in CONFIG_FIELDS:
      raise ValueError(f"weight table: {where} meaning has unknown field {k!r}")
    if k in FIELD_VALUES and v not in FIELD_VALUES[k]:
      raise ValueError(f"weight table: {where} meaning {k} = {v!r} is not one of {FIELD_VALUES[k]}")


def load_weight_table(path: str | None = None) -> WeightTable | None:
  """The published-weight table, or None when the file is missing or malformed (the estimator then falls back)."""
  try:
    with open(path or DEFAULT_TABLE_PATH) as f:
      return parse_weight_table(json.load(f))
  except (OSError, ValueError) as e:
    from openpilot.common.swaglog import cloudlog
    cloudlog.warning(f"ramweightd: weight table not usable: {e}")
    return None


def vin_is_valid(vin: str) -> bool:
  if not isinstance(vin, str) or len(vin) != 17 or vin == "0" * 17:
    return False
  vin = vin.upper()
  if any(ch not in _VIN_VALUES for ch in vin):
    return False
  total = sum(_VIN_VALUES[ch] * w for ch, w in zip(vin, _VIN_WEIGHTS, strict=True))
  check = total % 11
  return vin[8] == ("X" if check == 10 else str(check))


def _merge(fields: dict, conflicts: set, meaning: dict) -> None:
  for k, v in meaning.items():
    if k in conflicts:
      continue
    if k in fields and fields[k] != v:
      del fields[k]
      conflicts.add(k)
    else:
      fields[k] = v


def decode_vin(vin: str, table: WeightTable) -> tuple[dict, set]:
  """Configuration fields from the VIN, using only the table's rules. Returns (fields, conflicting field names)."""
  fields: dict = {}
  conflicts: set = set()
  if not vin_is_valid(vin):
    return fields, conflicts
  vin = vin.upper()
  year = table.model_year_codes.get(vin[9])
  if year is None:
    return fields, conflicts
  fields["model_year"] = year
  for rule in table.vin_rules:
    years = rule.get("model_years")
    if years and year not in years:
      continue
    if vin[rule["position"] - 1] == rule["code"].upper():
      _merge(fields, conflicts, rule["meaning"])
  return fields, conflicts


def decode_parts(car_fw, table: WeightTable) -> tuple[dict, set]:
  """Configuration fields from the engine and transmission module part numbers (CarParams.carFw)."""
  fields: dict = {}
  conflicts: set = set()
  for fw in car_fw:
    kind = str(fw.ecu)
    if kind not in ("engine", "transmission"):
      continue
    part = bytes(fw.fwVersion).decode("ascii", errors="ignore").strip().upper()
    if not part:
      continue
    for rule_kind, rule in table.part_rules:
      if rule_kind == kind and part.startswith(rule["part_prefix_or_number"].strip().upper()):
        _merge(fields, conflicts, {k: v for k, v in rule.items() if k in CONFIG_FIELDS})
  return fields, conflicts


@dataclass
class Configuration:
  fields: dict = field(default_factory=dict)
  conflicts: set = field(default_factory=set)

  @property
  def text(self) -> str:
    return describe_configuration(self.fields, self.conflicts)


def identify_configuration(CP, table: WeightTable | None) -> Configuration:
  if table is None:
    return Configuration()
  fields, conflicts = decode_vin(CP.carVin, table)
  part_fields, part_conflicts = decode_parts(CP.carFw, table)
  conflicts |= part_conflicts
  _merge(fields, conflicts, part_fields)
  return Configuration(fields, conflicts)


_WORDS = {
  "rear_wheels": {"single": "single rear wheels", "dual": "dual rear wheels"},
  "cab": {"regular": "Regular Cab", "crew": "Crew Cab", "mega": "Mega Cab"},
  "bed": {"short": "short bed", "long": "long bed"},
  "drive": {"4x2": "two-wheel drive", "4x4": "four-wheel drive"},
  "engine": {"hemi_6.4": "6.4 L HEMI gasoline engine", "cummins_so": "Cummins 6.7 L diesel, standard output",
             "cummins_ho": "Cummins 6.7 L diesel, high output"},
  "transmission": {"8HP75": "8HP75 eight-speed automatic", "68RFE": "68RFE six-speed automatic",
                   "AS69RC": "AS69RC six-speed automatic"},
}
_FIELD_WORDS = {"model_year": "model year", "series": "series", "rear_wheels": "rear wheels", "cab": "cab", "bed": "bed",
                "drive": "drive", "engine": "engine", "transmission": "transmission"}


def describe_configuration(fields: dict, conflicts: set | None = None) -> str:
  """The configuration in words (never the VIN)."""
  head = " ".join(x for x in (str(fields["model_year"]) if "model_year" in fields else "", "Ram",
                              fields.get("series", "")) if x)
  parts = [head] + [_WORDS[k][fields[k]] for k in ("rear_wheels", "cab", "bed", "drive", "engine", "transmission") if k in fields]
  unknown = [_FIELD_WORDS[k] for k in CONFIG_FIELDS if k not in fields]
  text = ", ".join(parts)
  if unknown:
    text += "; not identified: " + ", ".join(unknown)
  if conflicts:
    text += "; conflicting: " + ", ".join(_FIELD_WORDS[k] for k in CONFIG_FIELDS if k in conflicts)
  return text


def matching_configs(fields: dict, table: WeightTable) -> list[dict]:
  return [c for c in table.configs if all(c[k] == v for k, v in fields.items() if k in CONFIG_FIELDS)]


def eric_curb_weight_lb(table: WeightTable) -> float | None:
  for c in table.configs:
    if all(c[k] == v for k, v in ERIC_CONFIGURATION.items()):
      return c.get("curb_weight_lb")
  return None


@dataclass
class StartWeight:
  mass_kg: float
  std_kg: float
  source: str            # "setting", "table", "tableBedAverage" or "fallback" (the message's StartSource names)
  tank_gal: float
  lb_per_gal: float
  fuel_level_known: bool
  detail: str


def fuel_density_lb_per_gal(fields: dict, table: WeightTable | None) -> float:
  diesel = table.diesel_lb_per_gal if table else DEFAULT_DIESEL_LB_PER_GAL
  gasoline = table.gasoline_lb_per_gal if table else DEFAULT_GASOLINE_LB_PER_GAL
  return gasoline if fields.get("engine") == "hemi_6.4" else diesel  # unknown engine: diesel, like most Ram HD


def choose_start_weight(config: Configuration, table: WeightTable | None, base_weight_lb: float,
                        fuel_level_pct: float | None, cp_mass_kg: float) -> StartWeight:
  fields = config.fields
  lb_per_gal = fuel_density_lb_per_gal(fields, table)
  matches = matching_configs(fields, table) if table else []
  tanks = [c["fuel_tank_gal"] for c in matches if c.get("fuel_tank_gal")]
  tank_gal = sum(tanks) / len(tanks) if tanks else DEFAULT_TANK_GAL

  full_tank_lb = tank_gal * lb_per_gal
  if fuel_level_pct is not None:
    missing_lb = (1. - min(max(fuel_level_pct, 0.), 100.) / 100.) * full_tank_lb
    fuel_std_kg = 0.03 * full_tank_lb * LB_TO_KG
    fuel_text = f"minus {missing_lb:.0f} lb of missing fuel"
  else:
    missing_lb = UNKNOWN_FUEL_FRACTION * full_tank_lb
    fuel_std_kg = full_tank_lb * LB_TO_KG / math.sqrt(12.)
    fuel_text = f"minus {missing_lb:.0f} lb of fuel (level unknown: half tank assumed)"
  known = fuel_level_pct is not None

  if base_weight_lb > 0.:
    mass_lb = base_weight_lb - missing_lb
    return StartWeight(mass_lb * LB_TO_KG, math.hypot(SETTING_STD_KG, fuel_std_kg), "setting", tank_gal, lb_per_gal, known,
                       f"setting {base_weight_lb:.0f} lb (full tank) {fuel_text}")

  eric_curb = eric_curb_weight_lb(table) if table else None
  if eric_curb is not None and matches and all(c.get("curb_weight_lb") for c in matches):
    allowance = ERIC_EMPTY_LB - eric_curb
    # everything but the bed must be identified; rows that differ only in the bed are averaged
    others = {tuple(c[k] for k in CONFIG_FIELDS if k != "bed") for c in matches}
    if len(others) == 1:
      curbs = [c["curb_weight_lb"] for c in matches]
      curb = sum(curbs) / len(curbs)
      half_spread_kg = (max(curbs) - min(curbs)) * LB_TO_KG / 2.
      if len({c["bed"] for c in matches}) == 1:
        source, std = "table", math.hypot(TABLE_STD_KG, half_spread_kg)
      else:
        source, std = "tableBedAverage", math.hypot(TABLE_STD_KG, max(half_spread_kg, 50.))
      mass_lb = curb + allowance + DRIVER_ALLOWANCE_LB - missing_lb
      return StartWeight(mass_lb * LB_TO_KG, math.hypot(std, fuel_std_kg), source, tank_gal, lb_per_gal, known,
                         f"published curb weight {curb:.0f} lb + options allowance {allowance:.0f} lb + " +
                         f"{DRIVER_ALLOWANCE_LB:.0f} lb {fuel_text}")

  return StartWeight(cp_mass_kg, FALLBACK_STD_KG, "fallback", tank_gal, lb_per_gal, known,
                     f"platform weight {cp_mass_kg / LB_TO_KG:.0f} lb from CarParams (configuration or table weight unknown)")
