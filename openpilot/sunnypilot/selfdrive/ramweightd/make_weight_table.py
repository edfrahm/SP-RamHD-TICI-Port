#!/usr/bin/env python3
"""
Build the device's slim published-weight table (ram_hd_weights.json) from the full research table.

  make_weight_table.py <full research table, ram_hd_specs.json> [--out ram_hd_weights.json]

The full table (about 1 MB, not committed) was built for edfrahm/SP-RamHD-TICI-Port#8 from Stellantis documents:
Ram Heavy Duty weight-and-tow charts and body builder instructions (curb weights, "base weight" of the base trim with
a full fuel tank), owner's manuals (fuel tanks), media specifications, and the NHTSA VIN decoding guides that
Stellantis files (VIN rules). Its retrieval date is copied into the slim table. The slim table keeps only what the
estimator needs:
- configs: model year, series, rear wheels, cab, bed, bed length, box variant (standard or RamBox), package (Power
  Wagon, Rebel or none), drive, engine, transmission, recommended curb weight (the research table's best figure,
  falling back to its secondary source), curb weight kind, standard fuel tank, optional larger tank (Crew Cab with the
  8 ft box, 2020-2024: 50 gallons), source id;
- sources (id, title, url), fuel densities;
- vin: model-year codes (code -> year) and the rules with a meaning for those fields. Positions 5-7 are one
  three-character code (series, rear wheels, drive, cab, bed); position 8 is the engine ("L" = Cummins, standard or
  high output: the VIN cannot tell them apart on a 3500); positions 4 and 11 (weight rating band, plant) are left out;
- engine_parts: engine module part numbers whose meaning is known.
"""
import argparse
import json
import os
import re
import sys

from openpilot.sunnypilot.selfdrive.ramweightd.start_weight import DEFAULT_TABLE_PATH, FIELD_VALUES, parse_weight_table

CONFIG_KEYS = ("model_year", "series", "rear_wheels", "cab", "bed", "bed_length_in", "box_variant", "package", "drive", "engine",
               "transmission", "recommended_curb_weight_lb", "curb_weight_kind", "fuel_tank_gal", "fuel_tank_optional_gal", "source")
POWER_WAGON_TRIM = "Power Wagon"


def _valid(field: str, value) -> bool:
  return field in FIELD_VALUES and value in FIELD_VALUES[field]


def slim_vin_rules(rules: list[dict]) -> list[dict]:
  out = []
  for r in rules:
    m = r["meaning"]
    if r["position"] == "5-7":
      meaning = {k: m[k] for k in ("series", "rear_wheels", "drive", "cab", "bed") if _valid(k, m.get(k))}
      if POWER_WAGON_TRIM in str(m.get("trim", "")):
        # one code covers the Power Wagon, and from 2023 also the Rebel
        for years, package in (([y for y in r["model_years"] if y <= 2022], "power_wagon"),
                               ([y for y in r["model_years"] if y >= 2023], ["power_wagon", "rebel"])):
          if years:
            out.append({"model_years": years, "position": "5-7", "code": r["code"], "meaning": {**meaning, "package": package},
                        "source": r["source"]})
        continue
      meaning["package"] = None
    elif r["position"] == 8:
      if m.get("engine") == "cummins":
        meaning = {"engine": ["cummins_so", "cummins_ho"]}
      elif _valid("engine", m.get("engine")):
        meaning = {"engine": m["engine"]}
        if _valid("transmission", m.get("implies_transmission")):
          meaning["transmission"] = m["implies_transmission"]
      else:
        continue
    else:
      continue  # weight rating band and plant: not needed
    out.append({"model_years": r["model_years"], "position": r["position"], "code": r["code"], "meaning": meaning, "source": r["source"]})
  return out


def slim_engine_parts(parts: list[dict]) -> list[dict]:
  out = []
  for p in parts:
    meaning = {k: p[k] for k in ("engine", "transmission", "series") if _valid(k, p.get(k))}
    if meaning:
      out.append({"part_prefix_or_number": p["part_number"], **meaning, "source": p["source"]})
  return out


def referenced_sources(ids: list[str], text: str) -> set[str]:
  """Source ids named in a source text, including ranges like nhtsa_vin_2019..2024."""
  out = {i for i in ids if i in text}
  for m in re.finditer(r"([a-z_]+?)_(\d{4})\.\.(\d{4})", text):
    out |= {f"{m.group(1)}_{y}" for y in range(int(m.group(2)), int(m.group(3)) + 1) if f"{m.group(1)}_{y}" in ids}
  return out


def build(full: dict) -> dict:
  configs = []
  for c in full["configs"]:
    row = {k: c.get(k) for k in CONFIG_KEYS}
    if row["recommended_curb_weight_lb"] is None:
      row["recommended_curb_weight_lb"] = c.get("curb_weight_lb") or c.get("secondary_curb_weight_lb")
    configs.append(row)
  codes = full["vin"]["model_year_codes"]
  # the research table maps year -> code; the device table maps code -> year
  model_year_codes = {str(v): int(k) for k, v in codes.items()} if all(str(k).isdigit() for k in codes) else codes
  rules = slim_vin_rules(full["vin"]["rules"])
  parts = slim_engine_parts(full.get("engine_parts", []))
  ids = [s["id"] for s in full["sources"]]
  used = {"tedb_ed40", "media_presskit_2019"}  # fuel densities, drag coefficient
  for x in configs + rules + parts:
    used |= referenced_sources(ids, str(x["source"]))
  sources = [{"id": s["id"], "title": s["title"], "url": s.get("url", "")} for s in full["sources"] if s["id"] in used]
  return {
    "schema": 1,
    "placeholder": False,
    "note": "Slim device table generated by make_weight_table.py from the full research table (edfrahm/SP-RamHD-TICI-Port#8), " +
            "which was built from Stellantis documents: weight-and-tow charts and body builder instructions (curb weight = " +
            "published base weight of the base trim, full fuel tank), owner's manuals, media specifications, and the NHTSA " +
            "VIN guides. Do not edit by hand: regenerate.",
    "retrieved": full.get("retrieved"),
    "sources": sources,
    "diesel_lb_per_gal": full["diesel_lb_per_gal"],
    "gasoline_lb_per_gal": full["gasoline_lb_per_gal"],
    "configs": configs,
    "vin": {"model_year_codes": model_year_codes, "rules": rules},
    "engine_parts": parts,
    "transmission_parts": [],
  }


def dumps(table: dict) -> str:
  """Compact but diff-friendly: one config, rule or source per line."""
  lines = ["{"]
  keys = list(table)
  for i, k in enumerate(keys):
    v = table[k]
    comma = "," if i < len(keys) - 1 else ""
    if k in ("configs", "sources", "engine_parts") and v:
      lines.append(f'  "{k}": [')
      lines += [f"    {json.dumps(x)}{',' if j < len(v) - 1 else ''}" for j, x in enumerate(v)]
      lines.append(f"  ]{comma}")
    elif k == "vin":
      lines.append('  "vin": {')
      lines.append(f'    "model_year_codes": {json.dumps(v["model_year_codes"])},')
      lines.append('    "rules": [')
      lines += [f"      {json.dumps(x)}{',' if j < len(v['rules']) - 1 else ''}" for j, x in enumerate(v["rules"])]
      lines.append("    ]")
      lines.append(f"  }}{comma}")
    else:
      lines.append(f'  "{k}": {json.dumps(v)}{comma}')
  lines.append("}")
  return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("full", help="the full research table (ram_hd_specs.json)")
  parser.add_argument("--out", default=DEFAULT_TABLE_PATH)
  args = parser.parse_args(argv)
  with open(args.full) as f:
    table = build(json.load(f))
  parse_weight_table(json.loads(dumps(table)))  # the estimator must accept it
  with open(args.out, "w") as f:
    f.write(dumps(table))
  print(f"wrote {args.out}: {len(table['configs'])} configurations, {len(table['vin']['rules'])} VIN rules, " +
        f"{len(table['engine_parts'])} engine part numbers, {os.path.getsize(args.out) // 1024} KB")
  return 0


if __name__ == "__main__":
  sys.exit(main())
