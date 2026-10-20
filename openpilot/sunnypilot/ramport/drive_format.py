"""
The drive file layout, version 1, exactly as the contract fixes it ("RamPort shared data upload: file format
and upload protocol, version 1", section 1).

A drive file is gzip-compressed text (gzip is the common compression format). Line 1 is the marker
"#RAMPORT " followed by one JSON object (JavaScript Object Notation, a simple text format for named values)
with the metadata. Line 2 is the column header. Then one row per sample, about 10 per second.
"""
import gzip
import json
import math
import os
import re
from datetime import datetime, UTC

FORMAT_NAME = "ramport-drive"
FORMAT_VERSION = 1
MARKER = "#RAMPORT "
SAMPLE_HZ = 10
TRIM_S = 120  # the first and the last 2 minutes of every drive are never written
MAX_ROWS_PER_PART = 36000  # one hour at 10 samples per second
MAX_PART_SECONDS = 3600.0
MAX_FILE_BYTES = 4_000_000  # the service refuses larger files (its host accepts requests up to 4.5 MB; a typical hour is about 0.7 MB)

# Column name, digits after the decimal point (None = whole number or 0/1 flag)
COLUMNS: list[tuple[str, int | None]] = [
  ("t_s", 3),
  ("speed_mps", 3),
  ("accel_mps2", 3),
  ("accel_pedal_pct", 1),
  ("engine_rpm", 1),
  ("engine_torque_nm", 2),
  ("gear_actual", None),
  ("gear_desired", None),
  ("converter_locked", None),
  ("shift_pending", None),
  ("trans_input_rpm", 1),
  ("trans_output_rpm", 1),
  ("brake_pressed", None),
  ("service_brake_on", None),
  ("exhaust_brake_on", None),
  ("tow_haul_on", None),
  ("trailer_connected", None),
  ("fuel_level_pct", 1),
  ("grade", 4),
  ("grade_std", 4),
  ("mass_kg", 1),
  ("mass_std_kg", 1),
  ("drag_n_per_mps2", 4),
  ("cruise_engaged", None),
  ("set_speed_mps", 3),
  ("acc_engine_torque_req_nm", 2),
  ("acc_brake_req", None),
  ("acc_decel_req_mps2", 4),
  ("acc_distance_setting", None),
  ("lead_prob", 3),
  ("lead_dist_m", 2),
  ("lead_speed_mps", 3),
  ("lead_accel_mps2", 3),
]
COLUMN_NAMES = [name for name, _ in COLUMNS]
DECIMALS = dict(COLUMNS)
HEADER_LINE = ",".join(COLUMN_NAMES)

BOOLEAN_COLUMNS = {"converter_locked", "shift_pending", "brake_pressed", "service_brake_on", "exhaust_brake_on",
                   "tow_haul_on", "trailer_connected", "cruise_engaged"}

# Physical limits (contract section 1.3). The service refuses a file with any value outside these.
LIMITS: dict[str, tuple[float, float]] = {
  "speed_mps": (0, 60),
  "accel_mps2": (-12, 8),
  "accel_pedal_pct": (0, 100),
  "engine_rpm": (0, 5500),
  "engine_torque_nm": (-500, 1600),
  "acc_engine_torque_req_nm": (-500, 1600),
  "gear_actual": (0, 10),
  "gear_desired": (0, 10),
  "trans_input_rpm": (0, 8000),
  "trans_output_rpm": (0, 8000),
  "fuel_level_pct": (0, 100),
  "grade": (-0.4, 0.4),
  "grade_std": (0, 1),
  "mass_kg": (1000, 25000),
  "mass_std_kg": (0, 20000),
  "drag_n_per_mps2": (0, 10),
  "set_speed_mps": (0, 70),
  "acc_brake_req": (0, 7),
  "acc_decel_req_mps2": (-16, 4),
  "acc_distance_setting": (0, 3),
  "lead_prob": (0, 1),
  "lead_dist_m": (0, 300),
  "lead_speed_mps": (-20, 80),
  "lead_accel_mps2": (-20, 20),
}
for _name in BOOLEAN_COLUMNS:
  LIMITS[_name] = (0, 1)

# A value just outside a limit (by at most 1 percent of the allowed span) is set to the limit. This covers
# rounding and the edges of how the truck encodes a signal: the factory cruise computer's braking amount
# (ACC_DECEL) tops out at 4.0042, a hair above the contract's 4. A value farther outside is written as an
# empty field (not available for that sample), so one odd reading never makes the service refuse the file.
CLAMP_FRACTION = 0.01

FILE_NAME_RE = re.compile(r"^([0-9a-f]{16})_p(\d{2,})\.csv\.gz$")

METADATA_KEYS = ["format", "version", "truck", "engine_part", "transmission_part", "software", "drive_key", "part",
                 "rows", "sample_hz", "trim_start_s", "trim_end_s", "date"]


def wall_time() -> float:
  """Calendar time in seconds since 1970 (UTC), for the part date and the signature times only."""
  return datetime.now(UTC).timestamp()


def part_file_name(drive_key: str, part: int) -> str:
  return f"{drive_key}_p{part:02d}.csv.gz"


def format_number(value: float, decimals: int) -> str:
  """A plain decimal number: no exponent, at most `decimals` digits after the point, no trailing zeros."""
  text = f"{value:.{decimals}f}"
  if "." in text:
    text = text.rstrip("0").rstrip(".")
  if text in ("-0", ""):
    text = "0"
  return text


def format_value(column: str, value) -> str:
  """One field of a data row. None, not-a-number and values far outside the limits become an empty field."""
  if value is None:
    return ""
  if column in BOOLEAN_COLUMNS:
    return "1" if bool(value) else "0"

  try:
    value = float(value)
  except (TypeError, ValueError):
    return ""
  if not math.isfinite(value):
    return ""

  limits = LIMITS.get(column)
  if limits is not None:
    lo, hi = limits
    tolerance = (hi - lo) * CLAMP_FRACTION
    if value < lo:
      if value < lo - tolerance:
        return ""
      value = lo
    elif value > hi:
      if value > hi + tolerance:
        return ""
      value = hi

  decimals = DECIMALS[column]
  if decimals is None:
    return str(int(round(value)))
  return format_number(value, decimals)


def build_metadata(static_meta: dict, drive_key: str, part: int, rows: int, last_part: bool, date: str) -> dict:
  """The line 1 metadata (contract section 1.1). static_meta holds truck, engine_part, transmission_part, software."""
  return {
    "format": FORMAT_NAME,
    "version": FORMAT_VERSION,
    "truck": str(static_meta.get("truck", "")),
    "engine_part": str(static_meta.get("engine_part", "")),
    "transmission_part": str(static_meta.get("transmission_part", "")),
    "software": str(static_meta.get("software", "")),
    "drive_key": drive_key,
    "part": int(part),
    "rows": int(rows),
    "sample_hz": SAMPLE_HZ,
    "trim_start_s": TRIM_S if part == 1 else 0,
    "trim_end_s": TRIM_S if last_part else 0,
    "date": date,
  }


def write_part_file(out_path: str, metadata: dict, row_lines) -> None:
  """Write one finished part as gzip, atomically: a reader never sees a half-written file.

  The gzip header normally carries the original file name and the time it was written; both are left
  out (empty name, time 0) so the file says nothing beyond its contents.
  """
  tmp_path = out_path + ".tmp"
  with open(tmp_path, "wb") as raw, gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as gz:
    gz.write((MARKER + json.dumps(metadata, separators=(",", ":")) + "\n").encode("utf-8"))
    gz.write((HEADER_LINE + "\n").encode("utf-8"))
    for line in row_lines:
      gz.write(line.encode("utf-8"))
  os.replace(tmp_path, out_path)


def read_part_file(path: str) -> tuple[dict, list[str], list[list[str]]]:
  """Read a drive file back: (metadata, column names, rows). Used by the tests and the command-line tool."""
  with gzip.open(path, "rt", encoding="utf-8", newline="\n") as f:
    lines = f.read().split("\n")
  if lines and lines[-1] == "":
    lines.pop()
  assert lines[0].startswith(MARKER), "line 1 must start with the #RAMPORT marker"
  metadata = json.loads(lines[0][len(MARKER):])
  header = lines[1].split(",")
  rows = [line.split(",") for line in lines[2:]]
  return metadata, header, rows
