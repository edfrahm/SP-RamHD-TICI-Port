"""
The weight estimator's device settings, shared by the service and the settings screen (no user interface code here,
so it can be tested anywhere).

- CruiseBaseWeightLb: starting weight in pounds with a full tank (Eric's truck: 8,400); 0 uses the published table.
- CruiseKnownExtraLb: the known added load on a validation drive, 0 to 2,000 lb in 10 lb steps; published only.
- RamWeightGradeState: the estimate kept between drives (written by the service).
"""
LB_TO_KG = 0.45359237
RAM_HD_PLATFORM = "RAM_HD_5TH_GEN"
MAX_BASE_WEIGHT_LB = 40000.
KNOWN_EXTRA_STEPS = {i: float(i * 10) for i in range(201)}  # option index -> pounds: 0 to 2,000 lb in 10 lb steps


def parse_weight_lb(text: str) -> float | None:
  """A typed weight in pounds (commas and "lb" allowed); None unless it is a number from 0 to 40,000."""
  try:
    value = float(text.lower().replace(",", "").replace("lb", "").strip())
  except (AttributeError, ValueError):
    return None
  return value if 0. <= value <= MAX_BASE_WEIGHT_LB else None


def estimate_lb(state) -> tuple[float, float] | None:
  """(weight, uncertainty) in pounds from the kept estimate RamWeightGradeState, or None when there is none."""
  try:
    return float(state["mass_kg"]) / LB_TO_KG, float(state["mass_std_kg"]) / LB_TO_KG
  except (TypeError, KeyError, ValueError):
    return None
