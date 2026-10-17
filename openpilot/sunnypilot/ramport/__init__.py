"""
Ram port shared data upload (edfrahm/SP-RamHD-TICI-Port#9).

A small on-device process, ramportd, that records a minimal table of driving numbers on Ram Heavy Duty
trucks and sends it to the project's upload service. The file layout and the upload rules are fixed by
the contract "RamPort shared data upload: file format and upload protocol, version 1".

This file holds the one check the process manager needs: is the car a Ram Heavy Duty truck?
"""
from opendbc.car.chrysler.values import RAM_HD

_persistent_fingerprint: str | None = None


def _fingerprint_from_persistent(params) -> str:
  """The platform name of the last car this device drove, kept in the setting CarParamsPersistent.

  Read once and cached: the setting only changes at the start of a drive, and then the live car
  parameters (carParams message) are available instead.
  """
  global _persistent_fingerprint
  if _persistent_fingerprint is None:
    _persistent_fingerprint = ""
    try:
      cp_bytes = params.get("CarParamsPersistent")
      if cp_bytes:
        from openpilot.cereal import messaging  # imported here so the process manager does not load it early
        from opendbc.car.structs import car
        _persistent_fingerprint = str(messaging.log_from_bytes(cp_bytes, car.CarParams).carFingerprint)
    except Exception:
      _persistent_fingerprint = ""
  return _persistent_fingerprint


def is_ram_hd(params, CP) -> bool:
  """True on a Ram Heavy Duty truck (platform RAM_HD_5TH_GEN).

  CP is the car parameters message the process manager holds. Before the first drive since boot it
  is empty, so the last car's saved parameters are used, which lets uploads run while parked.
  """
  fingerprint = str(CP.carFingerprint) if CP is not None and CP.carFingerprint else _fingerprint_from_persistent(params)
  return fingerprint in RAM_HD
