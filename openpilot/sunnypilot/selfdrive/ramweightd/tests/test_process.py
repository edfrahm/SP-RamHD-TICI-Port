from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.system.manager.process_config import managed_processes


def car_params(brand: str, fingerprint: str):
  CP = car.CarParams.new_message()
  CP.brand = brand
  CP.carFingerprint = fingerprint
  return CP


class TestRamWeightProcess(OpenpilotTestCase):
  def test_registered(self):
    p = managed_processes["ramweightd"]
    assert p.module == "openpilot.sunnypilot.selfdrive.ramweightd.ramweightd"

  def test_runs_only_onroad_on_ram_heavy_duty(self):
    should_run = managed_processes["ramweightd"].should_run
    params = Params()
    assert should_run(True, params, car_params("chrysler", "RAM_HD_5TH_GEN"))
    assert not should_run(False, params, car_params("chrysler", "RAM_HD_5TH_GEN"))  # offroad
    assert not should_run(True, params, car_params("chrysler", "RAM_1500_5TH_GEN"))
    assert not should_run(True, params, car_params("chrysler", "CHRYSLER_PACIFICA_2018"))
    assert not should_run(True, params, car_params("toyota", "RAM_HD_5TH_GEN"))
    assert not should_run(True, params, car.CarParams.new_message())  # before fingerprinting
