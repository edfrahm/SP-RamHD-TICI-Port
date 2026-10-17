"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.input_dialog import InputDialogSP
from openpilot.system.ui.sunnypilot.widgets.list_view import button_item_sp, option_item_sp
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.list_view import text_item
from openpilot.sunnypilot.selfdrive.ramweightd.settings import KNOWN_EXTRA_STEPS, RAM_HD_PLATFORM, estimate_lb, parse_weight_lb

# Ram Heavy Duty weight estimator settings (edfrahm/SP-RamHD-TICI-Port#8). They only feed the estimator, which
# publishes its estimate; nothing here changes how the truck drives.
ESTIMATE_REFRESH_S = 2.


def estimate_text(state) -> str:
  est = estimate_lb(state)
  return tr("No estimate yet") if est is None else f"{est[0]:,.0f} lb ± {est[1]:,.0f} lb"


class ChryslerSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self._base_weight_item = button_item_sp(
      tr("Starting Weight"), tr("EDIT"),
      description=tr("Weight of the truck with a full tank, your usual gear and the driver, in pounds. The weight estimator " +
                     "starts from it. 0 uses Ram's published weight for this truck's configuration."),
      callback=self._edit_base_weight)
    self._known_extra_item = option_item_sp(
      tr("Known Extra Load"), "CruiseKnownExtraLb", 0, len(KNOWN_EXTRA_STEPS) - 1,
      description=tr("For a validation drive: load added on top of the starting weight (for example a 250 lb tank and " +
                     "passengers of known weight). It is only recorded, to measure the estimator's accuracy; the " +
                     "estimator does not use it. Set it back to 0 afterwards."),
      value_map=KNOWN_EXTRA_STEPS, label_callback=lambda lb: f"{lb:,.0f} lb")
    self._estimate_item = text_item(tr("Weight Estimate"), self._estimate_value,
                                    description=tr("The weight estimator's last kept estimate and its uncertainty."))
    self._ram_hd_items = [self._base_weight_item, self._known_extra_item, self._estimate_item]
    self.items = list(self._ram_hd_items)
    self._estimate = ""
    self._estimate_time = 0.
    self._dialog: InputDialogSP | None = None

  @staticmethod
  def _platform() -> str:
    bundle = ui_state.params.get("CarPlatformBundle")
    if bundle:
      return bundle.get("platform", "")
    return ui_state.CP.carFingerprint if ui_state.CP is not None else ""

  def _estimate_value(self) -> str:
    now = time.monotonic()
    if not self._estimate or now - self._estimate_time > ESTIMATE_REFRESH_S:
      self._estimate = estimate_text(ui_state.params.get("RamWeightGradeState"))
      self._estimate_time = now
    return self._estimate

  def _base_weight_lb(self) -> float:
    try:
      return float(ui_state.params.get("CruiseBaseWeightLb", return_default=True) or 0.)
    except (TypeError, ValueError):
      return 0.

  def _edit_base_weight(self):
    current = self._base_weight_lb()
    self._dialog = InputDialogSP(tr("Starting Weight"), tr("Pounds, 0 to 40,000 (0 uses Ram's published weight)"),
                                 current_text=f"{current:.0f}", callback=self._on_base_weight)
    self._dialog.show()

  @staticmethod
  def _on_base_weight(result: DialogResult, text: str):
    if result != DialogResult.CONFIRM:
      return
    value = parse_weight_lb(text)
    if value is not None:
      ui_state.params.put("CruiseBaseWeightLb", value)

  def update_settings(self):
    visible = self._platform() == RAM_HD_PLATFORM
    for item in self._ram_hd_items:
      item.set_visible(visible)
    if visible:
      base = self._base_weight_lb()
      self._base_weight_item.action_item.set_value(f"{base:,.0f} lb" if base > 0. else tr("Published table"))
