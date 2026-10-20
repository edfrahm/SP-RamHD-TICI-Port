"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs, apply_hysteresis
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers
from openpilot.sunnypilot.selfdrive.controls.lib.planned_speed import RAM_ACC_TUNING_PARAM, ram_acc_tuning_available

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

ALLOWED_SPEED_THRESHOLD = 1.8  # m/s, ~4 MPH
HYST_GAP = 0.0  # currently disabled; TODO-SP: might need to be brand-specific
INACTIVE_TIMER = 0.4

# Custom Ram ACC Tuning (edfrahm/SP-RamHD-TICI-Port#11). The Ram Heavy Duty's cruise control answers a held button with
# 1 mph and then a jump to the next multiple of 5 every 0.72 s, so holding the button hunts around any other target
# (75 -> 63 swings between 60 and 65). It is tapped instead: every tap moves the set speed exactly 1 mph (the driver's
# own taps of 0.06 to 0.18 s all registered, and taps 0.2 s apart all counted), at most 4 mph per second, still well
# ahead of the truck, which loses about 1.1 mph per second when it coasts.
TAP_PRESS = 0.1  # s
TAP_RELEASE = 0.15  # s
TAP_UNANSWERED = 4  # taps in a row that did not move the set speed ...
TAP_RETRY = 2.  # s ... then wait this long before tapping again
DRIVER_PAUSE = 3.  # s after the driver's last pedal or cruise button, so the driver's own change is not undone at once


SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.v_target_ms_last = 0.0
    self.is_metric = False

    self.cruise_button_timers = CRUISE_BUTTON_TIMER

    self.ram_acc_tuning_available = ram_acc_tuning_available(CP, CP_SP)
    self.ram_acc_tuning = False
    self.driver_pause_frames = 0
    self.tap_frame = 0  # frames into the current tap, 0 between taps
    self.tap_button = SendButtonState.none
    self.tap_idle_frames = 0
    self.tap_start_cluster: int | None = None
    self.tap_unanswered = 0
    self.tap_wait_frames = 0

  def read_params(self, params) -> None:
    self.ram_acc_tuning = self.ram_acc_tuning_available and params.get_bool(RAM_ACC_TUNING_PARAM)

  @property
  def v_cruise_equal(self) -> bool:
    return self.v_target == self.v_cruise_cluster

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    ms_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    self.v_target_ms_last = apply_hysteresis(LP_SP.vTarget, self.v_target_ms_last, HYST_GAP * ms_conv)

    self.v_target = round(self.v_target_ms_last * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_cruise_equal:
              self.state = State.holding

            elif self.v_target > self.v_cruise_cluster:
              self.state = State.increasing

            elif self.v_target < self.v_cruise_cluster and self.v_cruise_cluster > self.v_cruise_min:
              self.state = State.decreasing

        # HOLDING
        elif self.state == State.holding:
          if not self.v_cruise_equal:
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          if self.v_target <= self.v_cruise_cluster:
            self.state = State.holding

        # DECELERATING
        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    # INACTIVE
    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    send_button = SEND_BUTTONS.get(self.state, SendButtonState.none)
    if self.ram_acc_tuning:
      send_button = self.update_taps(send_button)

    return send_button

  def update_taps(self, want: custom.IntelligentCruiseButtonManagement.SendButtonState) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    """Turns a wanted change into taps: TAP_PRESS pressed, TAP_RELEASE released. Each tap starts only after the last
    one's release, and is judged at the next tap's start: if TAP_UNANSWERED taps in a row did not move the set speed,
    tapping waits TAP_RETRY."""
    if self.tap_frame == 0:
      self.tap_wait_frames = max(0, self.tap_wait_frames - 1)
      if want == SendButtonState.none or self.tap_wait_frames > 0:
        self.tap_idle_frames += 1
        if self.tap_idle_frames > int((TAP_PRESS + TAP_RELEASE) / DT_CTRL):
          self.tap_start_cluster = None
          self.tap_unanswered = 0
        return SendButtonState.none

      answered = self.tap_start_cluster is None or self.v_cruise_cluster != self.tap_start_cluster
      self.tap_unanswered = 0 if answered else self.tap_unanswered + 1
      if self.tap_unanswered >= TAP_UNANSWERED:
        self.tap_wait_frames = int(TAP_RETRY / DT_CTRL)
        self.tap_start_cluster = None
        self.tap_unanswered = 0
        return SendButtonState.none
      self.tap_start_cluster = self.v_cruise_cluster
      self.tap_button = want
      self.tap_idle_frames = 0

    self.tap_frame += 1
    pressing = self.tap_frame <= int(TAP_PRESS / DT_CTRL)
    if self.tap_frame >= int((TAP_PRESS + TAP_RELEASE) / DT_CTRL):
      self.tap_frame = 0
    return self.tap_button if pressing and want == self.tap_button else SendButtonState.none

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    if self.ram_acc_tuning and (button_pressed or CS.gasPressed or CS.brakePressed or CC.cruiseControl.override):
      self.driver_pause_frames = int(DRIVER_PAUSE / DT_CTRL)
    elif self.driver_pause_frames > 0:
      self.driver_pause_frames -= 1
    paused = self.ram_acc_tuning and self.driver_pause_frames > 0

    self.is_ready = ready and not button_pressed and not paused

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    self.is_metric = is_metric

    self.update_calculations(CS, LP_SP)
    self.update_readiness(CS, CC)

    self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
