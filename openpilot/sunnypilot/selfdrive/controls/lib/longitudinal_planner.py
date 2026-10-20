"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import messaging, custom, log
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.lead_speed_calibration import PARAM as LEAD_CAL_PARAM, LeadSpeedCalibration
from openpilot.sunnypilot.selfdrive.controls.lib.planned_speed import PlannedSpeed, RAM_ACC_TUNING_PARAM, ram_acc_tuning_available
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
LaneChangeState = log.LaneChangeState

PLANNED_SPEED_PARAMS_PERIOD = 1.  # s, Custom Ram ACC Tuning takes effect within this, on or off
LEAD_CAL_SAVE_PERIOD = 60.  # s, the learned camera correction is saved at most this often


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.

    # Planned-speed target for Intelligent Cruise Button Management (Custom Ram ACC Tuning). It only changes the target
    # published to the cruise buttons (vTarget); the planner's own plan above keeps the usual cruise speed.
    self.params = Params()
    self.lead_cal = None
    self.planned_speed = None
    if ram_acc_tuning_available(CP, CP_SP):
      self.lead_cal = LeadSpeedCalibration(model_bundle.ref if model_bundle else "default", self.params.get(LEAD_CAL_PARAM))
      self.planned_speed = PlannedSpeed(calibration=self.lead_cal)
    self.planned_speed_enabled = False
    self.planned_speed_frame = 0
    self.icbm_v_target = 0.
    self.icbm_source = LongitudinalPlanSource.cruise

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
      return experimental_mode

    return experimental_mode and self.dec.mode() == "blended"

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    return self.output_v_target, self.output_a_target

  def update_planned_speed(self, sm: messaging.SubMaster, v_plan, a_plan, allow_throttle: bool) -> None:
    """After the plan: the target for the cruise buttons, from the plan and the car ahead (planned_speed.py), never
    above the driver's set speed or the curve and speed-limit targets. During a soft-disable or a driver-monitoring
    slowdown the planner's cruise speed is 0; the buttons then follow the truck down as it coasts instead."""
    self.icbm_v_target, self.icbm_source = self.output_v_target, self.source
    if self.planned_speed is None or self.lead_cal is None:
      return

    CS = sm['carState']
    CC = sm['carControl']
    lead = sm['radarState'].leadOne
    lane_change = CS.leftBlinker != CS.rightBlinker or sm['modelV2'].meta.laneChangeState != LaneChangeState.off

    # The camera correction learns from steady following whether or not the toggle is on: it only observes
    self.lead_cal.update(CS.vEgo, lead.present, lead.dRel, lead.vLead, lane_change)
    if self.lead_cal.changed and self.planned_speed_frame % int(LEAD_CAL_SAVE_PERIOD / DT_MDL) == 0:
      self.params.put(LEAD_CAL_PARAM, self.lead_cal.to_param())  # not blocking
      self.lead_cal.changed = False

    if self.planned_speed_frame % int(PLANNED_SPEED_PARAMS_PERIOD / DT_MDL) == 0:
      self.planned_speed_enabled = self.params.get_bool(RAM_ACC_TUNING_PARAM)
    self.planned_speed_frame += 1
    if not self.planned_speed_enabled:
      self.planned_speed.reset()
      return
    if not CC.enabled or CC.cruiseControl.override:
      self.planned_speed.reset()
    v_planned = self.planned_speed.update(CS.vEgo, v_plan, a_plan, allow_throttle, lead.present, lead.dRel, lead.vLead,
                                          lane_change, sm['controlsState'].forceDecel)

    ceilings = {
      LongitudinalPlanSource.cruise: min(CS.vCruise, V_CRUISE_MAX) * CV.KPH_TO_MS,
      LongitudinalPlanSource.sccVision: self.scc.vision.output_v_target,
      LongitudinalPlanSource.sccMap: self.scc.map.output_v_target,
      LongitudinalPlanSource.speedLimitAssist: self.sla.output_v_target,
    }
    source = min(ceilings, key=lambda k: ceilings[k])
    if v_planned < ceilings[source]:
      self.icbm_v_target, self.icbm_source = v_planned, LongitudinalPlanSource.plannedSpeed
    else:
      self.icbm_v_target, self.icbm_source = ceilings[source], source

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self.dec.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.icbm_source
    longitudinalPlanSP.vTarget = float(self.icbm_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)
