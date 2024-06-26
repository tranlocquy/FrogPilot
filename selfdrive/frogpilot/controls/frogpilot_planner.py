import numpy as np

import cereal.messaging as messaging

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params

from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import A_CHANGE_COST, COMFORT_BRAKE, DANGER_ZONE_COST, J_EGO_COST, STOP_DISTANCE, \
                                                                           get_jerk_factor, get_safe_obstacle_distance, get_stopped_equivalence_factor, get_T_FOLLOW
from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MIN, get_max_accel

from openpilot.selfdrive.frogpilot.controls.lib.conditional_experimental_mode import ConditionalExperimentalMode
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_functions import calculate_lane_width, calculate_road_curvature
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED

class FrogPilotPlanner:
  def __init__(self):
    self.params_memory = Params("/dev/shm/params")

    self.cem = ConditionalExperimentalMode()

    self.slower_lead = False
    self.tracking_lead = False

    self.acceleration_jerk = 0
    self.danger_jerk = 0
    self.road_curvature = 0
    self.speed_jerk = 0

  def update(self, carState, controlsState, frogpilotCarControl, frogpilotCarState, frogpilotNavigation, modelData, radarState, frogpilot_toggles):
    self.lead_one = radarState.leadOne

    v_cruise_kph = min(controlsState.vCruise, V_CRUISE_UNSET)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    v_ego = max(carState.vEgo, 0)
    v_lead = self.lead_one.vLead

    lead_distance = self.lead_one.dRel
    stopping_distance = STOP_DISTANCE

    if controlsState.experimentalMode:
      self.max_accel = ACCEL_MAX
    else:
      self.max_accel = get_max_accel(v_ego)

    if controlsState.experimentalMode:
      self.min_accel = ACCEL_MIN
    else:
      self.min_accel = A_CRUISE_MIN

    self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
      frogpilot_toggles.aggressive_jerk_acceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed,
      frogpilot_toggles.standard_jerk_acceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed,
      frogpilot_toggles.relaxed_jerk_acceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed,
      frogpilot_toggles.custom_personalities, controlsState.personality
    )

    self.t_follow = get_T_FOLLOW(
      frogpilot_toggles.custom_personalities, frogpilot_toggles.aggressive_follow, frogpilot_toggles.standard_follow,
      frogpilot_toggles.relaxed_follow, controlsState.personality
    )

    if self.tracking_lead:
      self.update_follow_values(lead_distance, stopping_distance, v_ego, v_lead, frogpilot_toggles)
    else:
      self.acceleration_jerk = self.base_acceleration_jerk
      self.danger_jerk = self.base_danger_jerk
      self.speed_jerk = self.base_speed_jerk

    check_lane_width = frogpilot_toggles.lane_detection
    if check_lane_width and v_ego >= frogpilot_toggles.minimum_lane_change_speed:
      self.lane_width_left = float(calculate_lane_width(modelData.laneLines[0], modelData.laneLines[1], modelData.roadEdges[0]))
      self.lane_width_right = float(calculate_lane_width(modelData.laneLines[3], modelData.laneLines[2], modelData.roadEdges[1]))
    else:
      self.lane_width_left = 0
      self.lane_width_right = 0

    self.road_curvature = calculate_road_curvature(modelData, v_ego)
    self.v_cruise = self.update_v_cruise(carState, controlsState, frogpilotCarState, frogpilotNavigation, modelData, v_cruise, v_ego, frogpilot_toggles)

    if frogpilot_toggles.conditional_experimental_mode and controlsState.enabled:
      self.cem.update(carState, frogpilotNavigation, self.lead_one, modelData, self.road_curvature, self.slower_lead, self.tracking_lead, v_ego, v_lead, frogpilot_toggles)

    if v_ego > CRUISING_SPEED:
      self.tracking_lead = self.lead_one.status

  def update_follow_values(self, lead_distance, stopping_distance, v_ego, v_lead, frogpilot_toggles):
    # Offset by FrogAi for FrogPilot for a more natural approach to a slower lead
    if frogpilot_toggles.conditional_experimental_mode and v_lead < v_ego:
      distance_factor = max(lead_distance - (v_lead * self.t_follow), 1)
      braking_offset = np.clip((v_ego - v_lead) - COMFORT_BRAKE, 1, distance_factor)
      self.slower_lead = max(braking_offset, 1) > 1

  def update_v_cruise(self, carState, controlsState, frogpilotCarState, frogpilotNavigation, modelData, v_cruise, v_ego, frogpilot_toggles):
    v_cruise_cluster = max(controlsState.vCruiseCluster, v_cruise) * CV.KPH_TO_MS
    v_cruise_diff = v_cruise_cluster - v_cruise

    v_ego_cluster = max(carState.vEgoCluster, v_ego)
    v_ego_diff = v_ego_cluster - v_ego

    targets = []
    filtered_targets = [target if target > CRUISING_SPEED else v_cruise for target in targets]

    return min(filtered_targets)

  def publish(self, sm, pm, frogpilot_toggles):
    frogpilot_plan_send = messaging.new_message('frogpilotPlan')
    frogpilot_plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])
    frogpilotPlan = frogpilot_plan_send.frogpilotPlan

    frogpilotPlan.accelerationJerk = A_CHANGE_COST * float(self.acceleration_jerk)
    frogpilotPlan.accelerationJerkStock = A_CHANGE_COST * float(self.base_acceleration_jerk)
    frogpilotPlan.dangerJerk = DANGER_ZONE_COST * float(self.danger_jerk)
    frogpilotPlan.speedJerk = J_EGO_COST * float(self.speed_jerk)
    frogpilotPlan.speedJerkStock = J_EGO_COST * float(self.base_speed_jerk)
    frogpilotPlan.tFollow = float(self.t_follow)

    frogpilotPlan.conditionalExperimentalActive = self.cem.experimental_mode

    frogpilotPlan.laneWidthLeft = self.lane_width_left
    frogpilotPlan.laneWidthRight = self.lane_width_right

    frogpilotPlan.maxAcceleration = self.max_accel
    frogpilotPlan.minAcceleration = self.min_accel

    frogpilotPlan.vCruise = float(self.v_cruise)

    pm.send('frogpilotPlan', frogpilot_plan_send)
