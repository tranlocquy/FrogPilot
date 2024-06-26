from openpilot.common.params import Params
from openpilot.selfdrive.modeld.constants import ModelConstants

from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_functions import MovingAverageCalculator
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_variables import CITY_SPEED_LIMIT, PROBABILITY

class ConditionalExperimentalMode:
  def __init__(self):
    self.params_memory = Params("/dev/shm/params")

    self.curve_detected = False
    self.experimental_mode = False

    self.curvature_mac = MovingAverageCalculator()
    self.slow_lead_mac = MovingAverageCalculator()
    self.stop_light_mac = MovingAverageCalculator()

  def update(self, carState, frogpilotNavigation, lead, modelData, road_curvature, slower_lead, tracking_lead, v_ego, v_lead, frogpilot_toggles):
    if not carState.standstill:
      self.update_conditions(lead.dRel, modelData, road_curvature, slower_lead, tracking_lead, v_ego, v_lead, frogpilot_toggles)
      self.experimental_mode = self.check_conditions(carState, frogpilotNavigation, modelData, tracking_lead, v_ego, v_lead, frogpilot_toggles)
      self.params_memory.put_int("CEStatus", self.status_value if self.experimental_mode else 0)
    else:
      self.experimental_mode = carState.standstill and self.experimental_mode

  def check_conditions(self, carState, frogpilotNavigation, modelData, tracking_lead, v_ego, v_lead, frogpilot_toggles):
    if (tracking_lead and v_ego <= frogpilot_toggles.conditional_limit_lead) or (not tracking_lead and v_ego <= frogpilot_toggles.conditional_limit):
      self.status_value = 7 if tracking_lead else 8
      return True

    approaching_maneuver = modelData.navEnabled and (frogpilotNavigation.approachingIntersection or frogpilotNavigation.approachingTurn)
    if frogpilot_toggles.conditional_navigation and approaching_maneuver and (frogpilot_toggles.conditional_navigation_lead or not tracking_lead):
      self.status_value = 10 if frogpilotNavigation.approachingIntersection else 11
      return True

    if frogpilot_toggles.conditional_curves and self.curve_detected:
      self.status_value = 12
      return True

    if frogpilot_toggles.conditional_lead and self.slow_lead_detected:
      self.status_value = 13 if v_lead < 1 else 14
      return True

    if frogpilot_toggles.conditional_stop_lights and self.stop_light_detected:
      self.status_value = 15
      return True

    return False

  def update_conditions(self, lead_distance, modelData, road_curvature, slower_lead, tracking_lead, v_ego, v_lead, frogpilot_toggles):
    self.road_curvature(road_curvature, tracking_lead, v_ego, frogpilot_toggles)
    self.slow_lead(slower_lead, tracking_lead, v_lead, frogpilot_toggles)
    self.stop_sign_and_light(lead_distance, modelData, tracking_lead, v_ego, v_lead, frogpilot_toggles)

  def road_curvature(self, road_curvature, tracking_lead, v_ego, frogpilot_toggles):
    curve_detected = (1 / road_curvature)**0.5 < v_ego and (frogpilot_toggles.conditional_curves_lead or not tracking_lead)
    curve_active = (0.9 / road_curvature)**0.5 < v_ego and self.curve_detected

    self.curvature_mac.add_data(curve_detected or curve_active)
    self.curve_detected = self.curvature_mac.get_moving_average() >= PROBABILITY

  def slow_lead(self, slower_lead, tracking_lead, v_lead, frogpilot_toggles):
    if tracking_lead:
      slower_lead &= frogpilot_toggles.conditional_slower_lead
      stopped_lead = frogpilot_toggles.conditional_stopped_lead and v_lead < 1

      self.slow_lead_mac.add_data(slower_lead or stopped_lead)
      self.slow_lead_detected = self.slow_lead_mac.get_moving_average() >= PROBABILITY
    else:
      self.slow_lead_mac.reset_data()
      self.slow_lead_detected = False

  def stop_sign_and_light(self, lead_distance, modelData, tracking_lead, v_ego, v_lead, frogpilot_toggles):
    model_length = modelData.position.x[ModelConstants.IDX_N - 1]

    lead_close = lead_distance < CITY_SPEED_LIMIT
    lead_stopped = v_lead < 1
    lead_stopping = lead_distance < model_length
    following_lead = tracking_lead and (lead_close or lead_stopped or lead_stopping)

    model_stopped = model_length < ModelConstants.IDX_N
    model_threshold = v_ego * (ModelConstants.T_IDXS[ModelConstants.IDX_N - (5 if frogpilot_toggles.less_sensitive_lights else 3)])
    model_stopping = model_length < model_threshold

    self.stop_light_mac.add_data(not following_lead and (model_stopped or model_stopping))
    self.stop_light_detected = self.stop_light_mac.get_moving_average() >= PROBABILITY
