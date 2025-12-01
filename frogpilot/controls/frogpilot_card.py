#!/usr/bin/env python3
import cereal.messaging as messaging

from cereal import log
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.selfdrive.car.cruise import ButtonType

from openpilot.frogpilot.common import frogpilot_utilities, frogpilot_variables

EventName = log.OnroadEvent.EventName

class FrogPilotCard:
  def __init__(self, CP):
    self.CP = CP

    self.always_on_lateral_allowed = False
    self.cruise_available_previously = True

    self.accel_press_count = 0
    self.decel_press_count = 0
    self.lkas_button_press_count = 0

    self.always_on_lateral_set = bool(CP.alternativeExperience & ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL)

    self.ui_event_sock = messaging.sub_sock("frogpilotUIEvent")

  def update(self, carState, frogpilotCarState, sm, frogpilot_toggles):
    if self.CP.brand == "hyundai":
      for be in carState.buttonEvents:
        if be.type == ButtonType.lkas and be.pressed and frogpilot_toggles.always_on_lateral_lkas:
          self.always_on_lateral_allowed = not self.always_on_lateral_allowed
        elif be.type == ButtonType.mainCruise and be.pressed and frogpilot_toggles.always_on_lateral_main:
          self.always_on_lateral_allowed = not self.always_on_lateral_allowed
    elif frogpilot_toggles.always_on_lateral_main:
      self.always_on_lateral_allowed |= sm["carControl"].enabled or not self.cruise_available_previously
      self.always_on_lateral_allowed &= carState.cruiseState.available

      if carState.canValid:
        self.cruise_available_previously = carState.cruiseState.available

    if not frogpilot_toggles.always_on_lateral_lkas and not frogpilot_toggles.always_on_lateral_main:
      self.always_on_lateral_allowed = carState.cruiseState.enabled

    self.always_on_lateral_enabled = self.always_on_lateral_allowed and self.always_on_lateral_set
    self.always_on_lateral_enabled &= carState.gearShifter not in frogpilot_variables.NON_DRIVING_GEARS
    self.always_on_lateral_enabled &= sm["liveCalibration"].calPerc >= 1
    self.always_on_lateral_enabled &= not sm["frogpilotSelfdriveState"].hasDisableEvents
    self.always_on_lateral_enabled &= not any(event.name == EventName.tooDistracted for event in sm["onroadEvents"])
    self.always_on_lateral_enabled &= not (carState.brakePressed and carState.vEgo < frogpilot_toggles.always_on_lateral_pause_speed) or carState.standstill

    if any(be.pressed and be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in carState.buttonEvents):
      self.accel_press_count += 1

    if any(be.pressed and be.type == ButtonType.decelCruise for be in carState.buttonEvents):
      self.decel_press_count += 1

    if any(be.pressed and be.type == ButtonType.lkas for be in carState.buttonEvents):
      self.lkas_button_press_count += 1

    frogpilotCarState.accelPressCount = self.accel_press_count
    frogpilotCarState.alwaysOnLateralAllowed = self.always_on_lateral_allowed
    frogpilotCarState.alwaysOnLateralEnabled = self.always_on_lateral_enabled
    frogpilotCarState.decelPressCount = self.decel_press_count
    frogpilotCarState.lkasButtonPressCount = self.lkas_button_press_count

    return frogpilotCarState
