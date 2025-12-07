#!/usr/bin/env python3
from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.common.constants import CV
from openpilot.system.hardware import HARDWARE, PC


def frogpilot_boot_functions():


def install_frogpilot():
  paths = [
  ]
  for path in paths:
    path.mkdir(parents=True, exist_ok=True)

  update_boot_logo(frogpilot_variables.FROGPILOT_BOOT_LOGO_PATH)


def migrate_params(params, params_cache):
  param_types = {
    "MaxLateralAcceleration": dict,
  }

  for key, expected_type in param_types.items():
    for param_store in (params, params_cache):
      value = param_store.get(key)

      if value is not None and not isinstance(value, expected_type):
        param_store.remove(key)

  if params.get_bool("IsMetric"):
    metric_conversions = {
      "CESignalSpeed": (CV.MPH_TO_KPH, 0),
      "LaneLinesWidth": (CV.INCH_TO_CM, 0),
      "MinimumLaneChangeSpeed": (CV.MPH_TO_KPH, 0),
      "Offset1": (CV.MPH_TO_KPH, 0),
      "Offset2": (CV.MPH_TO_KPH, 0),
      "Offset3": (CV.MPH_TO_KPH, 0),
      "Offset4": (CV.MPH_TO_KPH, 0),
      "Offset5": (CV.MPH_TO_KPH, 0),
      "Offset6": (CV.MPH_TO_KPH, 0),
      "Offset7": (CV.MPH_TO_KPH, 0),
      "PathWidth": (CV.FOOT_TO_METER, 1),
      "RoadEdgesWidth": (CV.INCH_TO_CM, 0),
    }

    for key, (conversion, decimals) in metric_conversions.items():
      if params.get(key) is None and params_cache.get(key) is None:
        params.put(key, round(params.get_default_value(key) * conversion, decimals))


def sync_params_cache(params, params_cache):
  for key in params.all_keys():
    current_value = params.get(key)
    cached_value = params_cache.get(key)

    if current_value is None:
      if cached_value is not None:
        params.put(key, cached_value)
    elif current_value != cached_value:
      params_cache.put(key, current_value)


def uninstall_frogpilot():
  update_boot_logo(frogpilot_variables.STOCK_BOOT_LOGO_PATH)

  HARDWARE.uninstall()


def update_boot_logo(target_logo):
  if PC:
    return

  if frogpilot_variables.BOOT_LOGO_PATH.read_bytes() != target_logo.read_bytes():
    frogpilot_utilities.run_cmd_on_writable_mount("/", ["sudo", "cp", target_logo, frogpilot_variables.BOOT_LOGO_PATH], "Failed to replace boot logo")
