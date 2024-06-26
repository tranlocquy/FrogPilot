"""Install exception handler for process crash."""
import http.client
import os
import sentry_sdk
import socket
import time
import traceback
import urllib.request
import urllib.error

from datetime import datetime
from enum import Enum
from sentry_sdk.integrations.threading import ThreadingIntegration

from openpilot.common.params import Params, ParamKeyType
from openpilot.system.athena.registration import is_registered_device
from openpilot.system.hardware import HARDWARE, PC
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_build_metadata, get_version

CRASHES_DIR = "/data/community/crashes/"

class SentryProject(Enum):
  # python project
  SELFDRIVE = "https://5ad1714d27324c74a30f9c538bff3b8d@o4505034923769856.ingest.sentry.io/4505034930651136"
  # native project
  SELFDRIVE_NATIVE = "https://5ad1714d27324c74a30f9c538bff3b8d@o4505034923769856.ingest.sentry.io/4505034930651136"


def sentry_pinged(url="https://sentry.io", timeout=5):
  try:
    urllib.request.urlopen(url, timeout=timeout)
    return True
  except (urllib.error.URLError, socket.timeout, http.client.RemoteDisconnected):
    return False


def bind_user() -> None:
  sentry_sdk.set_user({"id": HARDWARE.get_serial()})


def report_tombstone(fn: str, message: str, contents: str) -> None:
  FrogPilot = "frogai" in get_build_metadata().openpilot.git_origin.lower()
  if not FrogPilot or PC:
    return

  no_internet = 0
  while True:
    if sentry_pinged():
      cloudlog.error({'tombstone': message})

      with sentry_sdk.configure_scope() as scope:
        bind_user()
        scope.set_extra("tombstone_fn", fn)
        scope.set_extra("tombstone", contents)
        sentry_sdk.capture_message(message=message)
        sentry_sdk.flush()
      break
    elif no_internet > 10:
      break
    else:
      no_internet += 1
      time.sleep(no_internet * 60)


def capture_fingerprint(candidate, params, blocked=False):
  bind_user()

  param_types = {
    "FrogPilot Controls": ParamKeyType.FROGPILOT_CONTROLS,
    "FrogPilot Vehicles": ParamKeyType.FROGPILOT_VEHICLES,
    "FrogPilot Visuals": ParamKeyType.FROGPILOT_VISUALS,
    "FrogPilot Other": ParamKeyType.FROGPILOT_OTHER,
    "FrogPilot Tracking": ParamKeyType.FROGPILOT_TRACKING
  }

  matched_params = {label: {} for label in param_types}
  for key in params.all_keys():
    for label, key_type in param_types.items():
      if params.get_key_type(key) & key_type:
        matched_params[label][key] = params.get(key)

  no_internet = 0
  while True:
    if sentry_pinged():
      with sentry_sdk.configure_scope() as scope:
        scope.fingerprint = [candidate, HARDWARE.get_serial()]

        for label, key_values in matched_params.items():
          formatted_params = "\n".join([f"{key}: {value}" for key, value in key_values.items()])
          scope.set_extra(label, formatted_params)

      if blocked:
        sentry_sdk.capture_message("Blocked user from using the development branch", level='error')
      else:
        sentry_sdk.capture_message(f"Fingerprinted {candidate}", level='info')
        params.put_bool("FingerprintLogged", True)

      sentry_sdk.flush()
      break
    elif no_internet > 10:
      break
    else:
      no_internet += 1
      time.sleep(no_internet * 60)


def capture_exception(*args, **kwargs) -> None:
  exc_text = traceback.format_exc()

  phrases_to_check = [
    "To overwrite it, set 'overwrite' to True.",
  ]

  if any(phrase in exc_text for phrase in phrases_to_check):
    return

  save_exception(exc_text)
  cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  FrogPilot = "frogai" in get_build_metadata().openpilot.git_origin.lower()
  if not FrogPilot or PC:
    return

  try:
    bind_user()
    sentry_sdk.capture_exception(*args, **kwargs)
    sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
  except Exception:
    cloudlog.exception("sentry exception")


def save_exception(exc_text: str) -> None:
  if not os.path.exists(CRASHES_DIR):
    os.makedirs(CRASHES_DIR)

  files = [
    os.path.join(CRASHES_DIR, datetime.now().strftime('%Y-%m-%d--%H-%M-%S.log')),
    os.path.join(CRASHES_DIR, 'error.txt')
  ]

  for file in files:
    with open(file, 'w') as f:
      if file.endswith("error.txt"):
        lines = exc_text.splitlines()[-10:]
        f.write("\n".join(lines))
      else:
        f.write(exc_text)

  print('Logged current crash to {}'.format(files))


def set_tag(key: str, value: str) -> None:
  sentry_sdk.set_tag(key, value)


def init(project: SentryProject) -> bool:
  build_metadata = get_build_metadata()
  if not is_registered_device() or PC:
    return False

  params = Params()
  installed = params.get("InstallDate", encoding='utf-8')
  updated = params.get("Updated", encoding='utf-8')

  short_branch = build_metadata.channel

  if short_branch == "FrogPilot-Development":
    env = "Development"
  elif short_branch in {"FrogPilot-Staging", "FrogPilot-Testing"}:
    env = "Staging"
  elif short_branch == "FrogPilot":
    env = "Release"
  else:
    env = short_branch

  integrations = []
  if project == SentryProject.SELFDRIVE:
    integrations.append(ThreadingIntegration(propagate_hub=True))

  sentry_sdk.init(project.value,
                  default_integrations=False,
                  release=get_version(),
                  integrations=integrations,
                  traces_sample_rate=1.0,
                  max_value_length=8192,
                  environment=env)

  build_metadata = get_build_metadata()

  sentry_sdk.set_user({"id": HARDWARE.get_serial()})
  sentry_sdk.set_tag("origin", build_metadata.openpilot.git_origin)
  sentry_sdk.set_tag("branch", short_branch)
  sentry_sdk.set_tag("commit", build_metadata.openpilot.git_commit)
  sentry_sdk.set_tag("updated", updated)
  sentry_sdk.set_tag("installed", installed)

  if project == SentryProject.SELFDRIVE:
    sentry_sdk.Hub.current.start_session()

  return True
