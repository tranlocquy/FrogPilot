#include "selfdrive/ui/ui.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include <QtConcurrent>

#include "common/transformations/orientation.hpp"
#include "common/params.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "common/watchdog.h"
#include "system/hardware/hw.h"

#define BACKLIGHT_DT 0.05
#define BACKLIGHT_TS 10.00

// Projects a point in car to space to the corresponding point in full frame
// image space.
static bool calib_frame_to_full_frame(const UIState *s, float in_x, float in_y, float in_z, QPointF *out) {
  const float margin = 500.0f;
  const QRectF clip_region{-margin, -margin, s->fb_w + 2 * margin, s->fb_h + 2 * margin};

  const vec3 pt = (vec3){{in_x, in_y, in_z}};
  const vec3 Ep = matvecmul3(s->scene.wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib, pt);
  const vec3 KEp = matvecmul3(s->scene.wide_cam ? ECAM_INTRINSIC_MATRIX : FCAM_INTRINSIC_MATRIX, Ep);

  // Project.
  QPointF point = s->car_space_transform.map(QPointF{KEp.v[0] / KEp.v[2], KEp.v[1] / KEp.v[2]});
  if (clip_region.contains(point)) {
    *out = point;
    return true;
  }
  return false;
}

int get_path_length_idx(const cereal::XYZTData::Reader &line, const float path_height) {
  const auto line_x = line.getX();
  int max_idx = 0;
  for (int i = 1; i < line_x.size() && line_x[i] <= path_height; ++i) {
    max_idx = i;
  }
  return max_idx;
}

void update_leads(UIState *s, const cereal::ModelDataV2::Reader &model_data) {
  const cereal::XYZTData::Reader &line = model_data.getPosition();
  for (int i = 0; i < model_data.getLeadsV3().size() && i < 2; ++i) {
    const auto &lead = model_data.getLeadsV3()[i];
    if (s->scene.has_lead) {
      float d_rel = lead.getX()[0];
      float y_rel = lead.getY()[0];
      float z = line.getZ()[get_path_length_idx(line, d_rel)];
      calib_frame_to_full_frame(s, d_rel, y_rel, z + 1.22, &s->scene.lead_vertices[i]);
    }
  }
}

void update_line_data(const UIState *s, const cereal::XYZTData::Reader &line,
                      float y_off, float z_off, QPolygonF *pvd, int max_idx, bool allow_invert=true) {
  const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
  QPointF left, right;
  pvd->clear();
  for (int i = 0; i <= max_idx; i++) {
    // highly negative x positions  are drawn above the frame and cause flickering, clip to zy plane of camera
    if (line_x[i] < 0) continue;

    bool l = calib_frame_to_full_frame(s, line_x[i], line_y[i] - y_off, line_z[i] + z_off, &left);
    bool r = calib_frame_to_full_frame(s, line_x[i], line_y[i] + y_off, line_z[i] + z_off, &right);
    if (l && r) {
      // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
      if (!allow_invert && pvd->size() && left.y() > pvd->back().y()) {
        continue;
      }
      pvd->push_back(left);
      pvd->push_front(right);
    }
  }
}

void update_model(UIState *s,
                  const cereal::ModelDataV2::Reader &model,
                  const cereal::UiPlan::Reader &plan) {
  UIScene &scene = s->scene;
  auto plan_position = plan.getPosition();
  if (plan_position.getX().size() < model.getPosition().getX().size()) {
    plan_position = model.getPosition();
  }
  float max_distance = std::clamp(*(plan_position.getX().end() - 1),
                                  MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

  // update lane lines
  const auto lane_lines = model.getLaneLines();
  const auto lane_line_probs = model.getLaneLineProbs();
  int max_idx = get_path_length_idx(lane_lines[0], max_distance);
  for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
    scene.lane_line_probs[i] = lane_line_probs[i];
    update_line_data(s, lane_lines[i], 0.025 * scene.lane_line_probs[i], 0, &scene.lane_line_vertices[i], max_idx);
  }

  // update road edges
  const auto road_edges = model.getRoadEdges();
  const auto road_edge_stds = model.getRoadEdgeStds();
  for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
    scene.road_edge_stds[i] = road_edge_stds[i];
    update_line_data(s, road_edges[i], 0.025, 0, &scene.road_edge_vertices[i], max_idx);
  }

  // Update adjacent paths
  for (int i = 4; i <= 5; i++) {
    update_line_data(s, lane_lines[i], (i == 4 ? scene.lane_width_left : scene.lane_width_right) / 2.0f, 0, &scene.track_adjacent_vertices[i], max_idx, false);
  }

  // update path
  auto lead_count = model.getLeadsV3().size();
  if (lead_count > 0) {
    auto lead_one = model.getLeadsV3()[0];
    scene.has_lead = lead_one.getProb() > scene.lead_detection_threshold;
    if (scene.has_lead) {
      const float lead_d = lead_one.getX()[0] * 2.;
      max_distance = std::clamp((float)(lead_d - fmin(lead_d * 0.35, 10.)), 0.0f, max_distance);
    }
  }
  max_idx = get_path_length_idx(plan_position, max_distance);
  update_line_data(s, plan_position, 0.9, 1.22, &scene.track_vertices, max_idx, false);
}

void update_dmonitoring(UIState *s, const cereal::DriverStateV2::Reader &driverstate, float dm_fade_state, bool is_rhd) {
  UIScene &scene = s->scene;
  const auto driver_orient = is_rhd ? driverstate.getRightDriverData().getFaceOrientation() : driverstate.getLeftDriverData().getFaceOrientation();
  for (int i = 0; i < std::size(scene.driver_pose_vals); i++) {
    float v_this = (i == 0 ? (driver_orient[i] < 0 ? 0.7 : 0.9) : 0.4) * driver_orient[i];
    scene.driver_pose_diff[i] = fabs(scene.driver_pose_vals[i] - v_this);
    scene.driver_pose_vals[i] = 0.8 * v_this + (1 - 0.8) * scene.driver_pose_vals[i];
    scene.driver_pose_sins[i] = sinf(scene.driver_pose_vals[i]*(1.0-dm_fade_state));
    scene.driver_pose_coss[i] = cosf(scene.driver_pose_vals[i]*(1.0-dm_fade_state));
  }

  auto [sin_y, sin_x, sin_z] = scene.driver_pose_sins;
  auto [cos_y, cos_x, cos_z] = scene.driver_pose_coss;

  const mat3 r_xyz = (mat3){{
    cos_x * cos_z,
    cos_x * sin_z,
    -sin_x,

    -sin_y * sin_x * cos_z - cos_y * sin_z,
    -sin_y * sin_x * sin_z + cos_y * cos_z,
    -sin_y * cos_x,

    cos_y * sin_x * cos_z - sin_y * sin_z,
    cos_y * sin_x * sin_z + sin_y * cos_z,
    cos_y * cos_x,
  }};

  // transform vertices
  for (int kpi = 0; kpi < std::size(default_face_kpts_3d); kpi++) {
    vec3 kpt_this = matvecmul3(r_xyz, default_face_kpts_3d[kpi]);
    scene.face_kpts_draw[kpi] = (vec3){{kpt_this.v[0], kpt_this.v[1], (float)(kpt_this.v[2] * (1.0-dm_fade_state) + 8 * dm_fade_state)}};
  }

  scene.right_hand_drive = is_rhd;
}

static void update_sockets(UIState *s) {
  s->sm->update(0);
}

static void update_state(UIState *s) {
  SubMaster &sm = *(s->sm);
  UIScene &scene = s->scene;

  if (sm.updated("liveCalibration")) {
    auto live_calib = sm["liveCalibration"].getLiveCalibration();
    auto rpy_list = live_calib.getRpyCalib();
    auto wfde_list = live_calib.getWideFromDeviceEuler();
    Eigen::Vector3d rpy;
    Eigen::Vector3d wfde;
    if (rpy_list.size() == 3) rpy << rpy_list[0], rpy_list[1], rpy_list[2];
    if (wfde_list.size() == 3) wfde << wfde_list[0], wfde_list[1], wfde_list[2];
    Eigen::Matrix3d device_from_calib = euler2rot(rpy);
    Eigen::Matrix3d wide_from_device = euler2rot(wfde);
    Eigen::Matrix3d view_from_device;
    view_from_device << 0, 1, 0,
                        0, 0, 1,
                        1, 0, 0;
    Eigen::Matrix3d view_from_calib = view_from_device * device_from_calib;
    Eigen::Matrix3d view_from_wide_calib = view_from_device * wide_from_device * device_from_calib;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        scene.view_from_calib.v[i*3 + j] = view_from_calib(i, j);
        scene.view_from_wide_calib.v[i*3 + j] = view_from_wide_calib(i, j);
      }
    }
    scene.calibration_valid = live_calib.getCalStatus() == cereal::LiveCalibrationData::Status::CALIBRATED;
    scene.calibration_wide_valid = wfde_list.size() == 3;
  }
  if (sm.updated("pandaStates")) {
    auto pandaStates = sm["pandaStates"].getPandaStates();
    if (pandaStates.size() > 0) {
      scene.pandaType = pandaStates[0].getPandaType();

      if (scene.pandaType != cereal::PandaState::PandaType::UNKNOWN) {
        scene.ignition = false;
        for (const auto& pandaState : pandaStates) {
          scene.ignition |= pandaState.getIgnitionLine() || pandaState.getIgnitionCan();
        }
      }
    }
  } else if ((s->sm->frame - s->sm->rcv_frame("pandaStates")) > 5*UI_FREQ) {
    scene.pandaType = cereal::PandaState::PandaType::UNKNOWN;
  }
  if (sm.updated("carControl")) {
    auto carControl = sm["carControl"].getCarControl();
  }
  if (sm.updated("carParams")) {
    scene.longitudinal_control = sm["carParams"].getCarParams().getOpenpilotLongitudinalControl();
  }
  if (sm.updated("carState")) {
    auto carState = sm["carState"].getCarState();
    scene.blind_spot_left = carState.getLeftBlindspot();
    scene.blind_spot_right = carState.getRightBlindspot();
    scene.parked = carState.getGearShifter() == cereal::CarState::GearShifter::PARK;
    scene.reverse = carState.getGearShifter() == cereal::CarState::GearShifter::REVERSE;
    scene.standstill = carState.getStandstill() && !scene.reverse;
  }
  if (sm.updated("controlsState")) {
    auto controlsState = sm["controlsState"].getControlsState();
    scene.alert_size = controlsState.getAlertSize() == cereal::ControlsState::AlertSize::MID ? 350 : controlsState.getAlertSize() == cereal::ControlsState::AlertSize::SMALL ? 200 : 0;
    scene.enabled = controlsState.getEnabled();
    scene.experimental_mode = controlsState.getExperimentalMode();
  }
  if (sm.updated("deviceState")) {
    auto deviceState = sm["deviceState"].getDeviceState();
    scene.online = deviceState.getNetworkType() != cereal::DeviceState::NetworkType::NONE;
  }
  if (sm.updated("frogpilotCarControl")) {
    auto frogpilotCarControl = sm["frogpilotCarControl"].getFrogpilotCarControl();
    scene.always_on_lateral_active = !scene.enabled && frogpilotCarControl.getAlwaysOnLateral();
    scene.speed_limit_changed = scene.speed_limit_controller && frogpilotCarControl.getSpeedLimitChanged();
  }
  if (sm.updated("frogpilotCarState")) {
    auto frogpilotCarState = sm["frogpilotCarState"].getFrogpilotCarState();
    scene.brake_lights_on = frogpilotCarState.getBrakeLights();
    scene.traffic_mode_active = frogpilotCarState.getTrafficModeActive();
  }
  if (sm.updated("frogpilotPlan")) {
    auto frogpilotPlan = sm["frogpilotPlan"].getFrogpilotPlan();
    scene.adjusted_cruise = frogpilotPlan.getAdjustedCruise();
    scene.lane_width_left = frogpilotPlan.getLaneWidthLeft();
    scene.lane_width_right = frogpilotPlan.getLaneWidthRight();
    scene.speed_limit = frogpilotPlan.getSlcSpeedLimit();
    scene.speed_limit_offset = frogpilotPlan.getSlcSpeedLimitOffset();
    scene.speed_limit_overridden = frogpilotPlan.getSlcOverridden();
    scene.speed_limit_overridden_speed = frogpilotPlan.getSlcOverriddenSpeed();
    scene.unconfirmed_speed_limit = frogpilotPlan.getUnconfirmedSlcSpeedLimit();
    scene.vtsc_controlling_curve = frogpilotPlan.getVtscControllingCurve();
  }
  if (sm.updated("liveLocationKalman")) {
    auto liveLocationKalman = sm["liveLocationKalman"].getLiveLocationKalman();
    auto orientation = liveLocationKalman.getCalibratedOrientationNED();
    if (orientation.getValid()) {
      scene.bearing_deg = RAD2DEG(orientation.getValue()[2]);
    }
  }
  if (sm.updated("liveTorqueParameters")) {
    auto liveTorqueParameters = sm["liveTorqueParameters"].getLiveTorqueParameters();
  }
  if (sm.updated("wideRoadCameraState")) {
    auto cam_state = sm["wideRoadCameraState"].getWideRoadCameraState();
    float scale = (cam_state.getSensor() == cereal::FrameData::ImageSensor::AR0231) ? 6.0f : 1.0f;
    scene.light_sensor = std::max(100.0f - scale * cam_state.getExposureValPercent(), 0.0f);
  } else if (!sm.allAliveAndValid({"wideRoadCameraState"})) {
    scene.light_sensor = -1;
  }
  scene.started = sm["deviceState"].getDeviceState().getStarted() && scene.ignition;

  scene.world_objects_visible = scene.world_objects_visible ||
                                (scene.started &&
                                 sm.rcv_frame("liveCalibration") > scene.started_frame &&
                                 sm.rcv_frame("modelV2") > scene.started_frame &&
                                 sm.rcv_frame("uiPlan") > scene.started_frame);
}

void ui_update_params(UIState *s) {
  auto params = Params();
  s->scene.is_metric = params.getBool("IsMetric");
  s->scene.map_on_left = params.getBool("NavSettingLeftSide");
}

void ui_update_frogpilot_params(UIState *s) {
  UIScene &scene = s->scene;
  Params params = Params();

  auto carParams = params.get("CarParamsPersistent");
  if (!carParams.empty() && !scene.started) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(carParams.data(), carParams.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();
    scene.longitudinal_control = CP.getOpenpilotLongitudinalControl() && !params.getBool("DisableOpenpilotLongitudinal");
  }

  bool always_on_lateral = params.getBool("AlwaysOnLateral");
  scene.show_aol_status_bar = always_on_lateral && !params.getBool("HideAOLStatusBar");

  scene.conditional_experimental = scene.longitudinal_control && params.getBool("ConditionalExperimental");
  scene.conditional_speed = scene.conditional_experimental ? params.getInt("CESpeed") : 0;
  scene.conditional_speed_lead = scene.conditional_experimental ? params.getInt("CESpeedLead") : 0;
  scene.show_cem_status_bar = scene.conditional_experimental && !params.getBool("HideCEMStatusBar");

  bool custom_onroad_ui = params.getBool("CustomUI");
  bool custom_paths = custom_onroad_ui && params.getBool("CustomPaths");
  scene.acceleration_path = custom_paths && params.getBool("AccelerationPath");
  scene.adjacent_path = custom_paths && params.getBool("AdjacentPath");
  scene.adjacent_path_metrics = scene.adjacent_path && params.getBool("AdjacentPathMetrics");
  scene.blind_spot_path = custom_paths && params.getBool("BlindSpotPath");
  scene.compass = custom_onroad_ui && params.getBool("Compass");
  scene.pedals_on_ui = custom_onroad_ui && params.getBool("PedalsOnUI");
  scene.dynamic_pedals_on_ui = scene.pedals_on_ui && params.getBool("DynamicPedalsOnUI");
  scene.static_pedals_on_ui = scene.pedals_on_ui && params.getBool("StaticPedalsOnUI");
  scene.road_name_ui = custom_onroad_ui && params.getBool("RoadNameUI");
  scene.wheel_icon = custom_onroad_ui ? params.getInt("WheelIcon") : 0;

  scene.disable_smoothing_mtsc = params.getBool("MTSCEnabled") && params.getBool("DisableMTSCSmoothing");
  scene.disable_smoothing_vtsc = params.getBool("VisionTurnControl") && params.getBool("DisableVTSCSmoothing");

  bool driving_personalities = scene.longitudinal_control && params.getBool("DrivingPersonalities");
  scene.onroad_distance_button = driving_personalities && params.getBool("OnroadDistanceButton");
  scene.use_kaofui_icons = scene.onroad_distance_button && params.getBool("KaofuiIcons");

  scene.experimental_mode_via_screen = scene.longitudinal_control && params.getBool("ExperimentalModeActivation") && params.getBool("ExperimentalModeViaTap");

  bool lane_detection = params.getBool("NudgelessLaneChange") && params.getInt("LaneDetectionWidth") != 0;
  scene.lane_detection_width = lane_detection ? params.getInt("LaneDetectionWidth") * (scene.is_metric ? 1 : FOOT_TO_METER) / 10.0f : 2.75f;

  bool longitudinal_tune = scene.longitudinal_control && params.getBool("LongitudinalTune");
  bool radarless_model = params.get("Model") == "radical-turtle";
  scene.lead_detection_threshold = longitudinal_tune && !radarless_model ? params.getInt("LeadDetectionThreshold") / 100.0f : 0.5;

  bool quality_of_life_controls = params.getBool("QOLControls");
  scene.reverse_cruise = quality_of_life_controls && params.getBool("ReverseCruise");
  scene.reverse_cruise_ui = params.getBool("ReverseCruiseUI");

  scene.speed_limit_controller = scene.longitudinal_control && params.getBool("SpeedLimitController");
  scene.show_slc_offset = scene.speed_limit_controller && params.getBool("ShowSLCOffset");
  scene.show_slc_offset_ui = scene.speed_limit_controller && params.getBool("ShowSLCOffsetUI");
  scene.use_vienna_slc_sign = scene.speed_limit_controller && params.getBool("UseVienna");

  scene.tethering_config = params.getInt("TetheringEnabled");
  if (scene.tethering_config == 2) {
    WifiManager(s).setTetheringEnabled(true);
  }
}

void UIState::updateStatus() {
  if (scene.started && sm->updated("controlsState")) {
    auto controls_state = (*sm)["controlsState"].getControlsState();
    auto state = controls_state.getState();
    if (state == cereal::ControlsState::OpenpilotState::PRE_ENABLED || state == cereal::ControlsState::OpenpilotState::OVERRIDING) {
      status = STATUS_OVERRIDE;
    } else if (scene.always_on_lateral_active) {
      status = STATUS_ALWAYS_ON_LATERAL_ACTIVE;
    } else if (scene.traffic_mode_active && scene.enabled) {
      status = STATUS_TRAFFIC_MODE_ACTIVE;
    } else {
      status = scene.enabled ? STATUS_ENGAGED : STATUS_DISENGAGED;
    }
  }

  // Handle onroad/offroad transition
  if (scene.started != started_prev || sm->frame == 1) {
    if (scene.started) {
      status = STATUS_DISENGAGED;
      scene.started_frame = sm->frame;
    }
    started_prev = scene.started;
    scene.world_objects_visible = false;
    emit offroadTransition(!scene.started);
    if (scene.tethering_config == 1) {
      wifi->setTetheringEnabled(scene.started);
    }
  }
}

UIState::UIState(QObject *parent) : QObject(parent) {
  sm = std::make_unique<SubMaster, const std::initializer_list<const char *>>({
    "modelV2", "controlsState", "liveCalibration", "radarState", "deviceState",
    "pandaStates", "carParams", "driverMonitoringState", "carState", "liveLocationKalman", "driverStateV2",
    "wideRoadCameraState", "managerState", "navInstruction", "navRoute", "uiPlan", "clocks",
    "carControl", "liveTorqueParameters",
    "frogpilotCarControl", "frogpilotCarState", "frogpilotDeviceState", "frogpilotPlan",
  });

  Params params;
  language = QString::fromStdString(params.get("LanguageSetting"));
  auto prime_value = params.get("PrimeType");
  if (!prime_value.empty()) {
    prime_type = static_cast<PrimeType>(std::atoi(prime_value.c_str()));
  }

  // update timer
  timer = new QTimer(this);
  QObject::connect(timer, &QTimer::timeout, this, &UIState::update);
  timer->start(1000 / UI_FREQ);

  // FrogPilot variables
  wifi = new WifiManager(this);

  ui_update_frogpilot_params(this);
}

void UIState::update() {
  update_sockets(this);
  update_state(this);
  updateStatus();

  if (sm->frame % UI_FREQ == 0) {
    watchdog_kick(nanos_since_boot());
  }
  emit uiUpdate(*this);

  // Update FrogPilot parameters
  static bool update_toggles = false;

  if (paramsMemory.getBool("FrogPilotTogglesUpdated")) {
    update_toggles = true;
  } else if (update_toggles) {
    ui_update_frogpilot_params(this);
    update_toggles = false;
  }

  // FrogPilot variables that need to be constantly updated
  scene.conditional_status = scene.conditional_experimental && scene.enabled ? paramsMemory.getInt("CEStatus") : 0;
}

void UIState::setPrimeType(PrimeType type) {
  if (type != prime_type) {
    bool prev_prime = hasPrime();

    prime_type = type;
    Params().put("PrimeType", std::to_string(prime_type));
    emit primeTypeChanged(prime_type);

    bool prime = hasPrime();
    if (prev_prime != prime) {
      emit primeChanged(prime);
    }
  }
}

Device::Device(QObject *parent) : brightness_filter(BACKLIGHT_OFFROAD, BACKLIGHT_TS, BACKLIGHT_DT), QObject(parent) {
  setAwake(true);
  resetInteractiveTimeout();

  QObject::connect(uiState(), &UIState::uiUpdate, this, &Device::update);
}

void Device::update(const UIState &s) {
  updateBrightness(s);
  updateWakefulness(s);
}

void Device::setAwake(bool on) {
  if (on != awake) {
    awake = on;
    Hardware::set_display_power(awake);
    LOGD("setting display power %d", awake);
    emit displayPowerChanged(awake);
  }
}

void Device::resetInteractiveTimeout(int timeout) {
  if (timeout == -1) {
    timeout = (ignition_on ? 10 : 30);
  }
  interactive_timeout = timeout * UI_FREQ;
}

void Device::updateBrightness(const UIState &s) {
  float clipped_brightness = offroad_brightness;
  if (s.scene.started && s.scene.light_sensor > 0) {
    clipped_brightness = s.scene.light_sensor;

    // CIE 1931 - https://www.photonstophotos.net/GeneralTopics/Exposure/Psychometric_Lightness_and_Gamma.htm
    if (clipped_brightness <= 8) {
      clipped_brightness = (clipped_brightness / 903.3);
    } else {
      clipped_brightness = std::pow((clipped_brightness + 16.0) / 116.0, 3.0);
    }

    // Scale back to 10% to 100%
    clipped_brightness = std::clamp(100.0f * clipped_brightness, 10.0f, 100.0f);
  }

  int brightness = brightness_filter.update(clipped_brightness);
  if (!awake) {
    brightness = 0;
  }

  if (brightness != last_brightness) {
    if (!brightness_future.isRunning()) {
      brightness_future = QtConcurrent::run(Hardware::set_brightness, brightness);
      last_brightness = brightness;
    }
  }
}

void Device::updateWakefulness(const UIState &s) {
  bool ignition_just_turned_off = !s.scene.ignition && ignition_on;
  ignition_on = s.scene.ignition;

  if (ignition_just_turned_off) {
    resetInteractiveTimeout();
  } else if (interactive_timeout > 0 && --interactive_timeout == 0) {
    emit interactiveTimeout();
  }

  setAwake(s.scene.ignition || interactive_timeout > 0);
}

UIState *uiState() {
  static UIState ui_state;
  return &ui_state;
}

Device *device() {
  static Device _device;
  return &_device;
}
