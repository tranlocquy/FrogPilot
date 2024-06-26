#include "selfdrive/ui/qt/onroad/buttons.h"

#include <QPainter>

#include "selfdrive/ui/qt/util.h"

void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity, const int angle) {
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.save();
  p.translate(center);
  p.rotate(angle);
  p.setOpacity(opacity);
  p.drawPixmap(-QPoint(img.width() / 2, img.height() / 2), img);
  p.setOpacity(1.0);
  p.restore();
}

// ExperimentalButton
ExperimentalButton::ExperimentalButton(QWidget *parent) : experimental_mode(false), engageable(false), QPushButton(parent) {
  setFixedSize(btn_size, btn_size);

  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size, img_size});
  QObject::connect(this, &QPushButton::clicked, this, &ExperimentalButton::changeMode);

  wheelImages = {
    {0, loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size})},
    {1, loadPixmap("../frogpilot/assets/wheel_images/lexus.png", {img_size, img_size})},
    {2, loadPixmap("../frogpilot/assets/wheel_images/toyota.png", {img_size, img_size})},
    {3, loadPixmap("../frogpilot/assets/wheel_images/frog.png", {img_size, img_size})},
    {4, loadPixmap("../frogpilot/assets/wheel_images/rocket.png", {img_size, img_size})},
    {5, loadPixmap("../frogpilot/assets/wheel_images/hyundai.png", {img_size, img_size})},
    {6, loadPixmap("../frogpilot/assets/wheel_images/stalin.png", {img_size, img_size})},
  };
}

void ExperimentalButton::changeMode() {
  const auto cp = (*uiState()->sm)["carParams"].getCarParams();
  bool can_change = hasLongitudinalControl(cp) && params.getBool("ExperimentalModeConfirmed");
  if (can_change) {
    if (conditionalExperimental) {
      int override_value = (conditionalStatus >= 1 && conditionalStatus <= 6) ? 0 : conditionalStatus >= 7 ? 5 : 6;
      paramsMemory.putIntNonBlocking("CEStatus", override_value);
    } else {
      params.putBool("ExperimentalMode", !experimental_mode);
    }
  }
}

void ExperimentalButton::updateState(const UIState &s) {
  const auto cs = (*s.sm)["controlsState"].getControlsState();
  bool eng = cs.getEngageable() || cs.getEnabled() || s.scene.always_on_lateral_active;
  if ((cs.getExperimentalMode() != experimental_mode) || (eng != engageable)) {
    engageable = eng;
    experimental_mode = cs.getExperimentalMode();
    update();
  }

  // FrogPilot variables
  const UIScene &scene = s.scene;

  alwaysOnLateralActive = scene.always_on_lateral_active;
  conditionalExperimental = scene.conditional_experimental;
  conditionalStatus = scene.conditional_status;
  navigateOnOpenpilot = scene.navigate_on_openpilot;
  rotatingWheel = scene.rotating_wheel;
  trafficModeActive = scene.traffic_mode_active;
  wheelIcon = scene.wheel_icon;

  if (rotatingWheel && steeringAngleDeg != scene.steering_angle_deg) {
    update();
    steeringAngleDeg = scene.steering_angle_deg;
  } else if (!rotatingWheel) {
    steeringAngleDeg = 0;
  }
}

void ExperimentalButton::paintEvent(QPaintEvent *event) {
  if (wheelIcon < 0) {
    return;
  }

  QPainter p(this);
  engage_img = wheelImages[wheelIcon];
  QPixmap img = wheelIcon != 0 ? engage_img : (experimental_mode ? experimental_img : engage_img);

  QColor background_color = wheelIcon != 0 && !isDown() && engageable ?
    (alwaysOnLateralActive ? bg_colors[STATUS_ALWAYS_ON_LATERAL_ACTIVE] :
    (conditionalStatus == 1 || conditionalStatus == 3 || conditionalStatus == 5 ? bg_colors[STATUS_CONDITIONAL_OVERRIDDEN] :
    (experimental_mode ? bg_colors[STATUS_EXPERIMENTAL_MODE_ACTIVE] :
    (trafficModeActive ? bg_colors[STATUS_TRAFFIC_MODE_ACTIVE] :
    (navigateOnOpenpilot ? bg_colors[STATUS_NAVIGATION_ACTIVE] : QColor(0, 0, 0, 166)))))) :
    QColor(0, 0, 0, 166);

  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), img, QColor(0, 0, 0, 166), (isDown() || !engageable) ? 0.6 : 1.0, steeringAngleDeg);
}

// MapSettingsButton
MapSettingsButton::MapSettingsButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size, btn_size + 20);
  settings_img = loadPixmap("../assets/navigation/icon_directions_outlined.svg", {img_size, img_size});

  // hidden by default, made visible if map is created (has prime or mapbox token)
  setVisible(false);
  setEnabled(false);
}

void MapSettingsButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), settings_img, QColor(0, 0, 0, 166), isDown() ? 0.6 : 1.0);
}

// FrogPilot buttons

// DistanceButton
DistanceButton::DistanceButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size * 1.5, btn_size * 1.5);
  transitionTimer.start();

  connect(this, &QPushButton::pressed, this, &DistanceButton::buttonPressed);
  connect(this, &QPushButton::released, this, &DistanceButton::buttonReleased);
}

void DistanceButton::buttonPressed() {
  paramsMemory.putBool("OnroadDistanceButtonPressed", true);
}

void DistanceButton::buttonReleased() {
  paramsMemory.putBool("OnroadDistanceButtonPressed", false);
}

void DistanceButton::updateState(const UIScene &scene) {
  if ((trafficModeActive == scene.traffic_mode_active) &&
      (personality == static_cast<int>(scene.personality) || trafficModeActive)) {
    return;
  }

  personality = static_cast<int>(scene.personality);
  trafficModeActive = scene.traffic_mode_active;

  int profile = trafficModeActive ? 0 : personality + 1;
  std::tie(profileImage, profileText) = (scene.use_kaofui_icons ? profileDataKaofui : profileData)[profile];

  transitionTimer.restart();

  update();
}

void DistanceButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);

  elapsed = transitionTimer.elapsed();
  qreal textOpacity = qBound(0.0, 1.0 - ((elapsed - 3000.0) / 1000.0), 1.0);
  qreal imageOpacity = 1.0 - textOpacity;

  if (textOpacity > 0.0) {
    p.setOpacity(textOpacity);
    p.setFont(InterFont(40, QFont::Bold));
    p.setPen(Qt::white);
    QRect textRect(-25, 0, width(), height() + btn_size / 2);
    p.drawText(textRect, Qt::AlignCenter, profileText);
  }

  if (imageOpacity > 0.0) {
    drawIcon(p, QPoint((btn_size / 2) * 1.25, btn_size), profileImage, Qt::transparent, imageOpacity);
  }
}
