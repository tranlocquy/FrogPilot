#pragma once

#include <cmath>
#include <set>

#include <QDomDocument>
#include <QMovie>
#include <QNetworkReply>
#include <QStyle>
#include <QTimer>

#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/controls.h"

inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }

QFont fitInterFont(int pixelSize, QFont::Weight weight, int width, const QStringList &texts);

QString cleanModelName(QString modelName);

void loadImage(const QString &basePath, QPixmap &pixmap, QSharedPointer<FrogPilotMovie> &movie, const QSize &size, QWidget *parent, bool repaintOnFrame = true);
void loadMovie(const QString &moviePath, QSharedPointer<FrogPilotMovie> &movie, const QSize &size, QWidget *parent, bool repaintOnFrame = true);

template <typename Function>
void runOnUIThread(QObject *context, Function &&function) {
  QMetaObject::invokeMethod(context, std::forward<Function>(function), Qt::QueuedConnection);
}
