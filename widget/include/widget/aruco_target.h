#ifndef ARUCO_TARGET_H
#define ARUCO_TARGET_H

#include "configurable_widget.h"

namespace Ui {
class ArucoTarget;
}

class ArucoTarget : public ConfigurableWidget
{
public:
  explicit ArucoTarget(QWidget *parent = nullptr);
  ~ArucoTarget();

  void configure(const YAML::Node& node) override;
  YAML::Node getConfig() const override;

private:
  Ui::ArucoTarget *ui_;
};

#endif // ARUCO_TARGET_H
