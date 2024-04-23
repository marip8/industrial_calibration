#pragma once

#include <QWidget>
#include <yaml-cpp/yaml.h>

class ConfigurableWidget : public QWidget
{
  Q_OBJECT

public:
  ConfigurableWidget(QWidget *parent = nullptr);

  virtual void configure(const YAML::Node& node) = 0;
  virtual YAML::Node save() const = 0;
};
