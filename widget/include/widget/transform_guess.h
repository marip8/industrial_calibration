#ifndef TRANSFORM_GUESS_H
#define TRANSFORM_GUESS_H

#include <QWidget>
#include <yaml-cpp/yaml.h>

namespace Ui {
class TransformGuess;
}

class TransformGuess : public QWidget
{
  Q_OBJECT

public:
  explicit TransformGuess(QWidget *parent = nullptr);
  ~TransformGuess();

  void configure(const YAML::Node& node);
  YAML::Node getConfig();

private:
  Ui::TransformGuess *ui_;
};

#endif // TRANSFORM_GUESS_H
