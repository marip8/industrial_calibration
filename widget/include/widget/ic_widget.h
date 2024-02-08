#ifndef IC_WIDGET_H
#define IC_WIDGET_H

#include <QWidget>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include "industrial_calibration/target_finders/target_finder.h"

class QDialog;
class QAbstractButton;

namespace Ui {
class ICWidget;
}

class ICWidget : public QWidget
{
  Q_OBJECT

public:
  explicit ICWidget(QWidget *parent = nullptr);
  ~ICWidget();

private:
  Ui::ICWidget *ui_;

  void loadConfig();
  void saveConfig();
  void calibrate();
  void loadTargetFinder();
  cv::Mat getImageDetected(const cv::Mat& image);
  void updateProgressBar();
  void drawImage(const QPixmap& image);
  void getNextSample();
  void saveResults();
  void updateLog(const QString& message);

  QDialog* camera_transform_guess_dialog_;
  QDialog* target_transform_guess_dialog_;
  QDialog* camera_intrinsics_dialog_;
  QDialog* charuco_target_dialog_;
  QDialog* aruco_target_dialog_;
  QDialog* circle_target_dialog_;
  QString data_dir;
  
  industrial_calibration::TargetFinder::ConstPtr target_finder_;
};

#endif // IC_WIDGET_H
