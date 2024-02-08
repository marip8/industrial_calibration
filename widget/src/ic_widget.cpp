#include "widget/ic_widget.h"
#include "ui_ic_widget.h"

#include <QDialog>
#include <QScrollBar>
#include <QStandardPaths>
#include <QFileDialog>
#include <QDir>
#include <QFile>
#include <QPixmap>
#include <fstream>
#include <boost_plugin_loader/plugin_loader.hpp> 
#include "industrial_calibration/serialization.h"
#include "widget/transform_guess.h"
#include "widget/camera_intrinsics.h"
#include "widget/charuco_target.h"
#include "widget/aruco_target.h"
#include "widget/circle_target.h"
#include "widget/image_widget.h"

template<typename WidgetT>
class ICDialog : public QDialog
{
public:
  ICDialog(QWidget* parent = nullptr) : QDialog(parent)
  {
    auto* vl = new QVBoxLayout(this);
    widget = new WidgetT(this);
    vl->addWidget(widget);
  }
  WidgetT* widget;
};

template<typename WidgetT>
void setup(QWidget* const parent_widget, QDialog*& dialog, QAbstractButton* const button = nullptr)
{
  dialog = new ICDialog<WidgetT>(parent_widget);
  dialog->setWindowTitle("");
  if (button != nullptr)  // check if button is not for target detector dialog
  {
    QObject::connect(button, &QAbstractButton::clicked, dialog, &QWidget::show);
  }
};

ICWidget::ICWidget(QWidget *parent) :
  QWidget(parent),
  ui_(new Ui::ICWidget)
{
  ui_->setupUi(this);

  // Move the text edit scroll bar to the maximum limit whenever it is resized
  connect(ui_->textEditLog->verticalScrollBar(), &QScrollBar::rangeChanged, [this]() {
    ui_->textEditLog->verticalScrollBar()->setSliderPosition(ui_->textEditLog->verticalScrollBar()->maximum());
  });

  // Set up push buttons
  connect(ui_->loadConfigPushButton, &QPushButton::clicked, this, &ICWidget::loadConfig);
  connect(ui_->saveConfigPushButton, &QPushButton::clicked, this, &ICWidget::saveConfig);
  connect(ui_->nextPushButton, &QPushButton::clicked, this, &ICWidget::getNextSample);
  connect(ui_->calibratePushButton, &QPushButton::clicked, this, &ICWidget::calibrate);
  connect(ui_->saveResultsPushButton, &QPushButton::clicked, this, &ICWidget::saveResults);

  // Set up dialog boxes
  setup<TransformGuess>(this, camera_transform_guess_dialog_, ui_->cameraGuessPushButton);
  setup<TransformGuess>(this, target_transform_guess_dialog_, ui_->targetGuessToolButton);
  setup<CameraIntrinsics>(this, camera_intrinsics_dialog_, ui_->CameraIntrinsicsToolButton);
  setup<CharucoTarget>(this, charuco_target_dialog_);
  setup<ArucoTarget>(this, aruco_target_dialog_);
  setup<CircleTarget>(this, circle_target_dialog_);

  connect(ui_->targetToolButton, &QAbstractButton::clicked, [this](){
    QString type = ui_->targetComboBox->currentText();
    if(type == "CharucoGridTargetFinder")
      charuco_target_dialog_->show();
    else if(type == "ArucoGridTargetFinder")
      aruco_target_dialog_->show();
    else
      circle_target_dialog_->show();
  });

}

ICWidget::~ICWidget()
{
  delete ui_;
}

void ICWidget::updateLog(const QString& message)
{
  ui_->textEditLog->append(message);
}

void ICWidget::loadConfig()
{
  // Get yaml filepath
  const QString home = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);
  const QString file = QFileDialog::getOpenFileName(this, "Load calibration config", home, "YAML files (*.yaml *.yml)");
  if (file.isNull())
  {
    updateLog("Unable to load file, filepath is null");
    return;
  }
  ui_->configLineEdit->setText(file);
  YAML::Node node = YAML::LoadFile(file.toStdString());
  
  // Load parameters
  {
    // Camera intrinsics
    auto dialog = dynamic_cast<ICDialog<CameraIntrinsics>*>(camera_intrinsics_dialog_);
    dialog->widget->configure(node["intrinsics"]);
  }

  {
    // Camera guess
    auto dialog = dynamic_cast<ICDialog<TransformGuess>*>(camera_transform_guess_dialog_);
    dialog->widget->configure(node["camera_mount_to_camera_guess"]);
  }

  {
    // Target guess
    auto dialog = dynamic_cast<ICDialog<TransformGuess>*>(target_transform_guess_dialog_);
    dialog->widget->configure(node["target_mount_to_target_guess"]);
  }

  // Homography
  ui_->homographyDoubleSpinBox->setValue(node["homography_threshold"].as<double>());

  {
    // Target
    YAML::Node target_finder_config = node["target_finder"];
    std::string type = target_finder_config["type"].as<std::string>();
    
    // Target combo box: Charuco -> 0, Aruco -> 1, Circle -> 2
    if (type == "CharucoGridTargetFinder")
    {
      ui_->targetComboBox->setCurrentIndex(0);
      auto dialog = dynamic_cast<ICDialog<CharucoTarget>*>(charuco_target_dialog_);
      dialog->widget->configure(target_finder_config);
    }
    else if(type == "ArucoGridTargetFinder")
    {
      ui_->targetComboBox->setCurrentIndex(1);
      auto dialog = dynamic_cast<ICDialog<ArucoTarget>*>(aruco_target_dialog_);
      dialog->widget->configure(target_finder_config);
    }
    else if(type == "ModifiedCircleGridTargetFinder")
    { 
      ui_->targetComboBox->setCurrentIndex(2);
      auto dialog = dynamic_cast<ICDialog<CircleTarget>*>(circle_target_dialog_);
      dialog->widget->configure(target_finder_config);
    }
    else
    {
      updateLog("Unrecognized target finder type"); // print target finder types as hint?
      return;
    }

  }
  updateLog("Calibration config loaded from: " + file);
}

void ICWidget::saveConfig()
{
  // Get filepath
  const QString home = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);
  const QString file = QFileDialog::getSaveFileName(this, "Save calibration config", home, "YAML files (*.yaml *.yml)");
  
  if (file.isNull())
  {
    updateLog("Invalid filepath, filepath is null");
    return;
  }
  
  YAML::Node node;
  {
    // Camera intrinsics
    auto dialog = dynamic_cast<ICDialog<CameraIntrinsics>*>(camera_intrinsics_dialog_);
    node["intrinsics"] = dialog->widget->getConfig();
  }

  {
    // Camera guess
    auto dialog = dynamic_cast<ICDialog<TransformGuess>*>(camera_transform_guess_dialog_);
    node["camera_mount_to_camera_guess"] = dialog->widget->getConfig();
  }

  {
    // Target guess
    auto dialog = dynamic_cast<ICDialog<TransformGuess>*>(target_transform_guess_dialog_);
    node["target_mount_to_target_guess"] = dialog->widget->getConfig();
  }

  // Homography
  node["homography_threshold"] = ui_->homographyDoubleSpinBox->value();

  {
    // Target
    QString type = ui_->targetComboBox->currentText();
    if (type == "CharucoGridTargetFinder")
    {
      auto dialog = dynamic_cast<ICDialog<CharucoTarget>*>(charuco_target_dialog_);
      node["target_finder"] = dialog->widget->getConfig();
    }
    else if(type == "ArucoGridTargetFinder")
    {
      auto dialog = dynamic_cast<ICDialog<ArucoTarget>*>(aruco_target_dialog_);
      node["target_finder"] = dialog->widget->getConfig();
    }
    else
    { 
      auto dialog = dynamic_cast<ICDialog<CircleTarget>*>(circle_target_dialog_);
      node["target_finder"] = dialog->widget->getConfig();
    }

  }
  std::ofstream fout(file.toStdString());
  fout << node;
  fout.close();
  
  updateLog("Calibration config saved to: " + file);
}

void ICWidget::updateProgressBar()
{
  ui_->progressBar->setValue(ui_->progressBar->value()+1);
}

void ICWidget::drawImage(const QPixmap& image)
{
  ui_->imageWidget->setImage(image);
  update();  
}

void ICWidget::getNextSample()
{
  // Check if directory has been provided
  if (data_dir.isNull())
  {
    updateLog("Path to data folder is null, specify the directory using the 'Calibrate' button");
    return;
  }
  
  // Check if all images have been reviewed
  if (ui_->progressBar->value() == ui_->progressBar->maximum())
  {
    updateLog("All " + QString::number(ui_->progressBar->maximum()) + " images previewed. STAHP IT!");
    return;
  }

  const QString img_path = data_dir + "/images/" + QString::number(ui_->progressBar->value()) + ".png";
  
  // Check image exists
  QFile file(img_path);
  if (!file.exists())
  {
    updateLog("Failed to load image: '" + img_path + "' does not exist");
    return;
  } 
  
  // Detect image
  cv::Mat image = cv::imread(img_path.toStdString());
  cv::Mat image_detected = getImageDetected(image);

  
  // Convert cv::mat to QPixmap
  QPixmap image_to_draw = QPixmap::fromImage(QImage (image_detected.data, 
                                              image_detected.cols, 
                                              image_detected.rows, 
                                              image_detected.step, 
                                              QImage::Format_RGB888).rgbSwapped());
  
  // Draw image detected
  drawImage(image_to_draw);

  // Calc homography

  // Update progress
  updateProgressBar();
  
}

void ICWidget::calibrate()
{
  // Provide directory or provide YAML with absolute/relative paths to data or both YAML and directory?
  const QString home = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);
  const QString dir = QFileDialog::getExistingDirectory(this, "Select data directory", home);

  // Check directory exists
  if (dir.isNull())
  {
    updateLog("Invalid directory, path is null");
    return;
  }
  data_dir = dir;

  updateLog("Opened directory: " + data_dir);

  // Set up progress bar
  QDir directory(data_dir + "/images");
  QStringList files = directory.entryList(QDir::Files);
  int num_imgs = files.size();
  ui_->progressBar->setMaximum(num_imgs);
  ui_->progressBar->setMinimum(0);
  ui_->progressBar->setValue(0);
  
  // Load target finder
  loadTargetFinder();

  // Show first image
  getNextSample();
  
}

void ICWidget::loadTargetFinder()
{
  boost_plugin_loader::PluginLoader loader;
  loader.search_libraries.insert(INDUSTRIAL_CALIBRATION_PLUGIN_LIBRARIES);
  loader.search_libraries_env = INDUSTRIAL_CALIBRATION_SEARCH_LIBRARIES_ENV;

  // Get target type and currentconfig
  QString type = ui_->targetComboBox->currentText();
  YAML::Node target_finder_config;

  if (type == "CharucoGridTargetFinder")
  {
    auto dialog = dynamic_cast<ICDialog<CharucoTarget>*>(charuco_target_dialog_);
    target_finder_config = dialog->widget->getConfig();
  }
  else if(type == "ArucoGridTargetFinder")
  {
    auto dialog = dynamic_cast<ICDialog<ArucoTarget>*>(aruco_target_dialog_);
    target_finder_config = dialog->widget->getConfig();
  }
  else
  { 
    auto dialog = dynamic_cast<ICDialog<CircleTarget>*>(circle_target_dialog_);
    target_finder_config = dialog->widget->getConfig();
  }

  auto factory = loader.createInstance<industrial_calibration::TargetFinderFactory>(getMember<std::string>(target_finder_config, "type"));
  target_finder_ = factory->create(target_finder_config);
}

cv::Mat ICWidget::getImageDetected(const cv::Mat& image)
{
  industrial_calibration::TargetFeatures target_features = target_finder_->findTargetFeatures(image);
  cv::Mat image_detected = target_finder_->drawTargetFeatures(image, target_features);
  
  return image_detected;
}

void ICWidget::saveResults()
{
  const QString home = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);
  const QString file = QFileDialog::getSaveFileName(this, "Save results", home, "YAML files (*.yaml *.yml)");
  updateLog("Save results coming soon to a robot near you!");
}
