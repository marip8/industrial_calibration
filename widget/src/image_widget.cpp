#include "widget/image_widget.h"
#include <QPainter>

ImageWidget::ImageWidget(QWidget* parent) : QWidget(parent)
{
  image_original_ = QPixmap(size());
  image_original_.fill(Qt::white);
  image_scaled_ = image_original_;
}
  
void ImageWidget::setImage(const QPixmap& image)
{
  image_original_ = image;
  image_scaled_ = image_original_;
}
  
void ImageWidget::paintEvent(QPaintEvent *event)
{
  QWidget::paintEvent(event);
  QPainter painter(this);
  painter.drawPixmap(rect(), image_scaled_);
}

void ImageWidget::resizeEvent(QResizeEvent *event)
{
  image_scaled_ = image_original_.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
  update();  // Trigger a repaint
}