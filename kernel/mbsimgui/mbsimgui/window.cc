#include "window.h"
#include <QLabel>
#include <QDoubleSpinBox>
#include <QGridLayout>

Window::Window(QWidget *parent) : QWidget(parent) {

  QGridLayout *layout = new QGridLayout;
  QLabel *label = new QLabel;
  label->setText("x:");
  layout->addWidget(label, 0, 0);
  label = new QLabel;
  label->setText("y:");
  layout->addWidget(label, 1, 0);
  label = new QLabel;
  label->setText("z:");
  layout->addWidget(label, 2, 0);

  for(int i=0; i<3; i++) {
    box[i] = new QDoubleSpinBox(this);
    layout->addWidget(box[i], i, 1);
  }
  setLayout(layout);

}
