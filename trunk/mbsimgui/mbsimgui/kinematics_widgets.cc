/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <config.h>
#include "kinematics_widgets.h"
#include "variable_widgets.h"
#include "function_widgets.h"
#include "extended_widgets.h"
#include "octaveutils.h"
#include <QtGui>

using namespace std;

LinearTranslationWidget::LinearTranslationWidget() {
  vector<PhysicalVariableWidget*> input;
  MatColsVarWidget* m = new MatColsVarWidget(3,1,1,3);
  input.push_back(new PhysicalVariableWidget(m,noUnitUnits(),1));
  ExtPhysicalVarWidget *mat_ = new ExtPhysicalVarWidget(input);
  mat = new ExtWidget("Translation vectors",mat_);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(mat);
  QObject::connect(m, SIGNAL(sizeChanged(int)), this, SIGNAL(translationChanged()));
  QObject::connect(mat_, SIGNAL(inputDialogChanged(int)), this, SIGNAL(translationChanged()));
}

int LinearTranslationWidget::getqTSize() const {
  string str = evalOctaveExpression(static_cast<ExtPhysicalVarWidget*>(mat->getWidget())->getCurrentPhysicalVariableWidget()->getValue().toStdString());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

TimeDependentTranslationWidget::TimeDependentTranslationWidget() {
  function = new ExtWidget("Translation function",new Function1ChoiceWidget(false,3));

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(function);
}

StateDependentTranslationWidget::StateDependentTranslationWidget() {
  function = new ExtWidget("Translation function",new Function1ChoiceWidget(false,3,"VV"));

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(function);
}

int StateDependentTranslationWidget::getqSize() const {
  SymbolicFunction1Widget *func = dynamic_cast<SymbolicFunction1Widget*>(static_cast<Function1ChoiceWidget*>(function->getWidget())->getFunction());
  if(func)
    return func->getArgDim();
  return 0;
}

GeneralTranslationWidget::GeneralTranslationWidget() {
  function = new ExtWidget("Translation function",new Function2ChoiceWidget("VVS"));

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(function);
}

TranslationChoiceWidget::TranslationChoiceWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Translation in x direction"));
  comboBox->addItem(tr("Translation in y direction"));
  comboBox->addItem(tr("Translation in z direction"));
  comboBox->addItem(tr("Translation in x- and y- direction"));
  comboBox->addItem(tr("Translation in x- and z- direction"));
  comboBox->addItem(tr("Translation in y- and z- direction"));
  comboBox->addItem(tr("Translation in x-, y- and z- direction"));
  comboBox->addItem(tr("Linear translation"));
  comboBox->addItem(tr("Time dependent translation"));
  comboBox->addItem(tr("State dependent translation"));
  comboBox->addItem(tr("General translation"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineTranslation(int)));
  stackedWidget = new QStackedWidget;
  TranslationWidget *translation;
  translation = new TranslationInXDirectionWidget;  
  stackedWidget->addWidget(translation);
  translation = new TranslationInYDirectionWidget;  
  translation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(translation);
  translation = new TranslationInZDirectionWidget;  
  translation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(translation);
  translation = new TranslationInXYDirectionWidget;  
  translation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(translation);
  translation = new TranslationInXZDirectionWidget;  
  translation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(translation);
  translation = new TranslationInYZDirectionWidget;  
  translation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(translation);
  translation = new TranslationInXYZDirectionWidget;  
  translation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(translation);
  translation = new LinearTranslationWidget;  
  connect(translation, SIGNAL(translationChanged()), this, SIGNAL(translationChanged()));
  translation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(translation);
  translation = new TimeDependentTranslationWidget;  
  translation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(translation);
  translation = new StateDependentTranslationWidget;  
  translation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(translation);
  translation = new GeneralTranslationWidget;  
  translation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(translation);
  layout->addWidget(stackedWidget);
  connect(comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(defineTranslation(int)));
}

TranslationWidget* TranslationChoiceWidget::getTranslation() const {
  return static_cast<TranslationWidget*>(stackedWidget->currentWidget());
}

TranslationWidget* TranslationChoiceWidget::getTranslation(int i) {
  return static_cast<TranslationWidget*>(stackedWidget->widget(i));
}

void TranslationChoiceWidget::defineTranslation(int index) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(index);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
  emit translationChanged();
}

RotationAboutFixedAxisWidget::RotationAboutFixedAxisWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new VecWidget(3),noUnitUnits(),1));
  ExtPhysicalVarWidget *vec_ = new ExtPhysicalVarWidget(input);
  vec = new ExtWidget("Axis of rotation",vec_);
  layout->addWidget(vec);
}

TimeDependentRotationAboutFixedAxisWidget::TimeDependentRotationAboutFixedAxisWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new VecWidget(3),noUnitUnits(),1));
  ExtPhysicalVarWidget *vec_ = new ExtPhysicalVarWidget(input);
  vec = new ExtWidget("Axis of rotation",vec_);
  layout->addWidget(vec);

  function = new ExtWidget("Rotational function",new Function1ChoiceWidget(false,1,"SS"));
  layout->addWidget(function);
}

StateDependentRotationAboutFixedAxisWidget::StateDependentRotationAboutFixedAxisWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new VecWidget(3),noUnitUnits(),1));
  ExtPhysicalVarWidget *vec_ = new ExtPhysicalVarWidget(input);
  vec = new ExtWidget("Axis of rotation",vec_);
  layout->addWidget(vec);

  function = new ExtWidget("Rotational function",new Function1ChoiceWidget(false,1,"SV"));
  layout->addWidget(function);
}

int StateDependentRotationAboutFixedAxisWidget::getqSize() const {
  SymbolicFunction1Widget *func = dynamic_cast<SymbolicFunction1Widget*>(static_cast<Function1ChoiceWidget*>(function->getWidget())->getFunction());
  if(func)
    return func->getArgDim();
  return 0;
}

RotationChoiceWidget::RotationChoiceWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Rotation about x-axis"));
  comboBox->addItem(tr("Rotation about y-axis"));
  comboBox->addItem(tr("Rotation about z-axis"));
  comboBox->addItem(tr("Rotation about fixed axis"));
  comboBox->addItem(tr("Rotation about x- and y-axis"));
  comboBox->addItem(tr("Rotation about x- and z-axis"));
  comboBox->addItem(tr("Rotation about y- and z-axis"));
  comboBox->addItem(tr("Cardan angles"));
  comboBox->addItem(tr("Euler angles"));
  comboBox->addItem(tr("Rotation about x-, y- and z-axis"));
  comboBox->addItem(tr("Time dependent rotation about fixed axis"));
  comboBox->addItem(tr("State dependent rotation about fixed axis"));
  layout->addWidget(comboBox);
  stackedWidget = new QStackedWidget;
  RotationWidget *rotation;
  rotation = new RotationAboutXAxisWidget;  
  stackedWidget->addWidget(rotation);
  rotation = new RotationAboutYAxisWidget;  
  rotation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(rotation);
  rotation = new RotationAboutZAxisWidget;  
  rotation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(rotation);
  rotation = new RotationAboutFixedAxisWidget;  
  rotation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(rotation);
  rotation = new RotationAboutAxesXYWidget;  
  rotation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(rotation);
  rotation = new RotationAboutAxesXZWidget;  
  rotation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(rotation);
  rotation = new RotationAboutAxesYZWidget;  
  rotation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(rotation);
  rotation = new CardanAnglesWidget;  
  rotation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(rotation);
  rotation = new EulerAnglesWidget;  
  rotation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(rotation);
  rotation = new RotationAboutAxesXYZWidget;  
  rotation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(rotation);
  rotation = new TimeDependentRotationAboutFixedAxisWidget;  
  rotation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(rotation);
  rotation = new StateDependentRotationAboutFixedAxisWidget;  
  rotation->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(rotation);
  layout->addWidget(stackedWidget);
  connect(comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(defineRotation(int)));
}

RotationWidget* RotationChoiceWidget::getRotation() const {
  return static_cast<RotationWidget*>(stackedWidget->currentWidget());
}

RotationWidget* RotationChoiceWidget::getRotation(int i) {
  return static_cast<RotationWidget*>(stackedWidget->widget(i));
}

void RotationChoiceWidget::defineRotation(int index) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(index);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
  emit rotationChanged();
}

