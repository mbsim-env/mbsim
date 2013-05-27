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

TranslationChoiceWidget::TranslationChoiceWidget() : translation(0) {
  layout = new QVBoxLayout;
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
  defineTranslation(0);
}

void TranslationChoiceWidget::defineTranslation(int index) {
  layout->removeWidget(translation);
  delete translation;
  if(index==0)
    translation = new TranslationInXDirectionWidget;  
  else if(index==1)
    translation = new TranslationInYDirectionWidget;  
  else if(index==2)
    translation = new TranslationInZDirectionWidget;  
  else if(index==3)
    translation = new TranslationInXYDirectionWidget;  
  else if(index==4)
    translation = new TranslationInXZDirectionWidget;  
  else if(index==5)
    translation = new TranslationInYZDirectionWidget;  
  else if(index==6)
    translation = new TranslationInXYZDirectionWidget;  
  else if(index==7) {
    translation = new LinearTranslationWidget;  
    connect(translation, SIGNAL(translationChanged()), this, SIGNAL(translationChanged()));
  }
  else if(index==8)
    translation = new TimeDependentTranslationWidget;  
  else if(index==9)
    translation = new StateDependentTranslationWidget;  
  else if(index==10)
    translation = new GeneralTranslationWidget;  
  layout->addWidget(translation);
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

  function = new ExtWidget("Rotational function",new Function1ChoiceWidget(false,3,"SS"));
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

  function = new ExtWidget("Rotational function",new Function1ChoiceWidget(false,3,"SV"));
  layout->addWidget(function);
}

int StateDependentRotationAboutFixedAxisWidget::getqSize() const {
  SymbolicFunction1Widget *func = dynamic_cast<SymbolicFunction1Widget*>(static_cast<Function1ChoiceWidget*>(function->getWidget())->getFunction());
  if(func)
    return func->getArgDim();
  return 0;
}

RotationChoiceWidget::RotationChoiceWidget() : rotation(0) {
  layout = new QVBoxLayout;
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
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineRotation(int)));
  defineRotation(0);
}

void RotationChoiceWidget::defineRotation(int index) {
  layout->removeWidget(rotation);
  delete rotation;
  if(index==0)
    rotation = new RotationAboutXAxisWidget;  
  else if(index==1)
    rotation = new RotationAboutYAxisWidget;  
  else if(index==2)
    rotation = new RotationAboutZAxisWidget;  
  else if(index==3)
    rotation = new RotationAboutFixedAxisWidget;  
  else if(index==4)
    rotation = new RotationAboutAxesXYWidget;  
  else if(index==5)
    rotation = new RotationAboutAxesXZWidget;  
  else if(index==6)
    rotation = new RotationAboutAxesYZWidget;  
  else if(index==7)
    rotation = new CardanAnglesWidget;  
  else if(index==8)
    rotation = new EulerAnglesWidget;  
  else if(index==9)
    rotation = new RotationAboutAxesXYZWidget;  
  else if(index==10)
    rotation = new TimeDependentRotationAboutFixedAxisWidget;  
  else if(index==11)
    rotation = new StateDependentRotationAboutFixedAxisWidget;  
  layout->addWidget(rotation);
  emit rotationChanged();
}

