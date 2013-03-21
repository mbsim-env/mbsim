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
#include "string_widgets.h"
#include "extended_widgets.h"
#include "octaveutils.h"
#include <QtGui>

using namespace std;

LinearTranslationWidget::LinearTranslationWidget() {
  vector<PhysicalStringWidget*> input;
  MatColsVarWidget* m = new MatColsVarWidget(3,1,1,3);
  input.push_back(new PhysicalStringWidget(m,noUnitUnits(),1));
  ExtPhysicalVarWidget *mat_ = new ExtPhysicalVarWidget(input);
  mat = new ExtWidget("Translation vectors",mat_);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(mat);
  QObject::connect(m, SIGNAL(sizeChanged(int)), this, SIGNAL(translationChanged()));
  QObject::connect(mat_, SIGNAL(inputDialogChanged(int)), this, SIGNAL(translationChanged()));
}

int LinearTranslationWidget::getSize() const {
  string str = evalOctaveExpression(static_cast<ExtPhysicalVarWidget*>(mat->getWidget())->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

TranslationChoiceWidget::TranslationChoiceWidget(const string &xmlName_) : translation(0), xmlName(xmlName_) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("LinearTranslation"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineTranslation(int)));
  defineTranslation(0);
}

void TranslationChoiceWidget::defineTranslation(int index) {
//  if(index==0) {
//    layout->removeWidget(translation);
//    delete translation;
//    translation = 0;
//  } 
  if(index==0) {
    translation = new LinearTranslationWidget;  
    connect((LinearTranslationWidget*)translation, SIGNAL(translationChanged()), this, SIGNAL(translationChanged()));
  layout->addWidget(translation);
  }
  emit translationChanged();
}

RotationAboutFixedAxisWidget::RotationAboutFixedAxisWidget() {
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(3),noUnitUnits(),1));
  ExtPhysicalVarWidget *vec_ = new ExtPhysicalVarWidget(input);
  vec = new ExtWidget("Translation vectors",vec_);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(vec);
}

RotationChoiceWidget::RotationChoiceWidget(const string &xmlName_) : rotation(0), xmlName(xmlName_) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Rotation about x-axis"));
  comboBox->addItem(tr("Rotation about y-axis"));
  comboBox->addItem(tr("Rotation about z-axis"));
  comboBox->addItem(tr("Rotation about fixed axis"));
  comboBox->addItem(tr("Cardan angles"));
  comboBox->addItem(tr("Rotation about x- and y-axis"));
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
    rotation = new CardanAnglesWidget;  
  else if(index==5)
    rotation = new RotationAboutAxesXYWidget;  
  layout->addWidget(rotation);
  emit rotationChanged();
}

