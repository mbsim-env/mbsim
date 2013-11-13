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
#include "kinematic_functions_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "mainwindow.h"
#include <mbxmlutils/octeval.h>
#include <QtGui>

using namespace std;

TranslationAlongFixedAxisWidget::TranslationAlongFixedAxisWidget() {

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new VecWidget(3),QStringList(),0));
  a = new ExtWidget("Axis of translation",new ExtPhysicalVarWidget(input));
  varlayout->addWidget(a);
}

LinearTranslationWidget::LinearTranslationWidget(int m, int n) {
//  MatColsVarWidget* m = new MatColsVarWidget(3,1,1,3);
//  input.push_back(new PhysicalVariableWidget(m,noUnitUnits(),1));
//  ExtPhysicalVarWidget *mat_ = new ExtPhysicalVarWidget(input);
//  mat = new ExtWidget("Translation vectors",mat_);
//  layout->addWidget(mat);
//  QObject::connect(m, SIGNAL(sizeChanged(int)), this, SIGNAL(translationChanged()));
//  QObject::connect(mat_, SIGNAL(inputDialogChanged(int)), this, SIGNAL(translationChanged()));

  vector<PhysicalVariableWidget*> input;
  MatColsVarWidget *a_ = new MatColsVarWidget(m,1,1,3);
  input.push_back(new PhysicalVariableWidget(a_,QStringList(),0));
  connect(a_,SIGNAL(sizeChanged(int)),this,SIGNAL(arg1SizeChanged(int)));
  A = new ExtWidget("Slope",new ExtPhysicalVarWidget(input));
  varlayout->addWidget(A);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new VecWidget(m),QStringList(),0));
  b = new ExtWidget("Intercept",new ExtPhysicalVarWidget(input),true);
  varlayout->addWidget(b);
}

int LinearTranslationWidget::getArg1Size() const {
  string str = MBXMLUtils::OctEval::cast<string>(MainWindow::octEval->stringToOctValue(static_cast<ExtPhysicalVarWidget*>(A->getWidget())->getCurrentPhysicalVariableWidget()->getValue().toStdString()));
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

void LinearTranslationWidget::resize_(int m, int n) {
//  if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(c->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m)
//    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(c->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
}

RotationAboutFixedAxisWidget::RotationAboutFixedAxisWidget() {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new VecWidget(3),QStringList(),0));
  a = new ExtWidget("Axis of rotation",new ExtPhysicalVarWidget(input));
  layout->addWidget(a);
}

