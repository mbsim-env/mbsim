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
#include "kinetics_widgets.h"
#include "function_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "octaveutils.h"
#include <QtGui>

using namespace std;

RegularizedBilateralConstraintWidget::RegularizedBilateralConstraintWidget() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  funcList = new QComboBox;
  funcList->addItem(tr("Linear regularized bilateral constraint"));
  layout->addWidget(funcList);
  setLayout(layout);
  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
  forceFunc = new LinearRegularizedBilateralConstraintWidget;  
  layout->addWidget(forceFunc);
}

void RegularizedBilateralConstraintWidget::defineFunction(int index) {
  if(index==0) {
    layout->removeWidget(forceFunc);
    delete forceFunc;
    forceFunc = new LinearRegularizedBilateralConstraintWidget;  
    layout->addWidget(forceFunc);
  }
}

RegularizedUnilateralConstraintWidget::RegularizedUnilateralConstraintWidget() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  funcList = new QComboBox;
  funcList->addItem(tr("Linear regularized unilateral constraint"));
  layout->addWidget(funcList);
  setLayout(layout);
  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
  forceFunc = new LinearRegularizedUnilateralConstraintWidget;  
  layout->addWidget(forceFunc);
}

void RegularizedUnilateralConstraintWidget::defineFunction(int index) {
  if(index==0) {
    layout->removeWidget(forceFunc);
    delete forceFunc;
    forceFunc = new LinearRegularizedUnilateralConstraintWidget;  
    layout->addWidget(forceFunc);
  }
}

UnilateralNewtonImpactWidget::UnilateralNewtonImpactWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
  restitutionCoefficient = new ExtWidget("Restitution coefficient",new ExtPhysicalVarWidget(input));
  layout->addWidget(restitutionCoefficient);
}

PlanarCoulombFrictionWidget::PlanarCoulombFrictionWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
  frictionCoefficient = new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
  layout->addWidget(frictionCoefficient);
}

SpatialCoulombFrictionWidget::SpatialCoulombFrictionWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
  frictionCoefficient = new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
  layout->addWidget(frictionCoefficient);
}

RegularizedPlanarFrictionWidget::RegularizedPlanarFrictionWidget() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  funcList = new QComboBox;
  funcList->addItem(tr("Linear regularized coulomb friction"));
  layout->addWidget(funcList);
  setLayout(layout);
  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
  frictionForceFunc = new LinearRegularizedCoulombFrictionWidget;  
  layout->addWidget(frictionForceFunc);
}

void RegularizedPlanarFrictionWidget::defineFunction(int index) {
  if(index==0) {
    layout->removeWidget(frictionForceFunc);
    delete frictionForceFunc;
    frictionForceFunc = new LinearRegularizedCoulombFrictionWidget;  
    layout->addWidget(frictionForceFunc);
  }
}

RegularizedSpatialFrictionWidget::RegularizedSpatialFrictionWidget() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  funcList = new QComboBox;
  funcList->addItem(tr("Linear regularized coulomb friction"));
  layout->addWidget(funcList);
  setLayout(layout);
  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
  frictionForceFunc = new LinearRegularizedCoulombFrictionWidget;  
  layout->addWidget(frictionForceFunc);
}

void RegularizedSpatialFrictionWidget::defineFunction(int index) {
  if(index==0) {
    layout->removeWidget(frictionForceFunc);
    delete frictionForceFunc;
    frictionForceFunc = new LinearRegularizedCoulombFrictionWidget;  
    layout->addWidget(frictionForceFunc);
  }
}

PlanarCoulombImpactWidget::PlanarCoulombImpactWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
  frictionCoefficient = new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
  layout->addWidget(frictionCoefficient);
}

SpatialCoulombImpactWidget::SpatialCoulombImpactWidget() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
  frictionCoefficient = new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
  layout->addWidget(frictionCoefficient);
}

GeneralizedForceLawChoiceWidget::GeneralizedForceLawChoiceWidget() : generalizedForceLaw(0) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Bilateral constraint"));
  comboBox->addItem(tr("Regularized bilateral constraint"));
  comboBox->addItem(tr("Unilateral constraint"));
  comboBox->addItem(tr("Regularized unilateral constraint"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void GeneralizedForceLawChoiceWidget::defineForceLaw(int index) {
  layout->removeWidget(generalizedForceLaw);
  delete generalizedForceLaw;
  if(index==0)
    generalizedForceLaw = new BilateralConstraintWidget;  
  else if(index==1)
    generalizedForceLaw = new RegularizedBilateralConstraintWidget;  
  else if(index==2)
    generalizedForceLaw = new UnilateralConstraintWidget;  
  else if(index==3)
    generalizedForceLaw = new RegularizedUnilateralConstraintWidget;  
  layout->addWidget(generalizedForceLaw);
}

GeneralizedImpactLawChoiceWidget::GeneralizedImpactLawChoiceWidget() : generalizedImpactLaw(0) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Bilateral impact"));
  comboBox->addItem(tr("Unilateral Newton impact"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineImpactLaw(int)));
  defineImpactLaw(0);
}

void GeneralizedImpactLawChoiceWidget::defineImpactLaw(int index) {
  layout->removeWidget(generalizedImpactLaw);
  delete generalizedImpactLaw;
  if(index==0)
    generalizedImpactLaw = new BilateralImpactWidget;  
  else if(index==1)
    generalizedImpactLaw = new UnilateralNewtonImpactWidget;  
  layout->addWidget(generalizedImpactLaw);
}

FrictionForceLawChoiceWidget::FrictionForceLawChoiceWidget() : frictionForceLaw(0) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Planar coulomb friction"));
  comboBox->addItem(tr("Regularized planar friction"));
  comboBox->addItem(tr("Spatial coulomb friction"));
  comboBox->addItem(tr("Regularized spatial friction"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFrictionLaw(int)));
  defineFrictionLaw(0);
}

void FrictionForceLawChoiceWidget::defineFrictionLaw(int index) {
  layout->removeWidget(frictionForceLaw);
  delete frictionForceLaw;
  if(index==0)
    frictionForceLaw = new PlanarCoulombFrictionWidget;  
  if(index==1)
    frictionForceLaw = new RegularizedPlanarFrictionWidget;  
  if(index==2)
    frictionForceLaw = new SpatialCoulombFrictionWidget;  
  if(index==3)
    frictionForceLaw = new RegularizedSpatialFrictionWidget;  
  layout->addWidget(frictionForceLaw);
}

FrictionImpactLawChoiceWidget::FrictionImpactLawChoiceWidget() : frictionImpactLaw(0) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Planar coloumb impact"));
  comboBox->addItem(tr("Spatial coloumb impact"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFrictionImpactLaw(int)));
  defineFrictionImpactLaw(0);
}

void FrictionImpactLawChoiceWidget::defineFrictionImpactLaw(int index) {
  layout->removeWidget(frictionImpactLaw);
  delete frictionImpactLaw;
  if(index==0)
    frictionImpactLaw = new PlanarCoulombImpactWidget;  
  else if(index==1)
    frictionImpactLaw = new SpatialCoulombImpactWidget;  
  layout->addWidget(frictionImpactLaw);
}

GeneralizedForceChoiceWidget::GeneralizedForceChoiceWidget() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new MatColsVarWidget(3,1,1,3),noUnitUnits(),1));
  mat = new ExtWidget("Direction vectors",new ExtPhysicalVarWidget(input));
  layout->addWidget(mat);

  generalizedForceLaw = new ExtWidget("Generalized force law",new GeneralizedForceLawChoiceWidget);
  layout->addWidget(generalizedForceLaw);

  generalizedImpactLaw = new ExtWidget("Generalized impact law",new GeneralizedImpactLawChoiceWidget,true);
  layout->addWidget(generalizedImpactLaw);
}

int GeneralizedForceChoiceWidget::getSize() const {
  string str = evalOctaveExpression(static_cast<ExtPhysicalVarWidget*>(mat->getWidget())->getCurrentPhysicalVariableWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

ForceChoiceWidget::ForceChoiceWidget() {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  MatColsVarWidget *mat_ = new MatColsVarWidget(3,1,1,3);
  input.push_back(new PhysicalVariableWidget(mat_,noUnitUnits(),1));
  mat = new ExtWidget("Direction vectors",new ExtPhysicalVarWidget(input));
  layout->addWidget(mat);

  connect(mat->getWidget(),SIGNAL(inputDialogChanged(int)),this,SLOT(resizeVariables()));
  connect(mat_, SIGNAL(sizeChanged(int)), this, SLOT(resizeVariables()));

  Function1ChoiceWidget *forceLaw_ = new Function1ChoiceWidget;
  forceLaw_->resize(1,1);
  forceLaw = new ExtWidget("Function",forceLaw_);
  layout->addWidget(forceLaw);

  connect(forceLaw_,SIGNAL(resize()),this,SLOT(resizeVariables()));
}

void ForceChoiceWidget::resizeVariables() {
  static_cast<Function1ChoiceWidget*>(forceLaw->getWidget())->resize(getSize(),1);
}

int ForceChoiceWidget::getSize() const {
  string str = evalOctaveExpression(static_cast<ExtPhysicalVarWidget*>(mat->getWidget())->getCurrentPhysicalVariableWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

ForceDirectionWidget::ForceDirectionWidget(Element *element_) : element(element_) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  forceDirWidget = new QWidget;
  QVBoxLayout *hlayout = new QVBoxLayout;
  hlayout->setMargin(0);
  forceDirWidget->setLayout(hlayout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new VecWidget(3),noUnitUnits(),1));
  mat = new ExtWidget("Direction vector",new ExtPhysicalVarWidget(input));
  hlayout->addWidget(mat);
  refFrame = new ExtWidget("Frame of reference",new FrameOfReferenceWidget(element,0));
  hlayout->addWidget(refFrame);

  layout->addWidget(forceDirWidget);

  refFrame->updateWidget();
}

GeneralizedForceDirectionWidget::GeneralizedForceDirectionWidget() {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new MatColsVarWidget(3,1,1,3),noUnitUnits(),1));
  mat = new ExtWidget("Direction vectors",new ExtPhysicalVarWidget(input));
  layout->addWidget(mat);
}

int GeneralizedForceDirectionWidget::getSize() const {
  string str = evalOctaveExpression(static_cast<ExtPhysicalVarWidget*>(mat->getWidget())->getCurrentPhysicalVariableWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}
