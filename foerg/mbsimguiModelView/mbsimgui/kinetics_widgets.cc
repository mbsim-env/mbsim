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
#include "string_widgets.h"
#include "extended_widgets.h"
#include "octaveutils.h"
#include <QtGui>

using namespace std;

RegularizedBilateralConstraint::RegularizedBilateralConstraint() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  funcList = new QComboBox;
  funcList->addItem(tr("Linear regularized bilateral constraint"));
  layout->addWidget(funcList);
  setLayout(layout);
  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
  forceFunc = new LinearRegularizedBilateralConstraint;  
  layout->addWidget(forceFunc);
}

void RegularizedBilateralConstraint::defineFunction(int index) {
  if(index==0) {
    layout->removeWidget(forceFunc);
    delete forceFunc;
    forceFunc = new LinearRegularizedBilateralConstraint;  
    layout->addWidget(forceFunc);
  }
}

RegularizedUnilateralConstraint::RegularizedUnilateralConstraint() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  funcList = new QComboBox;
  funcList->addItem(tr("Linear regularized unilateral constraint"));
  layout->addWidget(funcList);
  setLayout(layout);
  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
  forceFunc = new LinearRegularizedUnilateralConstraint;  
  layout->addWidget(forceFunc);
}

void RegularizedUnilateralConstraint::defineFunction(int index) {
  if(index==0) {
    layout->removeWidget(forceFunc);
    delete forceFunc;
    forceFunc = new LinearRegularizedUnilateralConstraint;  
    layout->addWidget(forceFunc);
  }
}

UnilateralNewtonImpact::UnilateralNewtonImpact() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),noUnitUnits(),1));
  restitutionCoefficient = new ExtWidget("Restitution coefficient",new ExtPhysicalVarWidget(input));
  layout->addWidget(restitutionCoefficient);
}

PlanarCoulombFriction::PlanarCoulombFriction() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),noUnitUnits(),1));
  frictionCoefficient = new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
  layout->addWidget(frictionCoefficient);
}

SpatialCoulombFriction::SpatialCoulombFriction() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),noUnitUnits(),1));
  frictionCoefficient = new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
  layout->addWidget(frictionCoefficient);
}

RegularizedPlanarFriction::RegularizedPlanarFriction() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  funcList = new QComboBox;
  funcList->addItem(tr("Linear regularized coulomb friction"));
  layout->addWidget(funcList);
  setLayout(layout);
  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
  frictionForceFunc = new LinearRegularizedCoulombFriction;  
  layout->addWidget(frictionForceFunc);
}

void RegularizedPlanarFriction::defineFunction(int index) {
  if(index==0) {
    layout->removeWidget(frictionForceFunc);
    delete frictionForceFunc;
    frictionForceFunc = new LinearRegularizedCoulombFriction;  
    layout->addWidget(frictionForceFunc);
  }
}

RegularizedSpatialFriction::RegularizedSpatialFriction() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  funcList = new QComboBox;
  funcList->addItem(tr("Linear regularized coulomb friction"));
  layout->addWidget(funcList);
  setLayout(layout);
  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
  frictionForceFunc = new LinearRegularizedCoulombFriction;  
  layout->addWidget(frictionForceFunc);
}

void RegularizedSpatialFriction::defineFunction(int index) {
  if(index==0) {
    layout->removeWidget(frictionForceFunc);
    delete frictionForceFunc;
    frictionForceFunc = new LinearRegularizedCoulombFriction;  
    layout->addWidget(frictionForceFunc);
  }
}

PlanarCoulombImpact::PlanarCoulombImpact() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),noUnitUnits(),1));
  frictionCoefficient = new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
  layout->addWidget(frictionCoefficient);
}

SpatialCoulombImpact::SpatialCoulombImpact() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),noUnitUnits(),1));
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
    generalizedForceLaw = new BilateralConstraint;  
  else if(index==1)
    generalizedForceLaw = new RegularizedBilateralConstraint;  
  else if(index==2)
    generalizedForceLaw = new UnilateralConstraint;  
  else if(index==3)
    generalizedForceLaw = new RegularizedUnilateralConstraint;  
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
    generalizedImpactLaw = new BilateralImpact;  
  else if(index==1)
    generalizedImpactLaw = new UnilateralNewtonImpact;  
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
    frictionForceLaw = new PlanarCoulombFriction;  
  if(index==1)
    frictionForceLaw = new RegularizedPlanarFriction;  
  if(index==2)
    frictionForceLaw = new SpatialCoulombFriction;  
  if(index==3)
    frictionForceLaw = new RegularizedSpatialFriction;  
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
    frictionImpactLaw = new PlanarCoulombImpact;  
  else if(index==1)
    frictionImpactLaw = new SpatialCoulombImpact;  
  layout->addWidget(frictionImpactLaw);
}

GeneralizedForceChoiceWidget::GeneralizedForceChoiceWidget(ExtWidget* arrow_) : arrow(arrow_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new MatColsVarWidget(3,1,1,3),noUnitUnits(),1));
  mat_ = new ExtPhysicalVarWidget(input);  
  mat = new ExtWidget("Direction vectors",mat_);
  layout->addWidget(mat);

  generalizedForceLaw_ = new GeneralizedForceLawChoiceWidget;
  generalizedForceLaw = new ExtWidget("Generalized force law",generalizedForceLaw_);
  layout->addWidget(generalizedForceLaw);

  generalizedImpactLaw_ = new GeneralizedImpactLawChoiceWidget;
  generalizedImpactLaw = new ExtWidget("Generalized impact law",generalizedImpactLaw_,true);
  layout->addWidget(generalizedImpactLaw);
}

int GeneralizedForceChoiceWidget::getSize() const {
  string str = evalOctaveExpression(mat_->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

ForceChoiceWidget::ForceChoiceWidget(ExtWidget* arrow_) : arrow(arrow_) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  PhysicalStringWidget *mat = new PhysicalStringWidget(new MatColsVarWidget(3,1,1,3),noUnitUnits(),1);
  input.push_back(mat);
  widget = new ExtPhysicalVarWidget(input);  
  ExtWidget *extWidget = new ExtWidget("Direction vectors",widget);

  connect(widget,SIGNAL(inputDialogChanged(int)),this,SLOT(resizeVariables()));
  connect((MatColsVarWidget*)mat->getWidget(), SIGNAL(sizeChanged(int)), this, SLOT(resizeVariables()));
  layout->addWidget(extWidget);

  forceLaw = new Function1ChoiceWidget;
  forceLaw->resize(1,1);
  extWidget = new ExtWidget("Function",forceLaw);

  layout->addWidget(extWidget);
  connect(forceLaw,SIGNAL(resize()),this,SLOT(resizeVariables()));
}

void ForceChoiceWidget::resizeVariables() {
  forceLaw->resize(getSize(),1);
  //((Function1ChoiceWidget*)forceLaw->getWidget())->resize(getSize(),1);
}

int ForceChoiceWidget::getSize() const {
  string str = evalOctaveExpression(widget->getCurrentPhysicalStringWidget()->getValue());
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

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(3),noUnitUnits(),1));
  mat = new ExtPhysicalVarWidget(input);
  ExtWidget *extWidget = new ExtWidget("Direction vector",mat);
  hlayout->addWidget(extWidget);
  refFrame = new FrameOfReferenceWidget(element,0);
  extWidget = new ExtWidget("Frame of reference",refFrame);
  hlayout->addWidget(extWidget);

  layout->addWidget(forceDirWidget);

  refFrame->updateWidget();
}

GeneralizedForceDirectionWidget::GeneralizedForceDirectionWidget() {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new MatColsVarWidget(3,0,0,3),noUnitUnits(),1));
  mat = new ExtPhysicalVarWidget(input);  
  ExtWidget *extWidget = new ExtWidget("Direction vectors",mat);
  layout->addWidget(extWidget);
}

int GeneralizedForceDirectionWidget::getSize() const {
  string str = evalOctaveExpression(mat->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}
