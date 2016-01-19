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
#include "custom_widgets.h"
#include "function_widget_factory.h"
#include <QtGui>

using namespace std;

namespace MBSimGUI {

  RegularizedBilateralConstraintWidget::RegularizedBilateralConstraintWidget() {

    layout = new QVBoxLayout;
    layout->setMargin(0);
    funcList = new CustomComboBox;
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
    funcList = new CustomComboBox;
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

  PlanarStribeckFrictionWidget::PlanarStribeckFrictionWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget2(new FunctionWidgetFactory2(NULL)),false);
    layout->addWidget(frictionFunction);
  }

  SpatialStribeckFrictionWidget::SpatialStribeckFrictionWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget2(new FunctionWidgetFactory2(NULL)),false);
    layout->addWidget(frictionFunction);
  }

  RegularizedPlanarFrictionWidget::RegularizedPlanarFrictionWidget() {

    layout = new QVBoxLayout;
    layout->setMargin(0);
    funcList = new CustomComboBox;
    funcList->addItem(tr("Linear regularized Coulomb friction"));
    funcList->addItem(tr("Linear regularized Stribeck friction"));
    funcList->addItem(tr("Symbolic friction function"));
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
    else if(index==1) {
      layout->removeWidget(frictionForceFunc);
      delete frictionForceFunc;
      frictionForceFunc = new LinearRegularizedStribeckFrictionWidget;
      layout->addWidget(frictionForceFunc);
    }
    else {
      layout->removeWidget(frictionForceFunc);
      delete frictionForceFunc;
       QStringList var;
      var << "gd" << "laN";
      frictionForceFunc = new SymbolicFunctionWidget(var,1,1);
      layout->addWidget(frictionForceFunc);
    }
  }

  RegularizedSpatialFrictionWidget::RegularizedSpatialFrictionWidget() {

    layout = new QVBoxLayout;
    layout->setMargin(0);
    funcList = new CustomComboBox;
    funcList->addItem(tr("Linear regularized Coulomb friction"));
    funcList->addItem(tr("Linear regularized Stribeck friction"));
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
    else {
      layout->removeWidget(frictionForceFunc);
      delete frictionForceFunc;
      frictionForceFunc = new LinearRegularizedStribeckFrictionWidget;
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

  PlanarStribeckImpactWidget::PlanarStribeckImpactWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget2(new FunctionWidgetFactory2(NULL)),false);
    layout->addWidget(frictionFunction);
  }

  SpatialStribeckImpactWidget::SpatialStribeckImpactWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget2(new FunctionWidgetFactory2(NULL)),false);
    layout->addWidget(frictionFunction);
  }

  GeneralizedForceLawChoiceWidget::GeneralizedForceLawChoiceWidget() : generalizedForceLaw(0) {

    layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    comboBox = new CustomComboBox;
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

    comboBox = new CustomComboBox;
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

    comboBox = new CustomComboBox;
    comboBox->addItem(tr("Planar Coulomb friction"));
    comboBox->addItem(tr("Planar Stribeck friction"));
    comboBox->addItem(tr("Regularized planar friction"));
    comboBox->addItem(tr("Spatial Coulomb friction"));
    comboBox->addItem(tr("Spatial Stribeck friction"));
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
    else if(index==1)
      frictionForceLaw = new PlanarStribeckFrictionWidget;
    else if(index==2)
      frictionForceLaw = new RegularizedPlanarFrictionWidget;  
    else if(index==3)
      frictionForceLaw = new SpatialCoulombFrictionWidget;  
    else if(index==4)
      frictionForceLaw = new SpatialStribeckFrictionWidget;
    else if(index==5)
      frictionForceLaw = new RegularizedSpatialFrictionWidget;  
    layout->addWidget(frictionForceLaw);
  }

  FrictionImpactLawChoiceWidget::FrictionImpactLawChoiceWidget() : frictionImpactLaw(0) {

    layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    comboBox = new CustomComboBox;
    comboBox->addItem(tr("Planar Coloumb impact"));
    comboBox->addItem(tr("Planar Stribeck impact"));
    comboBox->addItem(tr("Spatial Coloumb impact"));
    comboBox->addItem(tr("Spatial Stribeck impact"));
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
      frictionImpactLaw = new PlanarStribeckImpactWidget;
    else if(index==2)
      frictionImpactLaw = new SpatialCoulombImpactWidget;  
    else if(index==3)
      frictionImpactLaw = new SpatialStribeckImpactWidget;
    layout->addWidget(frictionImpactLaw);
  }

}
