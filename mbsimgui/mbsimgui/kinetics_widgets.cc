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
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  DOMElement* GeneralizedForceLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getNameSpace()%getType().toStdString());
    if(forceFunc) {
      DOMElement *ele1 = D(doc)->createElement( MBSIM%"forceFunction" );
      forceFunc->writeXMLFile(ele1);
      ele0->insertBefore(ele1, NULL);
    }
    parent->insertBefore(ele0, ref);
    return ele0;
  }

  RegularizedBilateralConstraintWidget::RegularizedBilateralConstraintWidget() {

    layout = new QVBoxLayout;
    layout->setMargin(0);
    funcList = new CustomComboBox;
    funcList->addItem(tr("Linear regularized bilateral constraint"));
    layout->addWidget(funcList);
    setLayout(layout);
    connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
//    forceFunc = new LinearRegularizedBilateralConstraintWidget;
//    layout->addWidget(forceFunc);
  }

  void RegularizedBilateralConstraintWidget::defineFunction(int index) {
    if(index==0) {
      layout->removeWidget(forceFunc);
      delete forceFunc;
//      forceFunc = new LinearRegularizedBilateralConstraintWidget;
//      layout->addWidget(forceFunc);
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
//    forceFunc = new LinearRegularizedUnilateralConstraintWidget;
//    layout->addWidget(forceFunc);
  }

  void RegularizedUnilateralConstraintWidget::defineFunction(int index) {
    if(index==0) {
      layout->removeWidget(forceFunc);
      delete forceFunc;
//      forceFunc = new LinearRegularizedUnilateralConstraintWidget;
//      layout->addWidget(forceFunc);
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

  DOMElement* FrictionForceLawWidget::initializeUsingXML(DOMElement *element) {
    if(frictionForceFunc) frictionForceFunc->initializeUsingXML(element);
    return element;
  }

  DOMElement* FrictionForceLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getNameSpace()%getType().toStdString());
    parent->insertBefore(ele0, ref);
    if(frictionForceFunc) frictionForceFunc->writeXMLFile(ele0);
    return ele0;
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
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    frictionForceFunc = new ExtWidget("Friction force function",new ChoiceWidget2(new FrictionFunctionFactory,QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionForceFunction");
    layout->addWidget(frictionForceFunc);
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
//    frictionForceFunc = new LinearRegularizedCoulombFrictionWidget;
//    layout->addWidget(frictionForceFunc);
  }

  void RegularizedSpatialFrictionWidget::defineFunction(int index) {
    if(index==0) {
      layout->removeWidget(frictionForceFunc);
      delete frictionForceFunc;
//      frictionForceFunc = new LinearRegularizedCoulombFrictionWidget;
//      layout->addWidget(frictionForceFunc);
    }
    else {
      layout->removeWidget(frictionForceFunc);
      delete frictionForceFunc;
//      frictionForceFunc = new LinearRegularizedStribeckFrictionWidget;
//      layout->addWidget(frictionForceFunc);
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

  GeneralizedForceLawWidgetFactory::GeneralizedForceLawWidgetFactory() {
    name.push_back("Bilateral constraint");
    name.push_back("Regularized bilateral constraint");
    name.push_back("Unilateral constraint");
    name.push_back("Regularized unilateral constraint");
    xmlName.push_back(MBSIM%"BilateralConstraint");
    xmlName.push_back(MBSIM%"RegularizedBilateralConstraint");
    xmlName.push_back(MBSIM%"UnilateralConstraint");
    xmlName.push_back(MBSIM%"RegularizedUnilateralConstraint");
  }

  QWidget* GeneralizedForceLawWidgetFactory::createWidget(int i) {
    if(i==0)
      return new BilateralConstraintWidget;
    if(i==1)
      return new RegularizedBilateralConstraintWidget;
    if(i==2)
      return new UnilateralConstraintWidget;
    if(i==3)
      return new RegularizedUnilateralConstraintWidget;
    return NULL;
  }

  FrictionForceLawWidgetFactory::FrictionForceLawWidgetFactory() {
    name.push_back("Planar Coulomb friction");
    name.push_back("Planar Stribeck friction");
    name.push_back("Regularized planar friction");
    name.push_back("Spatial Coulomb friction");
    name.push_back("Spatial Stribeck friction");
    name.push_back("Regularized spatial friction");
    xmlName.push_back(MBSIM%"PlanarCoulombFriction");
    xmlName.push_back(MBSIM%"PlanarStribeckFriction");
    xmlName.push_back(MBSIM%"RegularizedPlanarFriction");
    xmlName.push_back(MBSIM%"SpatialCoulombFriction");
    xmlName.push_back(MBSIM%"SpatialStribeckFriction");
    xmlName.push_back(MBSIM%"RegularizedSpatialFriction");
  }

  QWidget* FrictionForceLawWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PlanarCoulombFrictionWidget;
    if(i==1)
      return new PlanarStribeckFrictionWidget;
    if(i==2)
      return new RegularizedPlanarFrictionWidget;
    if(i==3)
      return new SpatialCoulombFrictionWidget;
    if(i==4)
      return new SpatialStribeckFrictionWidget;
    if(i==5)
      return new RegularizedSpatialFrictionWidget;
    return NULL;
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

  FrictionFunctionFactory::FrictionFunctionFactory() {
    name.push_back("Linear regularized Coulomb friction");
    name.push_back("Linear regularized Stribeck friction");
    name.push_back("Symbolic function");
    xmlName.push_back(MBSIM%"LinearRegularizedCoulombFriction");
    xmlName.push_back(MBSIM%"LinearRegularizedStribeckFriction");
    xmlName.push_back(MBSIM%"SymbolicFunction");
  }

  QWidget* FrictionFunctionFactory::createWidget(int i) {
    if(i==0)
      return new LinearRegularizedCoulombFrictionWidget;
    if(i==1)
      return new LinearRegularizedStribeckFrictionWidget;
    if(i==2) {
      QStringList var;
      var << "gd" << "laN";
      return new SymbolicFunctionWidget(var,1,1);
    }
    return NULL;
  }

}
