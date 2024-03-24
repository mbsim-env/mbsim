/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2024 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#include <config.h>
#include "signal_property_dialog.h"
#include "function_widget_factory.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "frame.h"
#include "object.h"
#include "link_.h"
#include "constraint.h"
#include "mainwindow.h"
#include "signal_.h"
#include "contact.h"
#include "project.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  SignalPropertyDialog::SignalPropertyDialog(Element *signal) : LinkPropertyDialog(signal) {
  }

  SensorPropertyDialog::SensorPropertyDialog(Element *sensor) : SignalPropertyDialog(sensor) {
  }

  ObjectSensorPropertyDialog::ObjectSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    object = new ExtWidget("Object of reference",new ElementOfReferenceWidget<Object>(sensor,nullptr,this),false,false,MBSIMCONTROL%"object");
    addToTab("General", object);
  }

  DOMElement* ObjectSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    object->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ObjectSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    object->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  RigidBodyJointForceSensorPropertyDialog::RigidBodyJointForceSensorPropertyDialog(Element *sensor) : ObjectSensorPropertyDialog(sensor) {
    number = new ExtWidget("Joint force number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"jointForceNumber");
    addToTab("General", number);
  }

  DOMElement* RigidBodyJointForceSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ObjectSensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    number->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RigidBodyJointForceSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ObjectSensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    number->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  RigidBodyJointMomentSensorPropertyDialog::RigidBodyJointMomentSensorPropertyDialog(Element *sensor) : ObjectSensorPropertyDialog(sensor) {
    number = new ExtWidget("Joint moment number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"jointMomentNumber");
    addToTab("General", number);
  }

  DOMElement* RigidBodyJointMomentSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ObjectSensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    number->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RigidBodyJointMomentSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ObjectSensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    number->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  LinkSensorPropertyDialog::LinkSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    link = new ExtWidget("Link of reference",new ElementOfReferenceWidget<Link>(sensor,nullptr,this),false,false,MBSIMCONTROL%"link");
    addToTab("General", link);
  }

  DOMElement* LinkSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    link->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* LinkSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    link->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  MechanicalLinkForceSensorPropertyDialog::MechanicalLinkForceSensorPropertyDialog(Element *sensor) : LinkSensorPropertyDialog(sensor) {
    number = new ExtWidget("Force number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"forceNumber");
    addToTab("General", number);
  }

  DOMElement* MechanicalLinkForceSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    LinkSensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    number->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* MechanicalLinkForceSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    LinkSensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    number->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  MechanicalLinkMomentSensorPropertyDialog::MechanicalLinkMomentSensorPropertyDialog(Element *sensor) : LinkSensorPropertyDialog(sensor) {
    number = new ExtWidget("Moment number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"momentNumber");
    addToTab("General", number);
  }

  DOMElement* MechanicalLinkMomentSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    LinkSensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    number->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* MechanicalLinkMomentSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    LinkSensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    number->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ConstraintSensorPropertyDialog::ConstraintSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    constraint = new ExtWidget("Constraint of reference",new ElementOfReferenceWidget<Constraint>(sensor,nullptr,this),false,false,MBSIMCONTROL%"constraint");
    addToTab("General", constraint);
  }

  DOMElement* ConstraintSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    constraint->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ConstraintSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    constraint->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  MechanicalConstraintForceSensorPropertyDialog::MechanicalConstraintForceSensorPropertyDialog(Element *sensor) : ConstraintSensorPropertyDialog(sensor) {
    number = new ExtWidget("Force number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"forceNumber");
    addToTab("General", number);
  }

  DOMElement* MechanicalConstraintForceSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ConstraintSensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    number->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* MechanicalConstraintForceSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ConstraintSensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    number->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  MechanicalConstraintMomentSensorPropertyDialog::MechanicalConstraintMomentSensorPropertyDialog(Element *sensor) : ConstraintSensorPropertyDialog(sensor) {
    number = new ExtWidget("Moment number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"momentNumber");
    addToTab("General", number);
  }

  DOMElement* MechanicalConstraintMomentSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ConstraintSensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    number->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* MechanicalConstraintMomentSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ConstraintSensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    number->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  FrameSensorPropertyDialog::FrameSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    frame = new ExtWidget("Frame of reference",new ElementOfReferenceWidget<Frame>(sensor,nullptr,this),false,false,MBSIMCONTROL%"frame");
    addToTab("General", frame);

    outputFrame = new ExtWidget("Output frame",new ElementOfReferenceWidget<Frame>(sensor,nullptr,this),true,false,MBSIMCONTROL%"outputFrame");
    addToTab("General", outputFrame);
  }

  DOMElement* FrameSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    frame->initializeUsingXML(item->getXMLElement());
    outputFrame->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FrameSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    frame->writeXMLFile(item->getXMLElement(),ref);
    outputFrame->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  FunctionSensorPropertyDialog::FunctionSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    function = new ExtWidget("Function",new ChoiceWidget(new Function1ArgWidgetFactory(sensor,"t",1,FunctionWidget::scalar,1,FunctionWidget::varVec,this),QBoxLayout::TopToBottom,0),false,false,MBSIMCONTROL%"function");
    addToTab("General", function);
  }

  DOMElement* FunctionSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    function->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FunctionSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ContactSensorPropertyDialog::ContactSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    contact = new ExtWidget("Contact of reference",new ElementOfReferenceWidget<Contact>(sensor,nullptr,this),false,false,MBSIMCONTROL%"contact");
    addToTab("General", contact);
    number = new ExtWidget("Single contact number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"singleContactNumber");
    addToTab("General", number);
 }

  DOMElement* ContactSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    contact->initializeUsingXML(item->getXMLElement());
    number->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ContactSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    contact->writeXMLFile(item->getXMLElement(),ref);
    number->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  TyreContactSensorPropertyDialog::TyreContactSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    contact = new ExtWidget("Tyre Contact of reference",new ElementOfReferenceWidget<TyreContact>(sensor,nullptr,this),false,false,MBSIMCONTROL%"tyreContact");
    addToTab("General", contact);
 }

  DOMElement* TyreContactSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    contact->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* TyreContactSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    contact->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  TyreContactPositionSensorPropertyDialog::TyreContactPositionSensorPropertyDialog(Element *sensor) : TyreContactSensorPropertyDialog(sensor) {
    number = new ExtWidget("Position number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"positionNumber");
    addToTab("General", number);
 }

  DOMElement* TyreContactPositionSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    TyreContactSensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    number->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* TyreContactPositionSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    TyreContactSensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    number->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  TyreContactOrientationSensorPropertyDialog::TyreContactOrientationSensorPropertyDialog(Element *sensor) : TyreContactSensorPropertyDialog(sensor) {
    number = new ExtWidget("Orientation number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"orientationNumber");
    addToTab("General", number);
  }

  DOMElement* TyreContactOrientationSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    TyreContactSensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    number->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* TyreContactOrientationSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    TyreContactSensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    number->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  TyreContactVelocitySensorPropertyDialog::TyreContactVelocitySensorPropertyDialog(Element *sensor) : TyreContactSensorPropertyDialog(sensor) {
    number = new ExtWidget("Velocity number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"velocityNumber");
    addToTab("General", number);
 }

  DOMElement* TyreContactVelocitySensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    TyreContactSensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    number->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* TyreContactVelocitySensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    TyreContactSensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    number->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  TyreContactAngularVelocitySensorPropertyDialog::TyreContactAngularVelocitySensorPropertyDialog(Element *sensor) : TyreContactSensorPropertyDialog(sensor) {
    number = new ExtWidget("Angular velocity number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"angularVelocityNumber");
    addToTab("General", number);
 }

  DOMElement* TyreContactAngularVelocitySensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    TyreContactSensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    number->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* TyreContactAngularVelocitySensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    TyreContactSensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    number->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  MultiplexerPropertyDialog::MultiplexerPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {
    inputSignal = new ExtWidget("Input signals",new ElementsOfReferenceWidget<Signal>(MBSIMCONTROL%"inputSignal",signal,1,100,this),false,false,"",true);
    addToTab("General", inputSignal);
  }

  DOMElement* MultiplexerPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    inputSignal->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* MultiplexerPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    inputSignal->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  DemultiplexerPropertyDialog::DemultiplexerPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {
    inputSignal = new ExtWidget("Input signal",new ElementOfReferenceWidget<Signal>(signal,nullptr,this),false,false,MBSIMCONTROL%"inputSignal");
    addToTab("General", inputSignal);
    indices = new ExtWidget("Indices",new ChoiceWidget(new VecSizeVarWidgetFactory(1,1,100,1,vector<QStringList>(3,QStringList()),vector<int>(3,0),false,false,true,"1"),QBoxLayout::RightToLeft,5),false,false,MBSIMCONTROL%"indices");
    addToTab("General", indices);
  }

  DOMElement* DemultiplexerPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    inputSignal->initializeUsingXML(item->getXMLElement());
    indices->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DemultiplexerPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    inputSignal->writeXMLFile(item->getXMLElement(),ref);
    indices->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  LinearTransferSystemPropertyDialog::LinearTransferSystemPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {

    addTab("Initial conditions",1);

    x0 = new ExtWidget("Initial state",new ChoiceWidget(new VecWidgetFactory(0,vector<QStringList>(3,QStringList())),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"initialState");
    addToTab("Initial conditions", x0);

    inputSignal = new ExtWidget("Input signal",new ElementOfReferenceWidget<Signal>(signal,nullptr,this),false,false,MBSIMCONTROL%"inputSignal");
    addToTab("General", inputSignal);

    A = new ExtWidget("System matrix",new ChoiceWidget(new SqrMatSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),false,false,MBSIMCONTROL%"systemMatrix");
    addToTab("General", A);

    B = new ExtWidget("Input matrix",new ChoiceWidget(new MatColsVarWidgetFactory(1,1),QBoxLayout::RightToLeft,5),false,false,MBSIMCONTROL%"inputMatrix");
    addToTab("General", B);

    C = new ExtWidget("Output matrix",new ChoiceWidget(new MatRowsVarWidgetFactory(1,1),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"outputMatrix");
    addToTab("General", C);

    D = new ExtWidget("Feedthrough matrix",new ChoiceWidget(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"feedthroughMatrix");
    addToTab("General", D);

    connect(x0, &ExtWidget::widgetChanged, this, &LinearTransferSystemPropertyDialog::updateWidget);
    connect(A, &ExtWidget::widgetChanged, this, &LinearTransferSystemPropertyDialog::updateWidget);
    connect(B, &ExtWidget::widgetChanged, this, &LinearTransferSystemPropertyDialog::updateWidget);
    connect(C, &ExtWidget::widgetChanged, this, &LinearTransferSystemPropertyDialog::updateWidget);
    connect(D, &ExtWidget::widgetChanged, this, &LinearTransferSystemPropertyDialog::updateWidget);
  }

  void LinearTransferSystemPropertyDialog::updateWidget() {
    A->blockSignals(true);
    B->blockSignals(true);
    C->blockSignals(true);
    D->blockSignals(true);
    int n = A->getFirstWidget<PhysicalVariableWidget>()->rows();
    int m = B->getFirstWidget<PhysicalVariableWidget>()->cols();
    int p = C->isActive()?C->getFirstWidget<PhysicalVariableWidget>()->rows():m;
    x0->resize_(n,1);
    B->resize_(n,m);
    C->resize_(p,n);
    D->resize_(p,m);
    A->blockSignals(false);
    B->blockSignals(false);
    C->blockSignals(false);
    D->blockSignals(false);
  }

  DOMElement* LinearTransferSystemPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    x0->initializeUsingXML(item->getXMLElement());
    inputSignal->initializeUsingXML(item->getXMLElement());
    A->initializeUsingXML(item->getXMLElement());
    B->initializeUsingXML(item->getXMLElement());
    C->initializeUsingXML(item->getXMLElement());
    D->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* LinearTransferSystemPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    x0->writeXMLFile(item->getXMLElement(),ref);
    inputSignal->writeXMLFile(item->getXMLElement(),ref);
    A->writeXMLFile(item->getXMLElement(),ref);
    B->writeXMLFile(item->getXMLElement(),ref);
    C->writeXMLFile(item->getXMLElement(),ref);
    D->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  NonlinearTransferSystemPropertyDialog::NonlinearTransferSystemPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {

    addTab("Initial conditions",1);

    x0 = new ExtWidget("Initial state",new ChoiceWidget(new VecWidgetFactory(0,vector<QStringList>(3,QStringList())),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"initialState");
    addToTab("Initial conditions", x0);

    inputSignal = new ExtWidget("Input signal",new ElementOfReferenceWidget<Signal>(signal,nullptr,this),false,false,MBSIMCONTROL%"inputSignal");
    addToTab("General", inputSignal);

    F = new ExtWidget("System function",new ChoiceWidget(new Function2ArgWidgetFactory(getElement(),QStringList("x")<<"u",vector<int>(2,1),vector<FunctionWidget::VarType>(2,FunctionWidget::varVec),1,FunctionWidget::fixedVec,this),QBoxLayout::TopToBottom,0),false,false,MBSIMCONTROL%"systemFunction");
    addToTab("General", F);

    H = new ExtWidget("Output function",new ChoiceWidget(new Function2ArgWidgetFactory(getElement(),QStringList("x")<<"u",vector<int>(2,1),vector<FunctionWidget::VarType>(2,FunctionWidget::fixedVec),1,FunctionWidget::varVec,this),QBoxLayout::TopToBottom,0),true,false,MBSIMCONTROL%"outputFunction");
    addToTab("General", H);

    connect(x0, &ExtWidget::widgetChanged, this, &NonlinearTransferSystemPropertyDialog::updateWidget);
    connect(F, &ExtWidget::widgetChanged, this, &NonlinearTransferSystemPropertyDialog::updateWidget);
    connect(H, &ExtWidget::widgetChanged, this, &NonlinearTransferSystemPropertyDialog::updateWidget);
  }

  void NonlinearTransferSystemPropertyDialog::updateWidget() {
    F->blockSignals(true);
    H->blockSignals(true);
    int n = F->getFirstWidget<FunctionWidget>()->getArg1Size();
    int m = F->getFirstWidget<FunctionWidget>()->getArg2Size();
    x0->resize_(n,1);
    F->getFirstWidget<FunctionWidget>()->resize_(n,1);
    if(H->isActive()) {
      H->getFirstWidget<FunctionWidget>()->setArg1Size(n);
      H->getFirstWidget<FunctionWidget>()->setArg2Size(m);
    }
    F->blockSignals(false);
    H->blockSignals(false);
  }

  DOMElement* NonlinearTransferSystemPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    x0->initializeUsingXML(item->getXMLElement());
    inputSignal->initializeUsingXML(item->getXMLElement());
    F->initializeUsingXML(item->getXMLElement());
    H->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* NonlinearTransferSystemPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    x0->writeXMLFile(item->getXMLElement(),ref);
    inputSignal->writeXMLFile(item->getXMLElement(),ref);
    F->writeXMLFile(item->getXMLElement(),ref);
    H->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  SignalOperationPropertyDialog::SignalOperationPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {
    inputSignal = new ExtWidget("Input signals",new ElementsOfReferenceWidget<Signal>(MBSIMCONTROL%"inputSignal",signal,1,100,this),false,false,"",true);
    addToTab("General", inputSignal);

    multiplex = new ExtWidget("Multiplex input signals",new ChoiceWidget(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,true,MBSIMCONTROL%"multiplexInputSignals");
    addToTab("General", multiplex);

    function = new ExtWidget("Function",new ChoiceWidget(new Function1ArgWidgetFactory(signal,"u",1,FunctionWidget::varVec,1,FunctionWidget::varVec,this,17),QBoxLayout::TopToBottom,0),false,false,MBSIMCONTROL%"function");
    addToTab("General", function);

    connect(inputSignal,&ExtWidget::widgetChanged,this,&SignalOperationPropertyDialog::numberOfInputSignalsChanged);
    connect(multiplex,&ExtWidget::widgetChanged,this,&SignalOperationPropertyDialog::multiplexInputSignalsChanged);
  }

  void SignalOperationPropertyDialog::updateWidget() {
    function->updateWidget();
  }

  void SignalOperationPropertyDialog::numberOfInputSignalsChanged() {
    if((not multiplex->isActive()) or multiplex->getFirstWidget<PhysicalVariableWidget>()->getValue()==mw->getProject()->getVarFalse()) {
      if(inputSignal->getWidget<BasicElementsOfReferenceWidget>()->getSize()==2 and (not function->getWidget<ChoiceWidget>()->getWidgetFactory<Function2ArgWidgetFactory,true>()))
	function->getWidget<ChoiceWidget>()->setWidgetFactory(new Function2ArgWidgetFactory(getElement(),QStringList("u1")<<"u2",vector<int>(2,1),vector<FunctionWidget::VarType>(2,FunctionWidget::varVec),1,FunctionWidget::varVec,this));
      else if(not function->getWidget<ChoiceWidget>()->getWidgetFactory<Function1ArgWidgetFactory,true>())
	function->getWidget<ChoiceWidget>()->setWidgetFactory(new Function1ArgWidgetFactory(getElement(),"u",1,FunctionWidget::varVec,1,FunctionWidget::varVec,this,17));
    }
  }

  void SignalOperationPropertyDialog::multiplexInputSignalsChanged() {
    if(multiplex->isActive() and multiplex->getFirstWidget<PhysicalVariableWidget>()->getValue()==mw->getProject()->getVarTrue()) {
      inputSignal->getWidget<BasicElementsOfReferenceWidget>()->setRange(1,100);
      if(not function->getWidget<ChoiceWidget>()->getWidgetFactory<Function1ArgWidgetFactory,true>())
	function->getWidget<ChoiceWidget>()->setWidgetFactory(new Function1ArgWidgetFactory(getElement(),"u",1,FunctionWidget::varVec,1,FunctionWidget::varVec,this,17));
    }
    else {
      inputSignal->getWidget<BasicElementsOfReferenceWidget>()->setRange(1,2);
      if(inputSignal->getWidget<BasicElementsOfReferenceWidget>()->getSize()==2 and (not function->getWidget<ChoiceWidget>()->getWidgetFactory<Function2ArgWidgetFactory,true>()))
	function->getWidget<ChoiceWidget>()->setWidgetFactory(new Function2ArgWidgetFactory(getElement(),QStringList("u1")<<"u2",vector<int>(2,1),vector<FunctionWidget::VarType>(2,FunctionWidget::varVec),1,FunctionWidget::varVec,this));
    }
  }

  DOMElement* SignalOperationPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    inputSignal->initializeUsingXML(item->getXMLElement());
    multiplex->initializeUsingXML(item->getXMLElement());
    numberOfInputSignalsChanged();
    multiplexInputSignalsChanged();
    function->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* SignalOperationPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    inputSignal->writeXMLFile(item->getXMLElement(),ref);
    multiplex->writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ExternSignalSourcePropertyDialog::ExternSignalSourcePropertyDialog(Element *signal) : SignalPropertyDialog(signal) {
    sourceSize = new ExtWidget("Source size",new SpinBoxWidget(1,1,1000),false,false,MBSIMCONTROL%"sourceSize");
    addToTab("General", sourceSize);
  }

  DOMElement* ExternSignalSourcePropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    sourceSize->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ExternSignalSourcePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    sourceSize->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ExternSignalSinkPropertyDialog::ExternSignalSinkPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {
    inputSignal = new ExtWidget("Input signal",new ElementOfReferenceWidget<Signal>(signal,nullptr,this),false,false,MBSIMCONTROL%"inputSignal");
    addToTab("General", inputSignal);
  }

  DOMElement* ExternSignalSinkPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    inputSignal->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ExternSignalSinkPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    inputSignal->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  SwitchPropertyDialog::SwitchPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {

    dataSignal1 = new ExtWidget("First data input signal",new ElementOfReferenceWidget<Signal>(signal,nullptr,this),false,false,MBSIMCONTROL%"firstDataInputSignal");
    addToTab("General", dataSignal1);

    dataSignal2 = new ExtWidget("Second data input signal",new ElementOfReferenceWidget<Signal>(signal,nullptr,this),false,false,MBSIMCONTROL%"secondDataInputSignal");
    addToTab("General", dataSignal2);

    controlSignal = new ExtWidget("Control input signal",new ElementOfReferenceWidget<Signal>(signal,nullptr,this),false,false,MBSIMCONTROL%"controlInputSignal");
    addToTab("General", controlSignal);

    threshold = new ExtWidget("Threshold",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"threshold");
    addToTab("General", threshold);

    rootFinding = new ExtWidget("Root finding",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"rootFinding");
    addToTab("General", rootFinding);
  }

  DOMElement* SwitchPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    dataSignal1->initializeUsingXML(item->getXMLElement());
    dataSignal2->initializeUsingXML(item->getXMLElement());
    controlSignal->initializeUsingXML(item->getXMLElement());
    threshold->initializeUsingXML(item->getXMLElement());
    rootFinding->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* SwitchPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    dataSignal1->writeXMLFile(item->getXMLElement(),ref);
    dataSignal2->writeXMLFile(item->getXMLElement(),ref);
    controlSignal->writeXMLFile(item->getXMLElement(),ref);
    threshold->writeXMLFile(item->getXMLElement(),ref);
    rootFinding->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  DurationPropertyDialog::DurationPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {

    inputSignal = new ExtWidget("Input signal",new ElementOfReferenceWidget<Signal>(signal,nullptr,this),false,false,MBSIMCONTROL%"inputSignal");
    addToTab("General", inputSignal);

    threshold = new ExtWidget("Threshold",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"threshold");
    addToTab("General", threshold);
  }

  DOMElement* DurationPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    inputSignal->initializeUsingXML(item->getXMLElement());
    threshold->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DurationPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    inputSignal->writeXMLFile(item->getXMLElement(),ref);
    threshold->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  StateMachinePropertyDialog::StateMachinePropertyDialog(Element *signal) : SignalPropertyDialog(signal) {
    addTab("Initial conditions",1);

    state = new ExtWidget("States",new StateWidget,false,false,"",true);
    connect(state, &ExtWidget::widgetChanged, this, &StateMachinePropertyDialog::updateWidget);
    addToTab("General", state);

    transition = new ExtWidget("Transitions",new TransitionWidget(signal),false,false,"",true);
    addToTab("General", transition);

    initialState = new ExtWidget("Initial state",new TextChoiceWidget(vector<QString>(),0,true),true,false,MBSIMCONTROL%"initialState");
    addToTab("Initial conditions", initialState);
  }

  void StateMachinePropertyDialog::updateWidget() {
    initialState->getWidget<TextChoiceWidget>()->setStringList(state->getWidget<StateWidget>()->getNames());
    transition->getWidget<TransitionWidget>()->setStringList(state->getWidget<StateWidget>()->getNames());
  }

  DOMElement* StateMachinePropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    state->initializeUsingXML(item->getXMLElement());
    updateWidget();
    transition->initializeUsingXML(item->getXMLElement());
    initialState->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* StateMachinePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    state->writeXMLFile(item->getXMLElement(),ref);
    transition->writeXMLFile(item->getXMLElement(),ref);
    initialState->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  StateMachineSensorPropertyDialog::StateMachineSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    stateMachine = new ExtWidget("State machine",new ElementOfReferenceWidget<StateMachine>(sensor,nullptr,this),false,false,MBSIMCONTROL%"stateMachine");
    connect(stateMachine, &ExtWidget::widgetChanged, this, &StateMachineSensorPropertyDialog::updateWidget);
    addToTab("General", stateMachine);

    state = new ExtWidget("State",new TextChoiceWidget(vector<QString>(),0,true),true,false,MBSIMCONTROL%"state");
    addToTab("General", state);

    vector<QString> list;
    list.emplace_back("\"activity\"");
    list.emplace_back("\"durationOfActivity\"");
    list.emplace_back("\"value\"");
    selection = new ExtWidget("selection",new TextChoiceWidget(list,1,true),true,false,MBSIMCONTROL%"selection");
    addToTab("General", selection);
  }

  void StateMachineSensorPropertyDialog::updateWidget() {
    auto *sm = getElement()->getByPath<StateMachine>(stateMachine->getWidget<ElementOfReferenceWidget<StateMachine>>()->getElement());
    if(sm) {
      vector<QString> stringList;
      DOMElement *e=E(sm->getXMLElement())->getFirstElementChildNamed(MBSIMCONTROL%"state");
      while(e && (E(e)->getTagName()==MBSIMCONTROL%"state")) {
	stringList.emplace_back(QString::fromStdString(E(e)->getAttributeQName("name").second));
	e=e->getNextElementSibling();
      }
      state->getWidget<TextChoiceWidget>()->setStringList(stringList);
      state->getWidget<TextChoiceWidget>()->setCurrentIndex(0);
    }
  }

  DOMElement* StateMachineSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    stateMachine->initializeUsingXML(item->getXMLElement());
    updateWidget();
    state->initializeUsingXML(item->getXMLElement());
    selection->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* StateMachineSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    stateMachine->writeXMLFile(item->getXMLElement(),ref);
    state->writeXMLFile(item->getXMLElement(),ref);
    selection->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

}
