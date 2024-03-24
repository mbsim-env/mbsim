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
#include "constraint_property_dialog.h"
#include "function_widget_factory.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "frame.h"
#include "rigid_body.h"
#include "signal_.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  ConstraintPropertyDialog::ConstraintPropertyDialog(Element *constraint) : ElementPropertyDialog(constraint) {
  }

  MechanicalConstraintPropertyDialog::MechanicalConstraintPropertyDialog(Element *constraint) : ConstraintPropertyDialog(constraint) {
  }

  GeneralizedConstraintPropertyDialog::GeneralizedConstraintPropertyDialog(Element *constraint) : MechanicalConstraintPropertyDialog(constraint) {

    addTab("Visualization",2);

    support = new ExtWidget("Support frame",new ElementOfReferenceWidget<Frame>(constraint,nullptr,this),true,false,MBSIM%"supportFrame");
    addToTab("General",support);
  }

  DOMElement* GeneralizedConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    MechanicalConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    support->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    MechanicalConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    support->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedGearConstraintPropertyDialog::GeneralizedGearConstraintPropertyDialog(Element *constraint) : GeneralizedConstraintPropertyDialog(constraint) {

    dependentBody = new ExtWidget("Dependent rigid body",new ElementOfReferenceWidget<RigidBody>(constraint,nullptr,this),false,false,MBSIM%"dependentRigidBody");
    addToTab("General", dependentBody);

    independentBodies = new ExtWidget("Independent rigid bodies",new ElementsOfReferenceWidget<RigidBody>(MBSIM%"independentRigidBody",constraint,1,100,true,this),false,false,"",true);
    addToTab("General",independentBodies);
  }

  DOMElement* GeneralizedGearConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GeneralizedConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    dependentBody->initializeUsingXML(item->getXMLElement());
    independentBodies->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedGearConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GeneralizedConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    dependentBody->writeXMLFile(item->getXMLElement(),ref);
    independentBodies->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedDualConstraintPropertyDialog::GeneralizedDualConstraintPropertyDialog(Element *constraint) : GeneralizedConstraintPropertyDialog(constraint) {

    dependentBody = new ExtWidget("Dependent rigid body",new ElementOfReferenceWidget<RigidBody>(constraint,nullptr,this),false,false,MBSIM%"dependentRigidBody");
    addToTab("General", dependentBody);

    independentBody = new ExtWidget("Independent rigid body",new ElementOfReferenceWidget<RigidBody>(constraint,nullptr,this),true,false,MBSIM%"independentRigidBody");
    addToTab("General", independentBody);
  }

  DOMElement* GeneralizedDualConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GeneralizedConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    dependentBody->initializeUsingXML(item->getXMLElement());
    independentBody->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedDualConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GeneralizedConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    dependentBody->writeXMLFile(item->getXMLElement(),ref);
    independentBody->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedPositionConstraintPropertyDialog::GeneralizedPositionConstraintPropertyDialog(Element *constraint) : GeneralizedDualConstraintPropertyDialog(constraint) {

    constraintFunction = new ExtWidget("Constraint function",new ChoiceWidget(new Function1ArgWidgetFactory(constraint,"t",1,FunctionWidget::scalar,1,FunctionWidget::fixedVec,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"constraintFunction");
    addToTab("General", constraintFunction);
    connect(constraintFunction->getWidget<ChoiceWidget>(),&Widget::widgetChanged,this,&GeneralizedPositionConstraintPropertyDialog::updateWidget);
  }

  void GeneralizedPositionConstraintPropertyDialog::updateWidget() {
    //    RigidBody *refBody = static_cast<RigidBodyOfReferenceWidget*>(dependentBody->getWidget())->getSelectedBody();
    //    int size = refBody?refBody->getqRelSize():0;
    //    constraintFunction->resize_(size,1);
  }

  DOMElement* GeneralizedPositionConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GeneralizedDualConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    constraintFunction->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedPositionConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GeneralizedDualConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    constraintFunction->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedVelocityConstraintPropertyDialog::GeneralizedVelocityConstraintPropertyDialog(Element *constraint) : GeneralizedDualConstraintPropertyDialog(constraint) {
    addTab("Initial conditions",1);

    x0 = new ExtWidget("Initial state",new ChoiceWidget(new VecWidgetFactory(0,vector<QStringList>(3,QStringList())),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialState");
    addToTab("Initial conditions", x0);

    constraintFunction = new ExtWidget("Constraint function",new ChoiceWidget(new ConstraintWidgetFactory(constraint,this),QBoxLayout::TopToBottom,3),false,false,"",true);
    addToTab("General", constraintFunction);
    connect(constraintFunction->getWidget<ChoiceWidget>(),&Widget::widgetChanged,this,&GeneralizedVelocityConstraintPropertyDialog::updateWidget);
  }

  void GeneralizedVelocityConstraintPropertyDialog::updateWidget() {
    cerr << "GeneralizedVelocityConstraintPropertyDialog::updateWidget() not yet implemented" << endl;
  }

  DOMElement* GeneralizedVelocityConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GeneralizedDualConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    x0->initializeUsingXML(item->getXMLElement());
    constraintFunction->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedVelocityConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GeneralizedDualConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    x0->writeXMLFile(item->getXMLElement(),ref);
    constraintFunction->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedAccelerationConstraintPropertyDialog::GeneralizedAccelerationConstraintPropertyDialog(Element *constraint) : GeneralizedDualConstraintPropertyDialog(constraint) {
    addTab("Initial conditions",1);

    x0 = new ExtWidget("Initial state",new ChoiceWidget(new VecWidgetFactory(0,vector<QStringList>(3,QStringList())),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialState");
    addToTab("Initial conditions", x0);

    constraintFunction = new ExtWidget("Constraint function",new ChoiceWidget(new ConstraintWidgetFactory(constraint,this),QBoxLayout::TopToBottom,3),false,false,"",true);
    addToTab("General", constraintFunction);
    connect(constraintFunction->getWidget<ChoiceWidget>(),&Widget::widgetChanged,this,&GeneralizedAccelerationConstraintPropertyDialog::updateWidget);
  }

  void GeneralizedAccelerationConstraintPropertyDialog::updateWidget() {
    //    RigidBody *refBody = static_cast<RigidBodyOfReferenceWidget*>(dependentBody->getWidget())->getSelectedBody();
    //    int size = refBody?(refBody->getqRelSize()+refBody->getuRelSize()):0;
    //    static_cast<ChoiceWidget*>(constraintFunction->getWidget())->resize_(size,1);
    //    if(x0_ && x0_->size() != size)
    //      x0_->resize_(size);
    //    static_cast<FunctionWidget*>(static_cast<ChoiceWidget*>(constraintFunction->getWidget())->getWidget())->setArg1Size(size);
  }

  DOMElement* GeneralizedAccelerationConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GeneralizedDualConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    x0->initializeUsingXML(item->getXMLElement());
    constraintFunction->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedAccelerationConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GeneralizedDualConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    x0->writeXMLFile(item->getXMLElement(),ref);
    constraintFunction->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  JointConstraintPropertyDialog::JointConstraintPropertyDialog(Element *constraint) : MechanicalConstraintPropertyDialog(constraint) {

    addTab("Kinetics",1);
    addTab("Visualization",2);
    addTab("Initial conditions",2);

    dependentBodiesFirstSide = new ExtWidget("Dependent bodies on first side",new ElementsOfReferenceWidget<Signal>(MBSIM%"dependentRigidBodyOnFirstSide",constraint,0,100,this),false,false,"",true);
    addToTab("General",dependentBodiesFirstSide);
    connect(dependentBodiesFirstSide->getWidget<ElementsOfReferenceWidget<Signal>>(),&Widget::widgetChanged,this,&JointConstraintPropertyDialog::updateWidget);

    dependentBodiesSecondSide = new ExtWidget("Dependent bodies on second side",new ElementsOfReferenceWidget<Signal>(MBSIM%"dependentRigidBodyOnSecondSide",constraint,0,100,this),false,false,"",true);
    addToTab("General",dependentBodiesSecondSide);
    connect(dependentBodiesSecondSide->getWidget<ElementsOfReferenceWidget<Signal>>(),&Widget::widgetChanged,this,&JointConstraintPropertyDialog::updateWidget);

    independentBody = new ExtWidget("Independent rigid body",new ElementOfReferenceWidget<RigidBody>(constraint,nullptr,this),false,false,MBSIM%"independentRigidBody");
    addToTab("General", independentBody);

    connections = new ExtWidget("Connections",new ConnectElementsWidget<Frame>(2,constraint,this),false,false,MBSIM%"connect");
    addToTab("Kinetics", connections);

    vector<QString> list;
    list.emplace_back("\"firstFrame\"");
    list.emplace_back("\"secondFrame\"");
    refFrame = new ExtWidget("Frame of reference",new TextChoiceWidget(list,0,true),true,false,MBSIM%"frameOfReference");
    addToTab("Kinetics", refFrame);

    force = new ExtWidget("Force direction",new ChoiceWidget(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"forceDirection");
    addToTab("Kinetics", force);

    moment = new ExtWidget("Moment direction",new ChoiceWidget(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"momentDirection");
    addToTab("Kinetics", moment);

    q0 = new ExtWidget("Initial guess",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialGuess");
    addToTab("Initial conditions", q0);
  }

  void JointConstraintPropertyDialog::updateWidget() {
    //    int size = 0;
    //    ListWidget *list = static_cast<ListWidget*>(dependentBodiesFirstSide->getWidget());
    //    for(int i=0; i<list->getSize(); i++) {
    //      RigidBody *body = static_cast<RigidBodyOfReferenceWidget*>(list->getWidget(i))->getSelectedBody();
    //      if(body)
    //        size += body->getqRelSize();
    //    }
    //    list = static_cast<ListWidget*>(dependentBodiesSecondSide->getWidget());
    //    for(int i=0; i<list->getSize(); i++) {
    //      RigidBody *body = static_cast<RigidBodyOfReferenceWidget*>(list->getWidget(i))->getSelectedBody();
    //      if(body)
    //        size += body->getqRelSize();
    //    }
    //    if(q0_->size() != size)
    //      q0_->resize_(size);
  }

  DOMElement* JointConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    MechanicalConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    dependentBodiesFirstSide->initializeUsingXML(item->getXMLElement());
    dependentBodiesSecondSide->initializeUsingXML(item->getXMLElement());
    independentBody->initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    refFrame->initializeUsingXML(item->getXMLElement());
    force->initializeUsingXML(item->getXMLElement());
    moment->initializeUsingXML(item->getXMLElement());
    q0->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* JointConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    MechanicalConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    dependentBodiesFirstSide->writeXMLFile(item->getXMLElement(),ref);
    dependentBodiesSecondSide->writeXMLFile(item->getXMLElement(),ref);
    independentBody->writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    refFrame->writeXMLFile(item->getXMLElement(),ref);
    force->writeXMLFile(item->getXMLElement(),ref);
    moment->writeXMLFile(item->getXMLElement(),ref);
    q0->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  InverseKinematicsConstraintPropertyDialog::InverseKinematicsConstraintPropertyDialog(Element *constraint) : ConstraintPropertyDialog(constraint) {

    addTab("Kinematics",1);
    addTab("Visualization",2);
    addTab("Initial conditions",2);

    vector<QString> list;
    list.emplace_back("\"planar\"");
    list.emplace_back("\"spatial\"");
    kinematics = new ExtWidget("Kinematics",new TextChoiceWidget(list,1,true),true,false,MBSIM%"kinematics");
    addToTab("General", kinematics);

    frame = new ExtWidget("Frame",new ElementOfReferenceWidget<Frame>(constraint,nullptr,this),false,false,MBSIM%"frame");
    addToTab("General", frame);

    translation = new ExtWidget("Translation",new ChoiceWidget(new TimeDependentTranslationWidgetFactory(constraint,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"translation");
    addToTab("Kinematics", translation);

    rotation = new ExtWidget("Rotation",new ChoiceWidget(new TimeDependentRotationWidgetFactory(constraint,this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"rotation");
    addToTab("Kinematics", rotation);

    q0 = new ExtWidget("Initial guess",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialGuess");
    addToTab("Initial conditions", q0);
  }

  DOMElement* InverseKinematicsConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    kinematics->initializeUsingXML(item->getXMLElement());
    frame->initializeUsingXML(item->getXMLElement());
    translation->initializeUsingXML(item->getXMLElement());
    rotation->initializeUsingXML(item->getXMLElement());
    q0->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* InverseKinematicsConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    kinematics->writeXMLFile(item->getXMLElement(),ref);
    frame->writeXMLFile(item->getXMLElement(),ref);
    translation->writeXMLFile(item->getXMLElement(),ref);
    rotation->writeXMLFile(item->getXMLElement(),ref);
    q0->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedConnectionConstraintPropertyDialog::GeneralizedConnectionConstraintPropertyDialog(Element *constraint) : GeneralizedDualConstraintPropertyDialog(constraint) {
  }

}
