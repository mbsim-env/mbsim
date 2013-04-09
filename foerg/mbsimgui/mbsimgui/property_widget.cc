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
#include "property_widget.h"
#include "basic_widgets.h"
#include "string_widgets.h"
#include "kinematics_widgets.h"
#include "kinetics_widgets.h"
#include "ombv_widgets.h"
#include "extended_widgets.h"
#include "frame.h"
#include "rigidbody.h"
#include "solver.h"
#include "joint.h"
#include <iostream>
#include <QtGui>

using namespace std;

PropertyDialog::PropertyDialog(QWidget *parent, Qt::WindowFlags f) : QDialog(parent,f) {

  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  tabWidget = new QTabWidget(this);
  layout->addWidget(tabWidget);
  buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Apply | QDialogButtonBox::Cancel);
  buttonBox->addButton("Resize", QDialogButtonBox::ActionRole);

  //connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  //connect(buttonBox, SIGNAL(rejected()), this, SLOT(close()));
  connect(buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(clicked(QAbstractButton*)));
  layout->addWidget(buttonBox);
  setWindowTitle(QString("Properties"));
}

PropertyDialog::~PropertyDialog() {
}

void PropertyDialog::clicked(QAbstractButton *button) {
  if(button == buttonBox->button(QDialogButtonBox::Ok))
    emit ok(this);
  else if(button == buttonBox->button(QDialogButtonBox::Apply))
    emit apply(this);
  else if(button == buttonBox->button(QDialogButtonBox::Cancel))
    emit cancel(this);
  else if(button == buttonBox->buttons()[2])
    resizeVariables();
}

void PropertyDialog::addToTab(const QString &name, QWidget* widget_) {
  layout[name]->addWidget(widget_);
  widget.push_back(widget_);
}

void PropertyDialog::addStretch() {
  for ( std::map<QString,QVBoxLayout*>::iterator it=layout.begin() ; it != layout.end(); it++ )
    (*it).second->addStretch(1);
}

void PropertyDialog::updateWidget() {
  for(unsigned int i=0; i<widget.size(); i++)
    dynamic_cast<WidgetInterface*>(widget[i])->updateWidget();
}

void PropertyDialog::resizeVariables() {
  Object *obj = dynamic_cast<Object*>(parentObject);
  //if(obj) obj->resizeVariables();
  for(unsigned int i=0; i<widget.size(); i++)
    dynamic_cast<WidgetInterface*>(widget[i])->resizeVariables();
}

void PropertyDialog::addTab(const QString &name, int i) {  
  QScrollArea *tab = new QScrollArea;
  tab->setWidgetResizable(true);

  QWidget *box = new QWidget;
  QVBoxLayout *layout_ = new QVBoxLayout;
  box->setLayout(layout_);
  layout[name] = layout_;

  tab->setWidget(box);
  if(i==-1)
    tabWidget->addTab(tab, name);
  else 
    tabWidget->insertTab(i,tab,name);
}

void PropertyDialog::setParentObject(QObject *parentObject_) {
  parentObject=parentObject_;
}

ElementPropertyDialog::ElementPropertyDialog(QWidget *parent, Qt::WindowFlags f) : PropertyDialog(parent,f) {
  addTab("General");
  textWidget = new TextWidget;
  ExtWidget *name=new ExtWidget("Name",textWidget);
  addToTab("General",name);
}

void ElementPropertyDialog::toWidget(Element *element) {
  textWidget->setName(QString::fromStdString(element->getName()));
}

void ElementPropertyDialog::fromWidget(Element *element) {
  element->setName(textWidget->getName().toStdString());
}

FramePropertyDialog::FramePropertyDialog(Frame *frame, QWidget *parent, Qt::WindowFlags f) : ElementPropertyDialog(parent,f) {
  addTab("Visualisation");
  visuWidget = new ExtWidget("OpenMBV frame",new OMBVFrameWidget("NOTSET"),true,true);
  addToTab("Visualisation", visuWidget);
}

void FramePropertyDialog::toWidget(Element *element) {
  ElementPropertyDialog::toWidget(element);
  static_cast<Frame*>(element)->visuProperty.toWidget(visuWidget);
}

void FramePropertyDialog::fromWidget(Element *element) {
  ElementPropertyDialog::fromWidget(element);
  static_cast<Frame*>(element)->visuProperty.fromWidget(visuWidget);
}

FixedRelativeFramePropertyDialog::FixedRelativeFramePropertyDialog(FixedRelativeFrame *frame, QWidget *parent, Qt::WindowFlags f) : FramePropertyDialog(frame,parent,f) {
  addTab("Kinematics",1);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(3), lengthUnits(), 4));
  positionWidget = new ExtWidget("Relative position", new ExtPhysicalVarWidget(input),true);
  addToTab("Kinematics", positionWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new MatWidget(getEye<string>(3,3,"1","0")),noUnitUnits(),1));
  orientationWidget = new ExtWidget("Relative orientation",new ExtPhysicalVarWidget(input),true);
  addToTab("Kinematics", orientationWidget);

  refFrameWidget = new ExtWidget("Frame of reference",new ParentFrameOfReferenceWidget(frame,frame),true);
  addToTab("Kinematics", refFrameWidget);
}

void FixedRelativeFramePropertyDialog::toWidget(Element *element) {
  FramePropertyDialog::toWidget(element);
  static_cast<FixedRelativeFrame*>(element)->positionProperty.toWidget(positionWidget);
  static_cast<FixedRelativeFrame*>(element)->orientationProperty.toWidget(orientationWidget);
  static_cast<FixedRelativeFrame*>(element)->refFrameProperty.toWidget(refFrameWidget);
}

void FixedRelativeFramePropertyDialog::fromWidget(Element *element) {
  FramePropertyDialog::fromWidget(element);
  static_cast<FixedRelativeFrame*>(element)->positionProperty.fromWidget(positionWidget);
  static_cast<FixedRelativeFrame*>(element)->orientationProperty.fromWidget(orientationWidget);
  static_cast<FixedRelativeFrame*>(element)->refFrameProperty.fromWidget(refFrameWidget);
}

GroupPropertyDialog::GroupPropertyDialog(Group *group, QWidget *parent, Qt::WindowFlags f, bool disabled) : ElementPropertyDialog(parent,f), positionWidget(0), orientationWidget(0), frameOfReferenceWidget(0) {
  if(!disabled) {
    addTab("Kinematics");

    vector<PhysicalStringWidget*> input;
    input.push_back(new PhysicalStringWidget(new VecWidget(3),lengthUnits(),4));
    positionWidget = new ExtWidget("Position",new ExtPhysicalVarWidget(input),true); 
    addToTab("Kinematics", positionWidget);

    input.clear();
    input.push_back(new PhysicalStringWidget(new MatWidget(getEye<string>(3,3,"1","0")),noUnitUnits(),1));
    orientationWidget = new ExtWidget("Orientation",new ExtPhysicalVarWidget(input),true); 
    addToTab("Kinematics", orientationWidget);

    frameOfReferenceWidget = new ExtWidget("Frame of reference",new ParentFrameOfReferenceWidget(group,0),true);
    addToTab("Kinematics", frameOfReferenceWidget);
  }
}

void GroupPropertyDialog::toWidget(Element *element) {
  ElementPropertyDialog::toWidget(element);
  if(positionWidget) {
    static_cast<Group*>(element)->position.toWidget(positionWidget);
    static_cast<Group*>(element)->orientation.toWidget(orientationWidget);
    static_cast<Group*>(element)->frameOfReference.toWidget(frameOfReferenceWidget);
  }
}

void GroupPropertyDialog::fromWidget(Element *element) {
  ElementPropertyDialog::fromWidget(element);
  if(positionWidget) {
    static_cast<Group*>(element)->position.fromWidget(positionWidget);
    static_cast<Group*>(element)->orientation.fromWidget(orientationWidget);
    static_cast<Group*>(element)->frameOfReference.fromWidget(frameOfReferenceWidget);
  }
}

SolverPropertyDialog::SolverPropertyDialog(Solver *solver, QWidget *parent, Qt::WindowFlags f) : GroupPropertyDialog(solver,parent,f,true) {
  addTab("Environment");
  addTab("Solver parameters");
  addTab("Extra");

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(vector<string>(3)),accelerationUnits(),0));
  environmentWidget = new ExtWidget("Acceleration of gravity",new ExtPhysicalVarWidget(input));
  addToTab("Environment", environmentWidget);

  solverParametersWidget = new ExtWidget("Solver parameters",new SolverParametersWidget,true); 
  addToTab("Solver parameters",solverParametersWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new BoolWidget("1"),QStringList(),1));
  inverseKineticsWidget = new ExtWidget("Inverse kinetics",new ExtPhysicalVarWidget(input),true); 
  addToTab("Extra", inverseKineticsWidget);
}

void SolverPropertyDialog::toWidget(Element *element) {
  GroupPropertyDialog::toWidget(element);
  static_cast<Solver*>(element)->environmentProperty.toWidget(environmentWidget);
  static_cast<Solver*>(element)->solverParametersProperty.toWidget(solverParametersWidget);
  static_cast<Solver*>(element)->inverseKineticsProperty.toWidget(inverseKineticsWidget);
}

void SolverPropertyDialog::fromWidget(Element *element) {
  GroupPropertyDialog::fromWidget(element);
  static_cast<Solver*>(element)->environmentProperty.fromWidget(environmentWidget);
  static_cast<Solver*>(element)->solverParametersProperty.fromWidget(solverParametersWidget);
  static_cast<Solver*>(element)->inverseKineticsProperty.fromWidget(inverseKineticsWidget);
}

ObjectPropertyDialog::ObjectPropertyDialog(Object *object, QWidget *parent, Qt::WindowFlags f) : ElementPropertyDialog(parent,f) {
  addTab("Initial conditions");
  vector<PhysicalStringWidget*> input;
  q0 = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(q0,QStringList(),1));
  ExtPhysicalVarWidget *var = new ExtPhysicalVarWidget(input);  
  q0Widget = new ExtWidget("Initial generalized position",var,true);
  addToTab("Initial conditions", q0Widget);

  input.clear();
  u0 = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(u0,QStringList(),1));
  var = new ExtPhysicalVarWidget(input);  
  u0Widget = new ExtWidget("Initial generalized velocity",var,true);
  addToTab("Initial conditions", u0Widget);
}

void ObjectPropertyDialog::toWidget(Element *element) {
  ElementPropertyDialog::toWidget(element);
  static_cast<Object*>(element)->q0Property.toWidget(q0Widget);
  static_cast<Object*>(element)->u0Property.toWidget(u0Widget);
}

void ObjectPropertyDialog::fromWidget(Element *element) {
  ElementPropertyDialog::fromWidget(element);
  static_cast<Object*>(element)->q0Property.fromWidget(q0Widget);
  static_cast<Object*>(element)->u0Property.fromWidget(u0Widget);
}

BodyPropertyDialog::BodyPropertyDialog(Body *body, QWidget *parent, Qt::WindowFlags f) : ObjectPropertyDialog(body,parent,f) {
}

void BodyPropertyDialog::toWidget(Element *element) {
  ObjectPropertyDialog::toWidget(element);
}

void BodyPropertyDialog::fromWidget(Element *element) {
  ObjectPropertyDialog::fromWidget(element);
}

RigidBodyPropertyDialog::RigidBodyPropertyDialog(RigidBody *body_, QWidget *parent, Qt::WindowFlags f) : BodyPropertyDialog(body_,parent,f), body(body_) {
  addTab("Kinematics");
  addTab("Visualisation");
  addTab("Extra");

  RWidget = new ExtWidget("Frame of reference",new FrameOfReferenceWidget(body,0),true);
  addToTab("Kinematics",RWidget);

  KWidget = new ExtWidget("Frame for kinematics",new LocalFrameOfReferenceWidget(body,0),true);
  addToTab("Kinematics",KWidget);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"),massUnits(),2));
  massWidget = new ExtWidget("Mass",new ExtPhysicalVarWidget(input));
  addToTab("General", massWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SymMatWidget(getEye<string>(3,3,"0.01","0")),inertiaUnits(),2));
  inertiaWidget = new ExtWidget("Inertia tensor",new ExtPhysicalVarWidget(input));
  addToTab("General", inertiaWidget);

  TranslationChoiceWidget *translationWidget_ = new TranslationChoiceWidget("");
  translationWidget = new ExtWidget("Translation",translationWidget_,true);
  addToTab("Kinematics", translationWidget);
  connect(translationWidget_,SIGNAL(translationChanged()),this,SLOT(resizeVariables()));
  connect(translationWidget,SIGNAL(resize()),this,SLOT(resizeVariables()));

  RotationChoiceWidget *rotationWidget_ = new RotationChoiceWidget("");
  rotationWidget = new ExtWidget("Rotation",rotationWidget_,true);
  addToTab("Kinematics", rotationWidget);
  connect(rotationWidget_,SIGNAL(rotationChanged()),this,SLOT(resizeVariables()));
  connect(rotationWidget,SIGNAL(resize()),this,SLOT(resizeVariables()));

  ombvEditorWidget = new ExtWidget("OpenMBV body",new OMBVBodySelectionWidget(body),true);
  addToTab("Visualisation", ombvEditorWidget);

  weightArrowWidget = new ExtWidget("OpenMBV weight arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",weightArrowWidget);

  jointForceArrowWidget = new ExtWidget("OpenMBV joint force arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",jointForceArrowWidget);

  jointMomentArrowWidget = new ExtWidget("OpenMBV joint moment arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",jointMomentArrowWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new BoolWidget("0"),QStringList(),1));
  isFrameOfBodyForRotationWidget = new ExtWidget("Use body frame for rotation",new ExtPhysicalVarWidget(input),true); 
  addToTab("Extra", isFrameOfBodyForRotationWidget);
}

void RigidBodyPropertyDialog::toWidget(Element *element) {
  BodyPropertyDialog::toWidget(element);
  static_cast<RigidBody*>(element)->R.toWidget(RWidget);
  static_cast<RigidBody*>(element)->K.toWidget(KWidget);
  static_cast<RigidBody*>(element)->mass.toWidget(massWidget);
  static_cast<RigidBody*>(element)->translation.toWidget(translationWidget);
  static_cast<RigidBody*>(element)->rotation.toWidget(rotationWidget);
  static_cast<RigidBody*>(element)->ombvEditor.toWidget(ombvEditorWidget);
  static_cast<RigidBody*>(element)->weightArrow.toWidget(weightArrowWidget);
  static_cast<RigidBody*>(element)->jointForceArrow.toWidget(jointForceArrowWidget);
  static_cast<RigidBody*>(element)->jointMomentArrow.toWidget(jointMomentArrowWidget);
  static_cast<RigidBody*>(element)->isFrameOfBodyForRotation.toWidget(isFrameOfBodyForRotationWidget);
}

void RigidBodyPropertyDialog::fromWidget(Element *element) {
  BodyPropertyDialog::fromWidget(element);
  static_cast<RigidBody*>(element)->R.fromWidget(RWidget);
  static_cast<RigidBody*>(element)->K.fromWidget(KWidget);
  static_cast<RigidBody*>(element)->mass.fromWidget(massWidget);
  static_cast<RigidBody*>(element)->translation.fromWidget(translationWidget);
  static_cast<RigidBody*>(element)->rotation.fromWidget(rotationWidget);
  static_cast<RigidBody*>(element)->ombvEditor.fromWidget(ombvEditorWidget);
  static_cast<RigidBody*>(element)->weightArrow.fromWidget(weightArrowWidget);
  static_cast<RigidBody*>(element)->jointForceArrow.fromWidget(jointForceArrowWidget);
  static_cast<RigidBody*>(element)->jointMomentArrow.fromWidget(jointMomentArrowWidget);
  static_cast<RigidBody*>(element)->isFrameOfBodyForRotation.fromWidget(isFrameOfBodyForRotationWidget);
}

int RigidBodyPropertyDialog::getSize() const {
  return (translationWidget->isActive()?((TranslationChoiceWidget*)translationWidget->getWidget())->getSize():0) + (rotationWidget->isActive()?((RotationChoiceWidget*)rotationWidget->getWidget())->getSize():0);
}

void RigidBodyPropertyDialog::resizeGeneralizedPosition() {
  int size =  body->isConstrained() ? 0 : getSize();
  if(q0 && q0->size() != size)
    q0->resize(size);
}

void RigidBodyPropertyDialog::resizeGeneralizedVelocity() {
  int size = getSize();
  if(u0 && u0->size() != size)
    u0->resize(size);
}

LinkPropertyDialog::LinkPropertyDialog(Link *link, QWidget *parent, Qt::WindowFlags f) : ElementPropertyDialog(parent,f) {
}

void LinkPropertyDialog::toWidget(Element *element) {
  ElementPropertyDialog::toWidget(element);
}

void LinkPropertyDialog::fromWidget(Element *element) {
  ElementPropertyDialog::fromWidget(element);
}

JointPropertyDialog::JointPropertyDialog(Joint *joint, QWidget *parent, Qt::WindowFlags f) : LinkPropertyDialog(joint,parent,f) {
  addTab("Kinetics");
  addTab("Visualisation");

  forceArrowWidget = new ExtWidget("OpenMBV force arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",forceArrowWidget);

  momentArrowWidget = new ExtWidget("OpenMBV moment arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",momentArrowWidget);

  connectionsWidget = new ExtWidget("Connections",new ConnectFramesWidget(2,joint));
  addToTab("Kinetics", connectionsWidget);

  forceWidget = new ExtWidget("Force",new GeneralizedForceChoiceWidget,true);
  addToTab("Kinetics", forceWidget);

  momentWidget = new ExtWidget("Moment",new GeneralizedForceChoiceWidget,true);
  addToTab("Kinetics", momentWidget);
}

void JointPropertyDialog::toWidget(Element *element) {
  ElementPropertyDialog::toWidget(element);
  static_cast<Joint*>(element)->forceArrow.toWidget(forceArrowWidget);
  static_cast<Joint*>(element)->momentArrow.toWidget(momentArrowWidget);
  static_cast<Joint*>(element)->connections.toWidget(connectionsWidget);
  static_cast<Joint*>(element)->force.toWidget(forceWidget);
  static_cast<Joint*>(element)->moment.toWidget(momentWidget);
}

void JointPropertyDialog::fromWidget(Element *element) {
  ElementPropertyDialog::fromWidget(element);
  static_cast<Joint*>(element)->forceArrow.fromWidget(forceArrowWidget);
  static_cast<Joint*>(element)->momentArrow.fromWidget(momentArrowWidget);
  static_cast<Joint*>(element)->connections.fromWidget(connectionsWidget);
  static_cast<Joint*>(element)->force.fromWidget(forceWidget);
  static_cast<Joint*>(element)->moment.fromWidget(momentWidget);
}

