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
#include "ombv_widgets.h"
#include "extended_widgets.h"
#include "frame.h"
#include "object.h"
#include "solver.h"
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


