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
#include "element_context_menu.h"
#include "mainwindow.h"
#include "rigidbody.h"
#include "constraint.h"
#include "kinetic_excitation.h"
#include "spring_damper.h"
#include "joint.h"
#include "contact.h"
#include "signal_.h"
#include "observer.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include <QFileDialog>

extern MainWindow *mw;

ElementContextMenu::ElementContextMenu(Element *element_, QWidget *parent, bool removable) : QMenu(parent), element(element_) {
  if(removable) {
    QAction *action=new QAction("Save as", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(saveElementAs()));
    addAction(action);
    addSeparator();
    action=new QAction("Remove", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(removeElement()));
    addAction(action);
    addSeparator();
  }
}

void ElementContextMenu::addContour() {
  ContourContextContextMenu menu(element);
  menu.exec(QCursor::pos());
}

FrameContextMenu::FrameContextMenu(Element *frame, QWidget * parent, bool removable) : ElementContextMenu(frame,parent,removable) {
}

FixedRelativeFrameContextMenu::FixedRelativeFrameContextMenu(Element *frame, QWidget * parent) : FrameContextMenu(frame,parent,true) {
}

GroupContextMenu::GroupContextMenu(Element *element, QWidget *parent, bool removable) : ElementContextMenu(element,parent,removable) {
  QAction *action;
  action = new QAction("Add frame", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addFixedRelativeFrame()));
  addAction(action);
  action = new QAction("Add contour", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addContour()));
  addAction(action);
  action = new QAction("Add group", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addGroup()));
  addAction(action);
  action = new QAction("Add object", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addObject()));
  addAction(action);
  action = new QAction("Add link", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addLink()));
  addAction(action);
  action = new QAction("Add observer", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addObserver()));
  addAction(action);
  action = new QAction("Add element from File", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addElementFromFile()));
  addAction(action);
} 

void GroupContextMenu::addFixedRelativeFrame() {
  mw->addFrame(new FixedRelativeFrame("P",element));
}

void GroupContextMenu::addElementFromFile() {
  QString file=QFileDialog::getOpenFileName(0, "XML model files", ".", "XML files (*.xml)");
  if(file!="") {
    Frame *frame = Frame::readXMLFile(file.toStdString(),element);
    if(frame) return mw->addFrame(frame);
    Contour *contour = Contour::readXMLFile(file.toStdString(),element);
    if(contour) return mw->addContour(contour);
    Group *group = Group::readXMLFile(file.toStdString(),element);
    if(group) return mw->addGroup(group);
    Object *object = Object::readXMLFile(file.toStdString(),element);
    if(object) return mw->addObject(object);
    Link *link = Link::readXMLFile(file.toStdString(),element);
    if(link) return mw->addLink(link);
    Observer *observer = Observer::readXMLFile(file.toStdString(),element);
    if(observer) return mw->addObserver(observer);
  }
}

void GroupContextMenu::addGroup() {
  mw->addGroup(new Group("Group",element));
}

void GroupContextMenu::addObject() {
  ObjectContextContextMenu menu(element);
  menu.exec(QCursor::pos());
}

void GroupContextMenu::addLink() {
  LinkContextContextMenu menu(element);
  menu.exec(QCursor::pos());
}

void GroupContextMenu::addObserver() {
  ObserverContextContextMenu menu(element);
  menu.exec(QCursor::pos());
}

SolverContextMenu::SolverContextMenu(Element *solver, QWidget * parent) : GroupContextMenu(solver,parent,false) {
}

ObjectContextMenu::ObjectContextMenu(Element *element, QWidget *parent) : ElementContextMenu(element,parent,true) {
} 

BodyContextMenu::BodyContextMenu(Element *element, QWidget *parent) : ObjectContextMenu(element,parent) {
  QAction *action;
  action = new QAction("Add frame", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addFixedRelativeFrame()));
  addAction(action);
  action = new QAction("Add contour", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addContour()));
  addAction(action);
} 

void BodyContextMenu::addFixedRelativeFrame() {
  mw->addFrame(new FixedRelativeFrame("P",element));
}

ContourContextContextMenu::ContourContextContextMenu(Element *element_, QWidget *parent) : QMenu(parent), element(element_) {
  QAction *action = new QAction("Add point", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addPoint()));
  addAction(action);
  action = new QAction("Add line", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addLine()));
  addAction(action);
  action = new QAction("Add plane", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addPlane()));
  addAction(action);
  action = new QAction("Add sphere", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addSphere()));
  addAction(action);
}

void ContourContextContextMenu::addPoint() {
  mw->addContour(new Point("Point",element));
}

void ContourContextContextMenu::addLine() {
  mw->addContour(new Line("Line",element));
}

void ContourContextContextMenu::addPlane() {
  mw->addContour(new Plane("Plane",element));
}

void ContourContextContextMenu::addSphere() {
  mw->addContour(new Sphere("Sphere",element));
}

ObjectContextContextMenu::ObjectContextContextMenu(Element *element_, QWidget *parent) : QMenu(parent), element(element_) {
  QAction *action = new QAction("Add rigid body", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addRigidBody()));
  addAction(action);
  action = new QAction("Add kinematic constraint", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addKinematicConstraint()));
  addAction(action);
  action = new QAction("Add gear constraint", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addGearConstraint()));
  addAction(action);
  action = new QAction("Add joint constraint", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addJointConstraint()));
  addAction(action);
}

void ObjectContextContextMenu::addRigidBody() {
  mw->addObject(new RigidBody("RigidBody",element));
}

void ObjectContextContextMenu::addGearConstraint() {
  mw->addObject(new GearConstraint("GearConstraint",element));
}

void ObjectContextContextMenu::addKinematicConstraint() {
  mw->addObject(new KinematicConstraint("KinematicConstraint",element));
}

void ObjectContextContextMenu::addJointConstraint() {
  mw->addObject(new JointConstraint("JointConstraint",element));
}

LinkContextContextMenu::LinkContextContextMenu(Element *element_, QWidget *parent) : QMenu(parent), element(element_) {
  QAction *action = new QAction("Add kinetic excitation", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addKineticExcitation()));
  action = new QAction("Add spring damper", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addSpringDamper()));
  addAction(action);
  action = new QAction("Add joint", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addJoint()));
  addAction(action);
  action = new QAction("Add contact", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addContact()));
  addAction(action);
  action = new QAction("Add signal", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addSignal()));
  addAction(action);
}

void LinkContextContextMenu::addKineticExcitation() {
  mw->addLink(new KineticExcitation("KineticExcitation",element));
}

void LinkContextContextMenu::addSpringDamper() {
  mw->addLink(new SpringDamper("SpringDamper",element));
}

void LinkContextContextMenu::addJoint() {
  mw->addLink(new Joint("Joint",element));
}

void LinkContextContextMenu::addContact() {
  mw->addLink(new Contact("Contact",element));
}

void LinkContextContextMenu::addSignal() {
  SignalContextContextMenu menu(element);
  menu.exec(QCursor::pos());
}

void ObserverContextContextMenu::addAbsoluteKinematicsObserver() {
  mw->addObserver(new AbsoluteKinematicsObserver("AbsoluteKinematicsObserver",element));
}

ObserverContextContextMenu::ObserverContextContextMenu(Element *element_, QWidget *parent) : QMenu(parent), element(element_) {
  QAction *action = new QAction("Add absolute kinematics observer", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addAbsoluteKinematicsObserver()));
  addAction(action);
}

SignalContextContextMenu::SignalContextContextMenu(Element *element_, QWidget *parent) : QMenu(parent), element(element_) {
  QAction *action = new QAction("Add sensor", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addSensor()));
  addAction(action);
  action = new QAction("Add signal addition", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addSignalAddition()));
  addAction(action);
}

void SignalContextContextMenu::addSensor() {
  SensorContextContextMenu menu(element);
  menu.exec(QCursor::pos());
}

void SignalContextContextMenu::addSignalAddition() {
  mw->addLink(new SignalAddition("SignalAddition",element));
}

SensorContextContextMenu::SensorContextContextMenu(Element *element_, QWidget *parent) : QMenu(parent), element(element_) {
  QAction *action = new QAction("Add generalized position sensor", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addGeneralizedPositionSensor()));
  addAction(action);
  action = new QAction("Add absolute position sensor", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addAbsolutePositionSensor()));
  addAction(action);
  action = new QAction("Add function sensor", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addFunctionSensor()));
  addAction(action);
}

void SensorContextContextMenu::addGeneralizedPositionSensor() {
  mw->addLink(new GeneralizedPositionSensor("GeneralizedPositionSensor",element));
}

void SensorContextContextMenu::addAbsolutePositionSensor() {
  mw->addLink(new AbsolutePositionSensor("AbsolutePositionSensor",element));
}

void SensorContextContextMenu::addFunctionSensor() {
  mw->addLink(new FunctionSensor("FunctionSensor",element));
}
