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
#include <QFileDialog>

extern MainWindow *mw;

ElementContextMenu::ElementContextMenu(QWidget *parent, bool removable) : QMenu(parent) {
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
  ContourContextContextMenu menu;
  menu.exec(QCursor::pos());
}

GroupContextMenu::GroupContextMenu(QWidget *parent, bool removable) : ElementContextMenu(parent,removable) {
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
  mw->addFrame<FixedRelativeFrame>("P");
}


void GroupContextMenu::addElementFromFile() {
  QString file=QFileDialog::getOpenFileName(0, "XML model files", ".", "XML files (*.xml)");
  if(file!="") {
    if(mw->addFrame<Frame>(file.toStdString(),true))
      return;
    if(mw->addContour<Contour>(file.toStdString(),true))
      return;
    if(mw->addGroup<Group>(file.toStdString(),true))
      return;
    if(mw->addObject<Object>(file.toStdString(),true))
      return;
    if(mw->addLink<Link>(file.toStdString(),true))
      return;
    if(mw->addObserver<Observer>(file.toStdString(),true))
      return;
  }
}

void GroupContextMenu::addGroup() {
  mw->addGroup<Group>("Group");
}

void GroupContextMenu::addObject() {
  ObjectContextContextMenu menu;
  menu.exec(QCursor::pos());
}

void GroupContextMenu::addLink() {
  LinkContextContextMenu menu;
  menu.exec(QCursor::pos());
}

void GroupContextMenu::addObserver() {
  ObserverContextContextMenu menu;
  menu.exec(QCursor::pos());
}

ObjectContextMenu::ObjectContextMenu(QWidget *parent) : ElementContextMenu(parent,true) {
} 

BodyContextMenu::BodyContextMenu(QWidget *parent) : ObjectContextMenu(parent) {
  QAction *action;
  action = new QAction("Add frame", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addFixedRelativeFrame()));
  addAction(action);
  action = new QAction("Add contour", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addContour()));
  addAction(action);
} 

void BodyContextMenu::addFixedRelativeFrame() {
  mw->addFrame<FixedRelativeFrame>("P");
}

ContourContextContextMenu::ContourContextContextMenu(QWidget *parent) : QMenu(parent) {
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
  mw->addContour<Point>("Point");
}

void ContourContextContextMenu::addLine() {
  mw->addContour<Line>("Line");
}

void ContourContextContextMenu::addPlane() {
  mw->addContour<Plane>("Plane");
}

void ContourContextContextMenu::addSphere() {
  mw->addContour<Sphere>("Sphere");
}

ObjectContextContextMenu::ObjectContextContextMenu(QWidget *parent) : QMenu(parent) {
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
  mw->addObject<RigidBody>("RigidBody");
}

void ObjectContextContextMenu::addGearConstraint() {
  mw->addObject<GearConstraint>("GearConstraint");
}

void ObjectContextContextMenu::addKinematicConstraint() {
  mw->addObject<KinematicConstraint>("KinematicConstraint");
}

void ObjectContextContextMenu::addJointConstraint() {
  mw->addObject<JointConstraint>("JointConstraint");
}

LinkContextContextMenu::LinkContextContextMenu(QWidget *parent) : QMenu(parent) {
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
  action = new QAction("Add sensor", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addSensor()));
  addAction(action);
}

void LinkContextContextMenu::addKineticExcitation() {
  mw->addLink<KineticExcitation>("KineticExcitation");
}

void LinkContextContextMenu::addSpringDamper() {
  mw->addLink<SpringDamper>("SpringDamper");
}

void LinkContextContextMenu::addJoint() {
  mw->addLink<Joint>("Joint");
}

void LinkContextContextMenu::addContact() {
  mw->addLink<Contact>("Contact");
}

void LinkContextContextMenu::addSensor() {
  SensorContextContextMenu menu;
  menu.exec(QCursor::pos());
}

ObserverContextContextMenu::ObserverContextContextMenu(QWidget *parent) : QMenu(parent) {
  QAction *action = new QAction("Add absolute kinematics observer", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addAbsoluteKinematicsObserver()));
  addAction(action);
}

void ObserverContextContextMenu::addAbsoluteKinematicsObserver() {
  mw->addObserver<AbsoluteKinematicsObserver>("AbsoluteKinematicsObserver");
}

SensorContextContextMenu::SensorContextContextMenu(QWidget *parent) : QMenu(parent) {
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
  mw->addLink<GeneralizedPositionSensor>("GeneralizedPositionSensor");
}

void SensorContextContextMenu::addAbsolutePositionSensor() {
  mw->addLink<AbsolutePositionSensor>("AbsolutePositionSensor");
}

void SensorContextContextMenu::addFunctionSensor() {
  mw->addLink<FunctionSensor>("FunctionSensor");
}


