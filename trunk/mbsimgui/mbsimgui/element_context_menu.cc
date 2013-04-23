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

extern MainWindow *mw;

ElementContextMenu::ElementContextMenu(QWidget *parent, bool removable) : QMenu(parent) {
  if(removable) {
    QAction *action=new QAction("Save as", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(saveElementAs()));
    addAction(action);
    action=new QAction("Remove", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(removeElement()));
    addAction(action);
  }
}

void ElementContextMenu::addContour() {
  QMenu menu("Context Menu");
  QAction *action = new QAction("Add point", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addPoint()));
  menu.addAction(action);
  action = new QAction("Add line", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addLine()));
  menu.addAction(action);
  action = new QAction("Add plane", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addPlane()));
  menu.addAction(action);
  action = new QAction("Add sphere", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addSphere()));
  menu.addAction(action);
  menu.exec(QCursor::pos());
}

GroupContextMenu::GroupContextMenu(QWidget *parent, bool removable) : ElementContextMenu(parent,removable) {
  QAction *action;
  action = new QAction("Add frame", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addFrame()));
  addAction(action);
  action = new QAction("Add contour", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addContour()));
  addAction(action);
  action = new QAction("Add group", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addGroup()));
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
} 

void GroupContextMenu::addObject() {
  QMenu menu("Context Menu");
  QAction *action = new QAction("Add rigid body", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addRigidBody()));
  menu.addAction(action);
  action = new QAction("Add kinematic constraint", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addKinematicConstraint()));
  menu.addAction(action);
  action = new QAction("Add gear constraint", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addGearConstraint()));
  menu.addAction(action);
  action = new QAction("Add joint constraint", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addJointConstraint()));
  menu.addAction(action);
  menu.exec(QCursor::pos());
}

void GroupContextMenu::addLink() {
  QMenu menu("Context Menu");
  QAction *action = new QAction("Add kinetic excitation", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addKineticExcitation()));
  action = new QAction("Add spring damper", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addSpringDamper()));
  menu.addAction(action);
  action = new QAction("Add joint", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addJoint()));
  menu.addAction(action);
  action = new QAction("Add contact", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addContact()));
  menu.addAction(action);
  menu.exec(QCursor::pos());
}

void GroupContextMenu::addObserver() {
  QMenu menu("Context Menu");
  QAction *action = new QAction("Add absolute kinematics observer", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addAbsoluteKinematicsObserver()));
  menu.addAction(action);
  menu.exec(QCursor::pos());
}

ObjectContextMenu::ObjectContextMenu(QWidget *parent) : ElementContextMenu(parent,true) {
} 

BodyContextMenu::BodyContextMenu(QWidget *parent) : ObjectContextMenu(parent) {
  QAction *action;
  action = new QAction("Add frame", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(addFrame()));
  addAction(action);
  action = new QAction("Add contour", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addContour()));
  addAction(action);
} 
