/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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
#include "element_context_menu.h"
#include "mainwindow.h"
#include "rigid_body.h"
#include "flexible_ffr_body.h"
#include "constraint.h"
#include "kinetic_excitation.h"
#include "spring_damper.h"
#include "joint.h"
#include "friction.h"
#include "clutch.h"
#include "contact.h"
#include "observer.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "gear.h"
#include "connection.h"
#include "structure.h"
#include "sensor.h"
#include "physics.h"
#include "element_view.h"
#include "utils.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  ElementContextMenu::ElementContextMenu(Element *element, QWidget *parent, bool removable, bool saveable) : QMenu("",parent) {
    QAction *action=new QAction(QIcon::fromTheme("document-properties"), "Edit", this);
    connect(action,&QAction::triggered,this,[=](){ mw->openElementEditor(); });
    addAction(action);
    if(saveable) {
      action=new QAction(QIcon::fromTheme("document-properties"), "Edit XML", this);
      connect(action,&QAction::triggered,mw,&MainWindow::editElementSource);
      addAction(action);
      addSeparator();
      action=new QAction(QIcon::fromTheme("document-save-as"), "Export", this);
      connect(action,&QAction::triggered,mw,&MainWindow::exportElement);
      addAction(action);
    }
    if(removable) {
      addSeparator();
      action=new QAction(QIcon::fromTheme("edit-copy"), "Copy", this);
      connect(action,&QAction::triggered,this,[=](){ mw->copyElement(); });
      addAction(action);
      action=new QAction(QIcon::fromTheme("edit-cut"), "Cut", this);
      connect(action,&QAction::triggered,this,[=](){ mw->copyElement(true); });
      addAction(action);
      addSeparator();
      action=new QAction(QIcon::fromTheme("edit-delete"), "Remove", this);
      connect(action,&QAction::triggered,mw,&MainWindow::removeElement);
      addAction(action);
      addSeparator();
      action=new QAction(QIcon::fromTheme("edit-copy"), "Clone", this);
      connect(action,&QAction::triggered,this,[=](){ mw->openCloneEditor(); });
      addAction(action);
      addSeparator();
      action = new QAction("Enable", this);
      action->setCheckable(true);
      action->setChecked(element->getEnabled());
      action->setEnabled(element->getParent()->getEnabled());
      connect(action,&QAction::toggled,mw,&MainWindow::enableElement);
      addAction(action);
    }
  }

  DynamicSystemSolverContextMenu::DynamicSystemSolverContextMenu(Element *element, QWidget *parent) : ElementContextMenu(element,parent,false,true) {
    addSeparator();
    QAction *action = new QAction(QIcon::fromTheme("document-open"), "Load", this);
    connect(action,&QAction::triggered,this,[=](){ mw->loadDynamicSystemSolver(); });
    addAction(action);
  }

  FrameContextMenu::FrameContextMenu(Frame *frame, QWidget *parent, bool removable) : ElementContextMenu(frame,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setEnabled(frame->getParent()->getIndexOfFrame(frame)>1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveFrame(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setEnabled(frame->getParent()->getIndexOfFrame(frame)<frame->getParent()->getNumberOfFrames()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveFrame(false); });
    addAction(action);
  }

  ContourContextMenu::ContourContextMenu(Contour *contour, QWidget *parent, bool removable) : ElementContextMenu(contour,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setEnabled(contour->getParent()->getIndexOfContour(contour)>0);
    connect(action,&QAction::triggered,this,[=](){ mw->moveContour(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setEnabled(contour->getParent()->getIndexOfContour(contour)<contour->getParent()->getNumberOfContours()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveContour(false); });
    addAction(action);
  }

  GroupContextMenu::GroupContextMenu(Group *group, QWidget *parent, bool removable) : ElementContextMenu(group,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setEnabled(group->getParent()->getIndexOfGroup(group)>0);
    connect(action,&QAction::triggered,this,[=](){ mw->moveGroup(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setEnabled(group->getParent()->getIndexOfGroup(group)<group->getParent()->getNumberOfGroups()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveGroup(false); });
    addAction(action);
  }

  ObjectContextMenu::ObjectContextMenu(Object *object, QWidget *parent, bool removable) : ElementContextMenu(object,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setEnabled(object->getParent()->getIndexOfObject(object)>0);
    connect(action,&QAction::triggered,this,[=](){ mw->moveObject(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setEnabled(object->getParent()->getIndexOfObject(object)<object->getParent()->getNumberOfObjects()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveObject(false); });
    addAction(action);
  }

  LinkContextMenu::LinkContextMenu(Link *link, QWidget *parent, bool removable) : ElementContextMenu(link,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setEnabled(link->getParent()->getIndexOfLink(link)>0);
    connect(action,&QAction::triggered,this,[=](){ mw->moveLink(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setEnabled(link->getParent()->getIndexOfLink(link)<link->getParent()->getNumberOfLinks()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveLink(false); });
    addAction(action);
  }

  ConstraintContextMenu::ConstraintContextMenu(Constraint *constraint, QWidget *parent, bool removable) : ElementContextMenu(constraint,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setEnabled(constraint->getParent()->getIndexOfConstraint(constraint)>0);
    connect(action,&QAction::triggered,this,[=](){ mw->moveConstraint(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setEnabled(constraint->getParent()->getIndexOfConstraint(constraint)<constraint->getParent()->getNumberOfConstraints()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveConstraint(false); });
    addAction(action);
  }

  ObserverContextMenu::ObserverContextMenu(Observer *observer, QWidget *parent, bool removable) : ElementContextMenu(observer,parent,removable) {
    addSeparator();
    QAction *action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setEnabled(observer->getParent()->getIndexOfObserver(observer)>0);
    connect(action,&QAction::triggered,this,[=](){ mw->moveObserver(true); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setEnabled(observer->getParent()->getIndexOfObserver(observer)<observer->getParent()->getNumberOfObservers()-1);
    connect(action,&QAction::triggered,this,[=](){ mw->moveObserver(false); });
    addAction(action);
  }

  FramesContextMenu::FramesContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    QAction *action = new QAction(QIcon::fromTheme("document-open"), "Load", this);
    connect(action,&QAction::triggered,this,[=](){ mw->loadFrame(element); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setEnabled(dynamic_cast<Frame*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->pasteFrame(element,mw->getElementBuffer().first); });
    addAction(action);
  }

  FixedRelativeFramesContextMenu::FixedRelativeFramesContextMenu(Element *element, const QString &title, QWidget *parent) : FramesContextMenu(element,title,parent) {
    addSeparator();
    createContextMenuFor<FixedRelativeFrame>(this, element, "Add '");
  }

  NodeFramesContextMenu::NodeFramesContextMenu(Element *element, const QString &title, QWidget *parent) : FramesContextMenu(element,title,parent) {
    addSeparator();
    createContextMenuFor<NodeFrame>(this, element, "Add '");
  }

  ContoursContextMenu::ContoursContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    QAction *action = new QAction(QIcon::fromTheme("document-open"), "Load", this);
    connect(action,&QAction::triggered,this,[=](){ mw->loadContour(element); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setEnabled(dynamic_cast<Contour*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->pasteContour(element, mw->getElementBuffer().first); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Contour>(this, element, "Add '");
  }

  GroupsContextMenu::GroupsContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    QAction *action = new QAction(QIcon::fromTheme("document-open"), "Load", this);
    connect(action,&QAction::triggered,this,[=](){ mw->loadGroup(element); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setEnabled(dynamic_cast<Group*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->pasteGroup(element, mw->getElementBuffer().first); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Group>(this, element, "Add '");
  }

  ObjectsContextMenu::ObjectsContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    QAction *action = new QAction(QIcon::fromTheme("document-open"), "Load", this);
    connect(action,&QAction::triggered,this,[=](){ mw->loadObject(element); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setEnabled(dynamic_cast<Object*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->pasteObject(element, mw->getElementBuffer().first); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Object>(this, element, "Add '");
  }

  LinksContextMenu::LinksContextMenu(Element *element, const QString &title,  QWidget *parent) : QMenu(title,parent) {
    QAction *action = new QAction(QIcon::fromTheme("document-open"), "Load", this);
    connect(action,&QAction::triggered,this,[=](){ mw->loadLink(element); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setEnabled(dynamic_cast<Link*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->pasteLink(element, mw->getElementBuffer().first); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Link>(this, element, "Add '");
  }

  ConstraintsContextMenu::ConstraintsContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    QAction *action = new QAction(QIcon::fromTheme("document-open"), "Load", this);
    connect(action,&QAction::triggered,this,[=](){ mw->loadConstraint(element); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setEnabled(dynamic_cast<Constraint*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->pasteConstraint(element, mw->getElementBuffer().first); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Constraint>(this, element, "Add '");
  }

  ObserversContextMenu::ObserversContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    QAction *action = new QAction(QIcon::fromTheme("document-open"), "Load", this);
    connect(action,&QAction::triggered,this,[=](){ mw->loadObserver(element); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setEnabled(dynamic_cast<Observer*>(mw->getElementBuffer().first));
    connect(action,&QAction::triggered,this,[=](){ mw->pasteObserver(element, mw->getElementBuffer().first); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Observer>(this, element, "Add '");
  }

}
