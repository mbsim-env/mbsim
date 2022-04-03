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
    QAction *action = new QAction("Add fixed relative frame", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addFrame(new FixedRelativeFrame, element); });
    addAction(action);
    action = new QAction("Add unknown frame", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addFrame(new UnknownFrame, element); });
    addAction(action);
  }

  NodeFramesContextMenu::NodeFramesContextMenu(Element *element, const QString &title, QWidget *parent) : FramesContextMenu(element,title,parent) {
    addSeparator();
    QAction *action = new QAction("Add node frame", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addFrame(new NodeFrame, element); });
    addAction(action);
    action = new QAction("Add interface node frame", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addFrame(new InterfaceNodeFrame, element); });
    addAction(action);
    action = new QAction("Add ffr interface node frame", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addFrame(new FfrInterfaceNodeFrame, element); });
    addAction(action);
    action = new QAction("Add distributing ffr interface node frame", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addFrame(new DistributingFfrInterfaceNodeFrame, element); });
    addAction(action);
    action = new QAction("Add unknown frame", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addFrame(new UnknownFrame, element); });
    addAction(action);
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
    action = new QAction("Add bevel gear", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new BevelGear, element); });
    addAction(action);
    action = new QAction("Add circle", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new Circle, element); });
    addAction(action);
    action = new QAction("Add cuboid", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new Cuboid, element); });
    addAction(action);
    action = new QAction("Add cylindrical gear", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new CylindricalGear, element); });
    addAction(action);
    action = new QAction("Add disk", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new Disk, element); });
    addAction(action);
    QMenu *menu = new QMenu("Add fcl contour", this);
    action = new QAction("Add fcl box", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new FclBox, element); });
    menu->addAction(action);
    action = new QAction("Add fcl mesh", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new FclMesh, element); });
    menu->addAction(action);
    action = new QAction("Add fcl plane", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new FclPlane, element); });
    menu->addAction(action);
    action = new QAction("Add fcl sphere", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new FclSphere, element); });
    menu->addAction(action);
    addMenu(menu);
    menu = new QMenu("Add flexible contour", this);
    action = new QAction("Add flexible planar ffr nurbs contour", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new FlexiblePlanarFfrNurbsContour, element); });
    menu->addAction(action);
    action = new QAction("Add flexible planar nurbs contour", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new FlexiblePlanarNurbsContour, element); });
    menu->addAction(action);
    action = new QAction("Add flexible spatial ffr nurbs contour", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new FlexibleSpatialFfrNurbsContour, element); });
    menu->addAction(action);
    action = new QAction("Add flexible spatial nurbs contour", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new FlexibleSpatialNurbsContour, element); });
    menu->addAction(action);
    addMenu(menu);
    action = new QAction("Add line", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new Line, element); });
    addAction(action);
    action = new QAction("Add line segment", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new LineSegment, element); });
    addAction(action);
    action = new QAction("Add planar contour", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new PlanarContour, element); });
    addAction(action);
    action = new QAction("Add planar gear", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new PlanarGear, element); });
    addAction(action);
    action = new QAction("Add planar nurbs contour", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new PlanarNurbsContour, element); });
    addAction(action);
    action = new QAction("Add plane", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new Plane, element); });
    addAction(action);
    action = new QAction("Add point", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new Point, element); });
    addAction(action);
    action = new QAction("Add rack", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new Rack, element); });
    addAction(action);
    action = new QAction("Add sphere", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new Sphere, element); });
    addAction(action);
    action = new QAction("Add spatial contour", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new SpatialContour, element); });
    addAction(action);
    action = new QAction("Add spatial nurbs contour", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new SpatialNurbsContour, element); });
    addAction(action);
    action = new QAction("Add unknown contour", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addContour(new UnknownContour, element); });
    addAction(action);
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
    action = new QAction("Add group", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addGroup(new Group, element); });
    addAction(action);
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
    QMenu *menu = new QMenu("Add body", this);
    action = new QAction("Add calculix body", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addObject(new CalculixBody, element); });
    menu->addAction(action);
    action = new QAction("Add external finite elements ffr body", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addObject(new ExternalFiniteElementsFfrBody, element); });
    menu->addAction(action);
    action = new QAction("Add finite elements ffr body", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addObject(new FiniteElementsFfrBody, element); });
    menu->addAction(action);
    action = new QAction("Add flexible ffr beam", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addObject(new FlexibleFfrBeam, element); });
    menu->addAction(action);
    action = new QAction("Add flexible ffr body", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addObject(new FlexibleFfrBody, element); });
    menu->addAction(action);
    action = new QAction("Add rigid body", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addObject(new RigidBody, element); });
    menu->addAction(action);
    addMenu(menu);
    action = new QAction("Add unknown object", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addObject(new UnknownObject, element); });
    addAction(action);
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
    action = new QAction("Add aerodynamics", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new Aerodynamics, element); });
    addAction(action);
    action = new QAction("Add buoyancy", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new Buoyancy, element); });
    addAction(action);
    action = new QAction("Add contact", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new Contact, element);  });
    addAction(action);
    action = new QAction("Add directional spring damper", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new DirectionalSpringDamper, element); });
    addAction(action);
    action = new QAction("Add disk contact", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new DiskContact, element); });
    addAction(action);
    action = new QAction("Add drag", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new Drag, element); });
    addAction(action);
    action = new QAction("Add elastic joint", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new ElasticJoint, element); });
    addAction(action);
    action = new QAction("Add generalized clutch", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedClutch, element); });
    addAction(action);
    action = new QAction("Add generalized elastic connection", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedElasticConnection, element); });
    addAction(action);
    action = new QAction("Add generalized elastic structure", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedElasticStructure, element); });
    addAction(action);
    action = new QAction("Add generalized friction", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedFriction, element); });
    addAction(action);
    action = new QAction("Add generalized gear", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedGear, element); });
    addAction(action);
    action = new QAction("Add isotropic rotational spring damper", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new IsotropicRotationalSpringDamper, element); });
    addAction(action);
    action = new QAction("Add generalized spring damper", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedSpringDamper, element); });
    addAction(action);
    action = new QAction("Add joint", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new Joint, element); });
    addAction(action);
    action = new QAction("Add kinetic excitation", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new KineticExcitation, element); });
    addAction(action);
    QMenu *menu = new SignalsContextMenu(element, "Add signal");
    addMenu(menu);
    action = new QAction("Add spring damper", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new SpringDamper, element); });
    addAction(action);
    action = new QAction("Add universal gravitation", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new UniversalGravitation, element); });
    addAction(action);
    action = new QAction("Add unknown link", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new UnknownLink, element); });
    addAction(action);
    action = new QAction("Add weight", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new Weight, element); });
    addAction(action);
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
    action = new QAction("Add generalized acceleration constraint", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addConstraint(new GeneralizedAccelerationConstraint, element); });
    addAction(action);
    action = new QAction("Add generalized connection constraint", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addConstraint(new GeneralizedConnectionConstraint, element); });
    addAction(action);
    action = new QAction("Add generalized gear constraint", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addConstraint(new GeneralizedGearConstraint, element); });
    addAction(action);
    action = new QAction("Add generalized position constraint", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addConstraint(new GeneralizedPositionConstraint, element); });
    addAction(action);
    action = new QAction("Add generalized velocity constraint", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addConstraint(new GeneralizedVelocityConstraint, element); });
    addAction(action);
    action = new QAction("Add inverse kinematics constraint", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addConstraint(new InverseKinematicsConstraint, element); });
    addAction(action);
    action = new QAction("Add joint constraint", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addConstraint(new JointConstraint, element); });
    addAction(action);
    action = new QAction("Add unknown constraint", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addConstraint(new UnknownConstraint, element); });
    addAction(action);
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
    action = new QAction("Add contact observer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addObserver(new ContactObserver, element); });
    addAction(action);
    action = new QAction("Add frame observer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addObserver(new FrameObserver, element); });
    addAction(action);
    action = new QAction("Add inverse kinematics constraint observer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addObserver(new InverseKinematicsConstraintObserver, element); });
    addAction(action);
    action = new QAction("Add mechanical constraint observer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addObserver(new MechanicalConstraintObserver, element); });
    addAction(action);
    action = new QAction("Add mechanical link observer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addObserver(new MechanicalLinkObserver, element); });
    addAction(action);
    action = new QAction("Add rigid body observer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addObserver(new RigidBodyObserver, element); });
    addAction(action);
    action = new QAction("Add signal observer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addObserver(new SignalObserver, element); });
    addAction(action);
    action = new QAction("Add unknown observer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addObserver(new UnknownObserver, element); });
    addAction(action);
  }

  SignalsContextMenu::SignalsContextMenu(Element *element, const QString &title, QWidget *parent) : QMenu(title,parent) {
    QAction *action = new QAction("Add demultiplexer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new Demultiplexer, element); });
    addAction(action);
    action = new QAction("Add duration", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new Duration, element); });
    addAction(action);
    action = new QAction("Add extern signal sink", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new ExternSignalSink, element); });
    addAction(action);
    action = new QAction("Add extern signal source", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new ExternSignalSource, element); });
    addAction(action);
    action = new QAction("Add linear transfer system", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new LinearTransferSystem, element); });
    addAction(action);
    action = new QAction("Add multiplexer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new Multiplexer, element); });
    addAction(action);
    action = new QAction("Add nonlinear transfer system", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new NonlinearTransferSystem, element); });
    addAction(action);
    QMenu *menu = new QMenu("Add sensor", this);
    action = new QAction("Add acceleration sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new AccelerationSensor, element); });
    menu->addAction(action);
    action = new QAction("Add angular acceleration sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new AngularAccelerationSensor, element); });
    menu->addAction(action);
    action = new QAction("Add angular velocity sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new AngularVelocitySensor, element); });
    menu->addAction(action);
    action = new QAction("Add function sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new FunctionSensor, element); });
    menu->addAction(action);
    action = new QAction("Add generalized acceleration sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedAccelerationSensor, element); });
    menu->addAction(action);
    action = new QAction("Add generalized contact force sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedContactForceSensor, element); });
    menu->addAction(action);
    action = new QAction("Add generalized force sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedForceSensor, element); });
    menu->addAction(action);
    action = new QAction("Add generalized position sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedPositionSensor, element); });
    menu->addAction(action);
    action = new QAction("Add generalized relative contact position sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedRelativeContactPositionSensor, element); });
    menu->addAction(action);
    action = new QAction("Add generalized relative contact velocity sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedRelativeContactVelocitySensor, element); });
    menu->addAction(action);
    action = new QAction("Add generalized relative position sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedRelativePositionSensor, element); });
    menu->addAction(action);
    action = new QAction("Add generalized relative velocity sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedRelativeVelocitySensor, element); });
    menu->addAction(action);
    action = new QAction("Add generalized velocity sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new GeneralizedVelocitySensor, element); });
    menu->addAction(action);
    action = new QAction("Add mechanical constraint force sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new MechanicalConstraintForceSensor, element); });
    menu->addAction(action);
    action = new QAction("Add mechanical constraint moment sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new MechanicalConstraintMomentSensor, element); });
    menu->addAction(action);
    action = new QAction("Add mechanical link force sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new MechanicalLinkForceSensor, element); });
    menu->addAction(action);
    action = new QAction("Add mechanical link moment sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new MechanicalLinkMomentSensor, element); });
    menu->addAction(action);
    action = new QAction("Add rigid body joint force sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new RigidBodyJointForceSensor, element); });
    menu->addAction(action);
    action = new QAction("Add rigid body joint moment sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new RigidBodyJointMomentSensor, element); });
    menu->addAction(action);
    action = new QAction("Add orientation sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new OrientationSensor, element); });
    menu->addAction(action);
    action = new QAction("Add position sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new PositionSensor, element); });
    menu->addAction(action);
    action = new QAction("Add velocity sensor", menu);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new VelocitySensor, element); });
    menu->addAction(action);
    addMenu(menu);
    action = new QAction("Add signal operation", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new SignalOperation, element); });
    addAction(action);
    action = new QAction("Add state machine", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new StateMachine, element); });
    addAction(action);
    action = new QAction("Add switch", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addLink(new Switch, element); });
    addAction(action);
  }

}
