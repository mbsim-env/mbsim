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

#ifndef _ELEMENT_CONTEXT_MENU_H_
#define _ELEMENT_CONTEXT_MENU_H_

#include <QMenu>

namespace MBSimGUI {

  class Element;
  class Frame;
  class Contour;
  class Group;
  class Object;
  class Link;
  class Constraint;
  class Observer;

  class BasicElementMenu : public QMenu {
    public:
      BasicElementMenu(Element *element_, const QString &title="", QWidget *parent=nullptr);
    protected:
      Element *element;
      void addAction(QAction *action);
  };

  class ElementContextMenu : public BasicElementMenu {
    public:
      ElementContextMenu(Element *element, QWidget *parent=nullptr, bool removable=true, bool saveable=true);
   };

  class FrameContextMenu : public ElementContextMenu {
    public:
      FrameContextMenu(Frame *frame, QWidget *parent=nullptr, bool removable=true);
  };

  class ContourContextMenu : public ElementContextMenu {
    public:
      ContourContextMenu(Contour *contour, QWidget *parent=nullptr, bool removable=true);
  };

  class GroupContextMenu : public ElementContextMenu {
    public:
      GroupContextMenu(Group *group, QWidget *parent=nullptr, bool removable=true);
  };

  class ObjectContextMenu : public ElementContextMenu {
    public:
      ObjectContextMenu(Object *object, QWidget *parent=nullptr, bool removable=true);
  };

  class LinkContextMenu : public ElementContextMenu {
    public:
      LinkContextMenu(Link *link, QWidget *parent=nullptr, bool removable=true);
  };

  class ConstraintContextMenu : public ElementContextMenu {
    public:
      ConstraintContextMenu(Constraint *constraint, QWidget *parent=nullptr, bool removable=true);
  };

  class ObserverContextMenu : public ElementContextMenu {
    public:
      ObserverContextMenu(Observer *observer, QWidget *parent=nullptr, bool removable=true);
  };

  class FramesContextMenu : public BasicElementMenu {
    Q_OBJECT

    public:
      FramesContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);

    protected slots:
      void paste();
      void load();
      void embed();
  };

  class FixedRelativeFramesContextMenu : public FramesContextMenu {
    Q_OBJECT

    public:
      FixedRelativeFramesContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);

    protected slots:
      void addFixedRelativeFrame();
      void addUnknownFrame();
  };

  class NodeFramesContextMenu : public FramesContextMenu {
    Q_OBJECT

    public:
      NodeFramesContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);

    protected slots:
      void addNodeFrame();
      void addInterfaceNodeFrame();
      void addFfrInterfaceNodeFrame();
      void addUnknownFrame();
  };

  class ContoursContextMenu : public BasicElementMenu {
    Q_OBJECT

    public:
      ContoursContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);

    protected slots:
      void addPoint();
      void addLine();
      void addPlane();
      void addSphere();
      void addCircle();
      void addCuboid();
      void addLineSegment();
      void addPlanarContour();
      void addPlanarNurbsContour();
      void addSpatialContour();
      void addSpatialNurbsContour();
      void addDisk();
      void addCylindricalGear();
      void addRack();
      void addBevelGear();
      void addPlanarGear();
      void addFlexiblePlanarNurbsContour();
      void addFlexiblePlanarFfrNurbsContour();
      void addFlexibleSpatialNurbsContour();
      void addFlexibleSpatialFfrNurbsContour();
      void addFclBox();
      void addFclSphere();
      void addFclPlane();
      void addFclMesh();
      void addUnknownContour();
      void paste();
      void load();
      void embed();
  };

  class GroupsContextMenu : public BasicElementMenu {
    Q_OBJECT

    public:
      GroupsContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);

    protected slots:
      void add();
      void paste();
      void load();
      void embed();
  };

  class ObjectsContextMenu : public BasicElementMenu {
    Q_OBJECT

    public:
      ObjectsContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);

    protected slots:
      void paste();
      void load();
      void embed();
      void addRigidBody();
      void addFlexibleFfrBody();
      void addCalculixBody();
      void addUnknownObject();
  };

  class LinksContextMenu : public BasicElementMenu {
    Q_OBJECT

    public:
      LinksContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);

    protected slots:
      void addSpringDamper();
      void addDirectionalSpringDamper();
      void addIsotropicRotationalSpringDamper();
      void addGeneralizedSpringDamper();
      void addKineticExcitation();
      void addJoint();
      void addElasticJoint();
      void addContact();
      void addDiskContact();
      void addGeneralizedFriction();
      void addGeneralizedGear();
      void addGeneralizedElasticConnection();
      void addGeneralizedElasticStructure();
      void addUniversalGravitation();
      void addWeight();
      void addBuoyancy();
      void addDrag();
      void addAerodynamics();
      void addUnknownLink();
      void paste();
      void load();
      void embed();
  };

  class ConstraintsContextMenu : public BasicElementMenu {
    Q_OBJECT

    public:
      ConstraintsContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);

    protected slots:
      void addGeneralizedGearConstraint();
      void addGeneralizedPositionConstraint();
      void addGeneralizedVelocityConstraint();
      void addGeneralizedAccelerationConstraint();
      void addJointConstraint();
      void addGeneralizedConnectionConstraint();
      void addInverseKinematicsConstraint();
      void addUnknownConstraint();
      void paste();
      void load();
      void embed();
  };


  class ObserversContextMenu : public BasicElementMenu {
    Q_OBJECT

    public:
      ObserversContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);

    protected slots:
      void addMechanicalLinkObserver();
      void addMechanicalConstraintObserver();
      void addContactObserver();
      void addFrameObserver();
      void addRigidBodyObserver();
      void addInverseKinematicsConstraintObserver();
      void addSignalObserver();
      void addUnknownObserver();
      void paste();
      void load();
      void embed();
  };

  class SignalsContextMenu : public BasicElementMenu {
    Q_OBJECT

    public:
      SignalsContextMenu(Element *element, const QString &title="", QWidget *parent=nullptr);

    protected slots:
      void addMultiplexer();
      void addDemultiplexer();
      void addLinearTransferSystem();
      void addSignalOperation();
      void addExternSignalSource();
      void addExternSignalSink();
      void addGeneralizedPositionSensor();
      void addGeneralizedVelocitySensor();
      void addGeneralizedAccelerationSensor();
      void addPositionSensor();
      void addOrientationSensor();
      void addVelocitySensor();
      void addAngularVelocitySensor();
      void addAccelerationSensor();
      void addAngularAccelerationSensor();
      void addFunctionSensor();
      void addGeneralizedRelativePositionSensor();
      void addGeneralizedRelativeVelocitySensor();
      void addGeneralizedForceSensor();
      void addMechanicalLinkForceSensor();
      void addMechanicalLinkMomentSensor();
      void addMechanicalConstraintForceSensor();
      void addMechanicalConstraintMomentSensor();
      void addGeneralizedRelativeContactPositionSensor();
      void addGeneralizedRelativeContactVelocitySensor();
      void addGeneralizedContactForceSensor();
      void addRigidBodyJointForceSensor();
      void addRigidBodyJointMomentSensor();
  };

}

#endif
