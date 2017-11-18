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

  class ElementContextMenu : public QMenu {
    public:
      ElementContextMenu(Element *element, QWidget * parent = nullptr, bool removable=true);

    protected:
      Element *element;
  };

  class FrameContextMenu : public ElementContextMenu {
    public:
      FrameContextMenu(Frame *frame, QWidget * parent = nullptr, bool removable=true);
  };

  class ContourContextMenu : public ElementContextMenu {
    public:
      ContourContextMenu(Contour *contour, QWidget * parent = nullptr, bool removable=true);
  };

  class GroupContextMenu : public ElementContextMenu {
    public:
      GroupContextMenu(Group *group, QWidget * parent = nullptr, bool removable=true);
  };

  class ObjectContextMenu : public ElementContextMenu {
    public:
      ObjectContextMenu(Object *object, QWidget * parent = nullptr, bool removable=true);
  };

  class LinkContextMenu : public ElementContextMenu {
    public:
      LinkContextMenu(Link *link, QWidget * parent = nullptr, bool removable=true);
  };

  class ConstraintContextMenu : public ElementContextMenu {
    public:
      ConstraintContextMenu(Constraint *constraint, QWidget * parent = nullptr, bool removable=true);
  };

  class ObserverContextMenu : public ElementContextMenu {
    public:
      ObserverContextMenu(Observer *observer, QWidget * parent = nullptr, bool removable=true);
  };

  class FramesContextMenu : public QMenu {
    Q_OBJECT

    public:
      FramesContextMenu(Element *frame, const QString &title="", QWidget * parent = nullptr);

    protected slots:
      void paste();
      void load();

    protected:
      Element *element;
  };

  class FixedRelativeFramesContextMenu : public FramesContextMenu {
    Q_OBJECT

    public:
      FixedRelativeFramesContextMenu(Element *contour, const QString &title="", QWidget * parent = nullptr);

    protected slots:
      void addFixedRelativeFrame();
  };

  class NodeFramesContextMenu : public FramesContextMenu {
    Q_OBJECT

    public:
      NodeFramesContextMenu(Element *contour, const QString &title="", QWidget * parent = nullptr);

    protected slots:
      void addNodeFrame();
  };

  class ContoursContextMenu : public QMenu {
    Q_OBJECT

    public:
      ContoursContextMenu(Element *contour, const QString &title="", QWidget * parent = nullptr);

    protected slots:
      void addPoint();
      void addLine();
      void addPlane();
      void addSphere();
      void addCircle();
      void addCuboid();
      void addLineSegment();
      void addPlanarContour();
      void addSpatialContour();
      void paste();
      void load();

    protected:
      Element *element;
  };

  class GroupsContextMenu : public QMenu {
    Q_OBJECT

    public:
      GroupsContextMenu(Element *contour, const QString &title="", QWidget * parent = nullptr);

    protected slots:
      void add();
      void paste();
      void load();

    protected:
      Element *element;
  };

  class ObjectsContextMenu : public QMenu {
    Q_OBJECT

    public:
      ObjectsContextMenu(Element *object, const QString &title="", QWidget * parent = nullptr);

    protected slots:
      void paste();
      void load();

    protected:
      Element *element;
  };

  class BodiesContextMenu : public QMenu {
    Q_OBJECT

    public:
      BodiesContextMenu(Element *object, const QString &title="", QWidget * parent = nullptr);

    protected slots:
      void addRigidBody();
      void addFlexibleBodyFFR();

    protected:
      Element *element;
  };

  class LinksContextMenu : public QMenu {
    Q_OBJECT

    public:
      LinksContextMenu(Element *link, const QString &title="", QWidget * parent = nullptr);

    protected slots:
      void addSpringDamper();
      void addDirectionalSpringDamper();
      void addGeneralizedSpringDamper();
      void addKineticExcitation();
      void addJoint();
      void addElasticJoint();
      void addContact();
      void addSignal();
      void addGeneralizedFriction();
      void addGeneralizedGear();
      void addGeneralizedElasticConnection();
      void paste();
      void load();

    protected:
      Element *element;
  };

  class ConstraintsContextMenu : public QMenu {
    Q_OBJECT

    public:
      ConstraintsContextMenu(Element *object, const QString &title="", QWidget * parent = nullptr);

    protected slots:
      void addGeneralizedGearConstraint();
      void addGeneralizedPositionConstraint();
      void addGeneralizedVelocityConstraint();
      void addGeneralizedAccelerationConstraint();
      void addJointConstraint();
      void addGeneralizedConnectionConstraint();
      void paste();
      void load();

    protected:
      Element *element;
  };


  class ObserversContextMenu : public QMenu {
    Q_OBJECT

    public:
      ObserversContextMenu(Element *observer, const QString &title="", QWidget * parent = nullptr);

    protected slots:
      void addMechanicalLinkObserver();
      void addMechanicalConstraintObserver();
      void addContactObserver();
      void addFrameObserver();
      void addRigidBodyObserver();
      void addKinematicCoordinatesObserver();
      void addRelativeKinematicsObserver();
      void addRigidBodySystemObserver();
      void paste();
      void load();

    protected:
      Element *element;
  };

  class SignalsContextMenu : public QMenu {
    Q_OBJECT

    public:
      SignalsContextMenu(Element *signal, const QString &title="", QWidget * parent = nullptr);

    protected slots:
      void addSensor();
      void addMultiplexer();
      void addDemultiplexer();
      void addLinearTransferSystem();
      void addSignalOperation();
      void addExternSignalSource();
      void addExternSignalSink();

    protected:
      Element *element;
  };

  class SensorsContextMenu : public QMenu {
    Q_OBJECT

    public:
      SensorsContextMenu(Element *sensor, const QString &title="", QWidget * parent = nullptr);

    protected slots:
      void addGeneralizedPositionSensor();
      void addGeneralizedVelocitySensor();
      void addPositionSensor();
      void addOrientationSensor();
      void addVelocitySensor();
      void addAngularVelocitySensor();
      void addFunctionSensor();

    protected:
      Element *element;
  };

}

#endif
