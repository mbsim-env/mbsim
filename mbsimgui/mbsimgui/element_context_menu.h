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

class Element;

class ElementContextMenu : public QMenu {
  Q_OBJECT

  public:
    ElementContextMenu(Element *element, QWidget * parent = 0, bool removable=true);

  protected slots:
    void addContour();

  protected:
    Element *element;
};

class GroupContextMenu : public ElementContextMenu {
  Q_OBJECT

  public:
    GroupContextMenu(Element *group, QWidget * parent = 0, bool removable=true);

  protected slots:
    void addFixedRelativeFrame();
    void addGroup();
    void addObject();
    void addExtraDynamic();
    void addLink();
    void addObserver();
    void addElementFromFile();
};

class ContourContextContextMenu : public QMenu {
  Q_OBJECT

  public:
    ContourContextContextMenu(Element *contour, QWidget * parent = 0);

  protected slots:
    void addPoint();
    void addLine();
    void addPlane();
    void addSphere();

  protected:
    Element *element;
};

class ObjectContextContextMenu : public QMenu {
  Q_OBJECT

  public:
    ObjectContextContextMenu(Element *object, QWidget * parent = 0);

  protected slots:
    void addRigidBody();
    void addGearConstraint();
    void addKinematicConstraint();
    void addJointConstraint();

  protected:
    Element *element;
};

class ExtraDynamicContextContextMenu : public QMenu {
  Q_OBJECT

  public:
    ExtraDynamicContextContextMenu(Element *ed, QWidget * parent = 0);

  protected slots:
    void addLinearTransferSystem();

  protected:
    Element *element;
};


class LinkContextContextMenu : public QMenu {
  Q_OBJECT

  public:
    LinkContextContextMenu(Element *link, QWidget * parent = 0);

  protected slots:
    void addSpringDamper();
    void addKineticExcitation();
    void addJoint();
    void addContact();
    void addActuator();
    void addSignal();

  protected:
    Element *element;
};

class ObserverContextContextMenu : public QMenu {
  Q_OBJECT

  public:
    ObserverContextContextMenu(Element *observer, QWidget * parent = 0);

  protected slots:
    void addAbsoluteKinematicsObserver();

  protected:
    Element *element;
};

class SignalContextContextMenu : public QMenu {
  Q_OBJECT

  public:
    SignalContextContextMenu(Element *signal, QWidget * parent = 0);

  protected slots:
    void addSensor();
    void addSignalAddition();

  protected:
    Element *element;
};

class SensorContextContextMenu : public QMenu {
  Q_OBJECT

  public:
    SensorContextContextMenu(Element *sensor, QWidget * parent = 0);

  protected slots:
    void addGeneralizedPositionSensor();
    void addAbsolutePositionSensor();
    void addFunctionSensor();
    void addSignalProcessingSystemSensor();

  protected:
    Element *element;
};

class SolverContextMenu : public GroupContextMenu {

  public:
    SolverContextMenu(Element *solver, QWidget * parent = 0);
};

class FrameContextMenu : public ElementContextMenu {

  public:
    FrameContextMenu(Element *frame, QWidget * parent = 0, bool removable=false);
};

class FixedRelativeFrameContextMenu : public FrameContextMenu {

  public:
    FixedRelativeFrameContextMenu(Element *frame, QWidget * parent = 0); 
};

class ObjectContextMenu : public ElementContextMenu {

  public:
    ObjectContextMenu(Element *object, QWidget * parent = 0);
};

class BodyContextMenu : public ObjectContextMenu {
  Q_OBJECT

  public:
    BodyContextMenu(Element *body, QWidget * parent = 0);

  protected slots:
    void addFixedRelativeFrame();
};


#endif
