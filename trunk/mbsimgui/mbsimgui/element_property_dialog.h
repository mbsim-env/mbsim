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

#ifndef _ELEMENT_PROPERTY_DIALOG_H_
#define _ELEMENT_PROPERTY_DIALOG_H_

#include "property_dialog.h"

class Element;
class Frame;
class FixedRelativeFrame;
class Contour;
class Plane;
class Sphere;
class Solver;
class Group;
class Object;
class Body;
class RigidBody;
class Constraint;
class GearConstraint;
class KinematicConstraint;
class JointConstraint;
class Link;
class KineticExcitation;
class SpringDamper;
class Joint;
class Contact;
class Observer;
class AbsoluteKinematicsObserver;
class TextWidget;
class VecWidget;
class ExtWidget;

class ElementPropertyDialog : public PropertyDialog {

  public:
    ElementPropertyDialog(QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual void toWidget(Element *element);
    virtual void fromWidget(Element *element);
  protected:
    ExtWidget *name, *href, *count, *counterName, *parameterList;
};

class FramePropertyDialog : public ElementPropertyDialog {

  public:
    FramePropertyDialog(Frame *frame, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *visu;
};

class FixedRelativeFramePropertyDialog : public FramePropertyDialog {

  public:
    FixedRelativeFramePropertyDialog(FixedRelativeFrame *frame, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *refFrame, *position, *orientation;
};

class ContourPropertyDialog : public ElementPropertyDialog {

  public:
    ContourPropertyDialog(Contour *contour, QWidget * parent = 0, Qt::WindowFlags f = 0) : ElementPropertyDialog(parent,f) {}
};

class PlanePropertyDialog : public ContourPropertyDialog {

  public:
    PlanePropertyDialog(Plane *plane, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *radius, *visu;
};

class SpherePropertyDialog : public ContourPropertyDialog {

  public:
    SpherePropertyDialog(Sphere *sphere, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *radius, *visu;
};

class GroupPropertyDialog : public ElementPropertyDialog {

  public:
    GroupPropertyDialog(Group *group, QWidget * parent = 0, Qt::WindowFlags f = 0, bool kinematics=true);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *position, *orientation, *frameOfReference; 
};

class SolverPropertyDialog : public GroupPropertyDialog {
  protected:
    ExtWidget *environment, *solverParameters, *inverseKinetics;

  public:
    SolverPropertyDialog(Solver *solver, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
};

class ObjectPropertyDialog : public ElementPropertyDialog {
  Q_OBJECT;

  public:
    ObjectPropertyDialog(Object *object, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
    virtual void resizeGeneralizedPosition() {}
    virtual void resizeGeneralizedVelocity() {}
  protected:
    ExtWidget *q0, *u0;
    VecWidget *q0_, *u0_;
  public slots:
    void resizeVariables() {resizeGeneralizedPosition();resizeGeneralizedVelocity();}
};

class BodyPropertyDialog : public ObjectPropertyDialog {

  public:
    BodyPropertyDialog(Body *body, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *R;
};

class RigidBodyPropertyDialog : public BodyPropertyDialog {

  public:
    RigidBodyPropertyDialog(RigidBody *body, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
    void resizeGeneralizedPosition();
    void resizeGeneralizedVelocity();
    int getSize() const; 
  protected:
    ExtWidget *K, *mass, *inertia, *translation, *rotation, *ombvEditor, *weightArrow, *jointForceArrow, *jointMomentArrow, *isFrameOfBodyForRotation;
    RigidBody *body;
};

class ConstraintPropertyDialog : public ObjectPropertyDialog {

  public:
    ConstraintPropertyDialog(Constraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
};

class GearConstraintPropertyDialog : public ConstraintPropertyDialog {
  Q_OBJECT

  public:
    GearConstraintPropertyDialog(GearConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *dependentBody, *independentBodies;
    RigidBody *refBody;
  protected slots:
    void updateReferenceBody();
};

class KinematicConstraintPropertyDialog : public ConstraintPropertyDialog {
  Q_OBJECT

  public:
    KinematicConstraintPropertyDialog(KinematicConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *dependentBody, *kinematicFunction, *firstDerivativeOfKinematicFunction, *secondDerivativeOfKinematicFunction;
    RigidBody *refBody;
  protected slots:
    void resizeVariables();
    void updateReferenceBody();
};

class JointConstraintPropertyDialog : public ConstraintPropertyDialog {

  public:
    JointConstraintPropertyDialog(JointConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
    void resizeGeneralizedPosition();
  protected:
    ExtWidget *force, *moment, *connections, *independentBody, *dependentBodiesFirstSide, *dependentBodiesSecondSide;
};

class LinkPropertyDialog : public ElementPropertyDialog {

  public:
    LinkPropertyDialog(Link *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
};

class KineticExcitationPropertyDialog : public LinkPropertyDialog {

  public:
    KineticExcitationPropertyDialog(KineticExcitation *kineticExcitation, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *force, *moment, *connections, *frameOfReference, *forceArrow, *momentArrow;
};

class SpringDamperPropertyDialog : public LinkPropertyDialog {

  public:
    SpringDamperPropertyDialog(SpringDamper *springDamper, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *forceFunction, *connections, *forceDirection, *coilSpring;
};

class JointPropertyDialog : public LinkPropertyDialog {

  public:
    JointPropertyDialog(Joint *joint, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *force, *moment, *connections, *forceArrow, *momentArrow;
};

class ContactPropertyDialog : public LinkPropertyDialog {

  public:
    ContactPropertyDialog(Contact *contact, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *contactForceLaw, *contactImpactLaw, *frictionForceLaw, *frictionImpactLaw, *connections, *enableOpenMBVContactPoints, *normalForceArrow, *frictionArrow;
};

class ObserverPropertyDialog : public ElementPropertyDialog {

  public:
    ObserverPropertyDialog(Observer *observer, QWidget * parent = 0, Qt::WindowFlags f = 0) : ElementPropertyDialog(parent,f) {}
};

class AbsoluteKinematicsObserverPropertyDialog : public ObserverPropertyDialog {

  public:
    AbsoluteKinematicsObserverPropertyDialog(AbsoluteKinematicsObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
  protected:
    ExtWidget *frame, *position, *velocity, *angularVelocity, *acceleration, *angularAcceleration;
};

#endif
