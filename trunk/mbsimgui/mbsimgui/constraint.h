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

#ifndef _CONSTRAINT__H_
#define _CONSTRAINT__H_

#include "object.h"
#include "extended_properties.h"

class RigidBody;

class Constraint : public Object {
  private:
  public:
    Constraint(const QString &str, QTreeWidgetItem *parentItem, int ind);
    ~Constraint();
  protected:
    friend class MainWindow;
};

class GearConstraint : public Constraint {
  Q_OBJECT

  public:
    GearConstraint(const QString &str, QTreeWidgetItem *parentItem, int ind);
    ~GearConstraint();

    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

    virtual QString getType() const { return "GearConstraint"; }

    virtual void initializeDialog();
    virtual void fromWidget();
    virtual void toWidget();
    void initialize();

  protected:
    RigidBody *refBody;
    ExtWidget *dependentBodyWidget, *independentBodiesWidget;
    ExtProperty dependentBody, independentBodies;

  protected slots:
    void resizeVariables();
    void updateReferenceBody();
};

class KinematicConstraint : public Constraint {
  Q_OBJECT

  public:
    KinematicConstraint(const QString &str, QTreeWidgetItem *parentItem, int ind);
    ~KinematicConstraint();

    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

    virtual QString getType() const { return "KinematicConstraint"; }
    //virtual void resizeVariables() {Element::resizeVariables();}

    virtual void initializeDialog();
    virtual void fromWidget();
    virtual void toWidget();
    void initialize();

  protected:
    RigidBody *refBody;
    ExtWidget *dependentBodyWidget, *kinematicFunctionWidget, *firstDerivativeOfKinematicFunctionWidget, *secondDerivativeOfKinematicFunctionWidget;
    ExtProperty dependentBody, kinematicFunction, firstDerivativeOfKinematicFunction, secondDerivativeOfKinematicFunction;

  protected slots:
    void resizeVariables();
    void updateReferenceBody();
};

class JointConstraint : public Constraint {

  public:
    JointConstraint(const QString &str, QTreeWidgetItem *parentItem, int ind);
    ~JointConstraint();

    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

    virtual QString getType() const { return "JointConstraint"; }

    virtual void initializeDialog();
    virtual void fromWidget();
    virtual void toWidget();
    void initialize();

  protected:
    ExtWidget *forceWidget, *momentWidget, *connectionsWidget, *independentBodyWidget, *dependentBodiesFirstSideWidget, *dependentBodiesSecondSideWidget;
    ExtProperty force, moment, connections, independentBody, dependentBodiesFirstSide, dependentBodiesSecondSide;

    void resizeGeneralizedPosition();
};

#endif
