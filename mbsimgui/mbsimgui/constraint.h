/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012-2016 Martin Förg

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

namespace MBSimGUI {

  class RigidBody;

  class RigidBodyOfReferencePropertyFactory : public PropertyFactory {
    public:
      RigidBodyOfReferencePropertyFactory(Element *element_, const MBXMLUtils::FQN &xmlName_) : element(element_), xmlName(xmlName_) { }
      Property* createProperty(int i=0);
    protected:
      Element *element;
      MBXMLUtils::FQN xmlName;
  };

  class GeneralizedGearConstraintPropertyFactory : public PropertyFactory {
    public:
      GeneralizedGearConstraintPropertyFactory(Element *element_, const MBXMLUtils::FQN &xmlName_="") : element(element_), xmlName(xmlName_) { }
      Property* createProperty(int i=0);
    protected:
      Element *element;
      MBXMLUtils::FQN xmlName;
  };

  class Constraint : public Element {
    public:
      Constraint(const std::string &str, Element *parent) : Element(str, parent) { }
      static Constraint* readXMLFile(const std::string &filename, Element *parent);
  };

  class GeneralizedConstraint : public Constraint {
    friend class GeneralizedConstraintPropertyDialog;
    public:
      GeneralizedConstraint(const std::string &str, Element *parent);
      void initialize();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    protected:
      ExtProperty support, forceArrow, momentArrow;
  };

  class GeneralizedGearConstraint : public GeneralizedConstraint {
    friend class GeneralizedGearConstraintPropertyDialog;
    public:
    GeneralizedGearConstraint(const std::string &str, Element *parent);
    virtual PropertyInterface* clone() const {return new GeneralizedGearConstraint(*this);}
    std::string getType() const { return "GeneralizedGearConstraint"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void initialize();
    void deinitialize();
    ElementPropertyDialog* createPropertyDialog() {return new GeneralizedGearConstraintPropertyDialog(this);}
    protected:
    ExtProperty dependentBody, independentBodies;
  };

  class GeneralizedDualConstraint : public GeneralizedConstraint {
    friend class GeneralizedDualConstraintPropertyDialog;
    public:
    GeneralizedDualConstraint(const std::string &str, Element *parent);
    std::string getType() const { return "GeneralizedDualConstraint"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void initialize();
    void deinitialize();
    ElementPropertyDialog* createPropertyDialog() {return new GeneralizedDualConstraintPropertyDialog(this);}
    protected:
    ExtProperty dependentBody, independentBody;
  };

  class GeneralizedPositionConstraint : public GeneralizedDualConstraint {
    friend class GeneralizedPositionConstraintPropertyDialog;
    public:
    GeneralizedPositionConstraint(const std::string &str, Element *parent);
    virtual PropertyInterface* clone() const {return new GeneralizedPositionConstraint(*this);}
    std::string getType() const { return "GeneralizedPositionConstraint"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new GeneralizedPositionConstraintPropertyDialog(this);}
    protected:
    ExtProperty constraintFunction;
  };

  class GeneralizedVelocityConstraint : public GeneralizedDualConstraint {
    friend class GeneralizedVelocityConstraintPropertyDialog;
    public:
    GeneralizedVelocityConstraint(const std::string &str, Element *parent);
    virtual PropertyInterface* clone() const {return new GeneralizedVelocityConstraint(*this);}
    std::string getType() const { return "GeneralizedVelocityConstraint"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new GeneralizedVelocityConstraintPropertyDialog(this);}
    protected:
    ExtProperty constraintFunction, x0;
  };

  class GeneralizedAccelerationConstraint : public GeneralizedDualConstraint {
    friend class GeneralizedAccelerationConstraintPropertyDialog;
    public:
    GeneralizedAccelerationConstraint(const std::string &str, Element *parent);
    virtual PropertyInterface* clone() const {return new GeneralizedAccelerationConstraint(*this);}
    std::string getType() const { return "GeneralizedAccelerationConstraint"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new GeneralizedAccelerationConstraintPropertyDialog(this);}
    protected:
    ExtProperty constraintFunction, x0;
  };

  class JointConstraint : public Constraint {
    friend class JointConstraintPropertyDialog;
    public:
    JointConstraint(const std::string &str, Element *parent);
    virtual PropertyInterface* clone() const {return new JointConstraint(*this);}
    std::string getType() const { return "JointConstraint"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void initialize();
    void deinitialize();
    ElementPropertyDialog* createPropertyDialog() {return new JointConstraintPropertyDialog(this);}
    protected:
    ExtProperty independentBody, dependentBodiesFirstSide, dependentBodiesSecondSide, refFrameID, force, moment, connections, jointForceArrow, jointMomentArrow, q0;

  };

  class GeneralizedConnectionConstraint : public GeneralizedDualConstraint {
    friend class GeneralizedConnectionConstraintPropertyDialog;
    public:
    GeneralizedConnectionConstraint(const std::string &str, Element *parent) : GeneralizedDualConstraint(str, parent) { }
    virtual PropertyInterface* clone() const {return new GeneralizedConnectionConstraint(*this);}
    std::string getType() const { return "GeneralizedConnectionConstraint"; }
    ElementPropertyDialog* createPropertyDialog() {return new GeneralizedConnectionConstraintPropertyDialog(this);}
  };

}

#endif
