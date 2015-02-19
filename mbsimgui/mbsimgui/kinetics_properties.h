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

#ifndef _KINETICS_PROPERTIES_H_
#define _KINETICS_PROPERTIES_H_

#include "element.h"
#include "basic_properties.h"
#include <QComboBox>

namespace MBSimGUI {

  class Function;

  class GeneralizedForceLaw : public Element {

    public:
      GeneralizedForceLaw(const std::string &name, Element *parent) : Element(name,parent), forceFunc(0) {}
      GeneralizedForceLaw(const GeneralizedForceLaw &p);
      ~GeneralizedForceLaw();
      GeneralizedForceLaw& operator=(const GeneralizedForceLaw &p);
      virtual std::string getType() const { return "GeneralizedForceLaw"; }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget) { }
      void toWidget(QWidget *widget) { }
   protected:
      Function *forceFunc;
  };

  class BilateralConstraint : public GeneralizedForceLaw {

    public:
      BilateralConstraint(const std::string &name, Element *parent) : GeneralizedForceLaw(name,parent) {}
      virtual PropertyInterface* clone() const {return new BilateralConstraint(*this);}
      std::string getType() const { return "BilateralConstraint"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) { return NULL; }
      void fromWidget(QWidget *widget) {}
      void toWidget(QWidget *widget) {}
  };

  class RegularizedBilateralConstraint : public GeneralizedForceLaw {

    public:
      RegularizedBilateralConstraint(const std::string &name, Element *parent) : GeneralizedForceLaw(name,parent), index(0) {}
      virtual PropertyInterface* clone() const {return new RegularizedBilateralConstraint(*this);}
      std::string getType() const { return "RegularizedBilateralConstraint"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      void defineFunction(int);
      int index;
  };

  class UnilateralConstraint : public GeneralizedForceLaw {

    public:
      UnilateralConstraint(const std::string &name, Element *parent) : GeneralizedForceLaw(name,parent) {}
      virtual PropertyInterface* clone() const {return new UnilateralConstraint(*this);}
      std::string getType() const { return "UnilateralConstraint"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) { return NULL; }
      void fromWidget(QWidget *widget) {}
      void toWidget(QWidget *widget) {}
  };

  class RegularizedUnilateralConstraint : public GeneralizedForceLaw {

    public:
      RegularizedUnilateralConstraint(const std::string &name, Element *parent) : GeneralizedForceLaw(name,parent), index(0) {} 
      virtual PropertyInterface* clone() const {return new RegularizedUnilateralConstraint(*this);}
      std::string getType() const { return "RegularizedUnilateralConstraint"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      void defineFunction(int);
      int index;
  };

  class GeneralizedImpactLaw : public Element {

    public:
      GeneralizedImpactLaw(const std::string &name, Element *parent) : Element(name,parent) {} 
      virtual std::string getType() const { return "GeneralizedImpactLaw"; }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget) { }
      void toWidget(QWidget *widget) { }
  };

  class BilateralImpact : public GeneralizedImpactLaw {

    public:
      BilateralImpact(const std::string &name, Element *parent) : GeneralizedImpactLaw(name,parent) {} 
      virtual PropertyInterface* clone() const {return new BilateralImpact(*this);}
      std::string getType() const { return "BilateralImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) { return NULL; }
      void fromWidget(QWidget *widget) {}
      void toWidget(QWidget *widget) {}
  };

  class UnilateralNewtonImpact : public GeneralizedImpactLaw {

    public:
      UnilateralNewtonImpact(const std::string &name, Element *parent);
      virtual PropertyInterface* clone() const {return new UnilateralNewtonImpact(*this);}
      std::string getType() const { return "UnilateralNewtonImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element); 
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty restitutionCoefficient;
  };

  class FrictionForceLaw : public Element {

    public:
      FrictionForceLaw(const std::string &name, Element *parent) : Element(name,parent), frictionForceFunc(0) {}
      FrictionForceLaw(const FrictionForceLaw &p);
      ~FrictionForceLaw();
      FrictionForceLaw& operator=(const FrictionForceLaw &p);
      virtual std::string getType() const { return "FrictionForceLaw"; }
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget) { }
      void toWidget(QWidget *widget) { }
    protected:
      Function *frictionForceFunc;
  };

  class PlanarCoulombFriction : public FrictionForceLaw {

    public:
      PlanarCoulombFriction(const std::string &name, Element *parent);
      virtual PropertyInterface* clone() const {return new PlanarCoulombFriction(*this);}
      std::string getType() const { return "PlanarCoulombFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element); 
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty frictionCoefficient;
  };

  class SpatialCoulombFriction : public FrictionForceLaw {

    public:
      SpatialCoulombFriction(const std::string &name, Element *parent);
      virtual PropertyInterface* clone() const {return new SpatialCoulombFriction(*this);}
      std::string getType() const { return "SpatialCoulombFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element); 
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty frictionCoefficient;
  };

  class RegularizedPlanarFriction : public FrictionForceLaw {

    public:
      RegularizedPlanarFriction(const std::string &name, Element *parent) : FrictionForceLaw(name,parent), index(0) {defineFunction(0);}
      virtual PropertyInterface* clone() const {return new RegularizedPlanarFriction(*this);}
      std::string getType() const { return "RegularizedPlanarFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element); 
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      void defineFunction(int);
      int index;
  };

  class RegularizedSpatialFriction : public FrictionForceLaw {

    public:
      RegularizedSpatialFriction(const std::string &name, Element *parent) : FrictionForceLaw(name,parent), index(0) {defineFunction(0);}
      virtual PropertyInterface* clone() const {return new RegularizedSpatialFriction(*this);}
      std::string getType() const { return "RegularizedSpatialFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element); 
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      void defineFunction(int);
      int index;
  };

  class FrictionImpactLaw : public Element {

    public:
      FrictionImpactLaw(const std::string &name, Element *parent) : Element(name,parent) {}
      virtual std::string getType() const { return "FrictionImpactLaw"; }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget) { }
      void toWidget(QWidget *widget) { }
  };

  class PlanarCoulombImpact : public FrictionImpactLaw {

    public:
      PlanarCoulombImpact(const std::string &name, Element *parent); 
      virtual PropertyInterface* clone() const {return new PlanarCoulombImpact(*this);}
      std::string getType() const { return "PlanarCoulombImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element); 
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty frictionCoefficient;
  };

  class SpatialCoulombImpact : public FrictionImpactLaw {

    public:
      SpatialCoulombImpact(const std::string &name, Element *parent);
      virtual PropertyInterface* clone() const {return new SpatialCoulombImpact(*this);}
      std::string getType() const { return "SpatialCoulombImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element); 
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty frictionCoefficient;
  };

  class GeneralizedForceLawChoiceProperty : public Property {

    public:
      GeneralizedForceLawChoiceProperty(Element *parent_, const MBXMLUtils::FQN &xmlName_) : parent(parent_), generalizedForceLaw(0), index(0), xmlName(xmlName_) {defineForceLaw(0);}
      GeneralizedForceLawChoiceProperty(const GeneralizedForceLawChoiceProperty &p) : generalizedForceLaw(static_cast<GeneralizedForceLaw*>(p.generalizedForceLaw->clone())), index(p.index), xmlName(p.xmlName) {}
      ~GeneralizedForceLawChoiceProperty() {delete generalizedForceLaw;}
      GeneralizedForceLawChoiceProperty& operator=(const GeneralizedForceLawChoiceProperty &p) {delete generalizedForceLaw; generalizedForceLaw=static_cast<GeneralizedForceLaw*>(p.generalizedForceLaw->clone()); index=p.index; xmlName=p.xmlName; return *this; }
      virtual PropertyInterface* clone() const {return new GeneralizedForceLawChoiceProperty(*this);}

      void defineForceLaw(int);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      Element *parent;
      GeneralizedForceLaw *generalizedForceLaw;
      int index;
      MBXMLUtils::FQN xmlName;
  };

  class GeneralizedImpactLawChoiceProperty : public Property {

    public:
      GeneralizedImpactLawChoiceProperty(Element *parent_, const MBXMLUtils::FQN &xmlName_) : parent(parent_), generalizedImpactLaw(0), index(0), xmlName(xmlName_) {defineImpactLaw(0);}
      GeneralizedImpactLawChoiceProperty(const GeneralizedImpactLawChoiceProperty &p) : generalizedImpactLaw(static_cast<GeneralizedImpactLaw*>(p.generalizedImpactLaw->clone())), index(p.index), xmlName(p.xmlName) {}
      ~GeneralizedImpactLawChoiceProperty() {delete generalizedImpactLaw;}
      GeneralizedImpactLawChoiceProperty& operator=(const GeneralizedImpactLawChoiceProperty &p) {delete generalizedImpactLaw; generalizedImpactLaw=static_cast<GeneralizedImpactLaw*>(p.generalizedImpactLaw->clone()); index=p.index; xmlName=p.xmlName; return *this; }
      virtual PropertyInterface* clone() const {return new GeneralizedImpactLawChoiceProperty(*this);}

      void defineImpactLaw(int);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      Element *parent;
      GeneralizedImpactLaw *generalizedImpactLaw;
      int index;
      MBXMLUtils::FQN xmlName;
  };

  class FrictionForceLawChoiceProperty : public Property {

    public:
      FrictionForceLawChoiceProperty(Element *parent_, const MBXMLUtils::FQN &xmlName_) : parent(parent_), frictionForceLaw(0), index(0), xmlName(xmlName_) {defineFrictionLaw(0);}
      FrictionForceLawChoiceProperty(const FrictionForceLawChoiceProperty &p) : frictionForceLaw(static_cast<FrictionForceLaw*>(p.frictionForceLaw->clone())), index(p.index), xmlName(p.xmlName) {}
      ~FrictionForceLawChoiceProperty() {delete frictionForceLaw;}
      FrictionForceLawChoiceProperty& operator=(const FrictionForceLawChoiceProperty &p) {delete frictionForceLaw; frictionForceLaw=static_cast<FrictionForceLaw*>(p.frictionForceLaw->clone()); index=p.index; xmlName=p.xmlName; return *this; }
      virtual PropertyInterface* clone() const {return new FrictionForceLawChoiceProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

      void defineFrictionLaw(int);

    protected:
      Element *parent;
      FrictionForceLaw *frictionForceLaw;
      int index;
      MBXMLUtils::FQN xmlName;
  };

  class FrictionImpactLawChoiceProperty : public Property {

    public:
      FrictionImpactLawChoiceProperty(Element *parent_, const MBXMLUtils::FQN &xmlName_) : parent(parent_), frictionImpactLaw(0), index(0), xmlName(xmlName_) {defineFrictionImpactLaw(0);}
      FrictionImpactLawChoiceProperty(const FrictionImpactLawChoiceProperty &p) : frictionImpactLaw(static_cast<FrictionImpactLaw*>(p.frictionImpactLaw->clone())), index(p.index), xmlName(p.xmlName) {}
      ~FrictionImpactLawChoiceProperty() {delete frictionImpactLaw;}
      FrictionImpactLawChoiceProperty& operator=(const FrictionImpactLawChoiceProperty &p) {delete frictionImpactLaw; frictionImpactLaw=static_cast<FrictionImpactLaw*>(p.frictionImpactLaw->clone()); index=p.index; xmlName=p.xmlName; return *this; }
      virtual PropertyInterface* clone() const {return new FrictionImpactLawChoiceProperty(*this);}

      void defineFrictionImpactLaw(int);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      Element *parent;
      FrictionImpactLaw *frictionImpactLaw;
      int index;
      MBXMLUtils::FQN xmlName;
  };

}

#endif

