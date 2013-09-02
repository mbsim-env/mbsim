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

#include "basic_properties.h"
#include <QComboBox>

class FunctionProperty;

class GeneralizedForceLawProperty : public Property {

  public:
    GeneralizedForceLawProperty() : forceFunc(0) {}
    GeneralizedForceLawProperty(const GeneralizedForceLawProperty &p);
    ~GeneralizedForceLawProperty();
    GeneralizedForceLawProperty& operator=(const GeneralizedForceLawProperty &p);
    virtual std::string getType() const { return "GeneralizedForceLaw"; }
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
   protected:
    FunctionProperty *forceFunc;
};

class BilateralConstraintProperty : public GeneralizedForceLawProperty {

  public:
    virtual Property* clone() const {return new BilateralConstraintProperty(*this);}
    std::string getType() const { return "BilateralConstraint"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RegularizedBilateralConstraintProperty : public GeneralizedForceLawProperty {

  public:
    RegularizedBilateralConstraintProperty() : index(0) {}
    virtual Property* clone() const {return new RegularizedBilateralConstraintProperty(*this);}
    std::string getType() const { return "RegularizedBilateralConstraint"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    void defineFunction(int);
    int index;
};

class UnilateralConstraintProperty : public GeneralizedForceLawProperty {

  public:
    virtual Property* clone() const {return new UnilateralConstraintProperty(*this);}
    std::string getType() const { return "UnilateralConstraint"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RegularizedUnilateralConstraintProperty : public GeneralizedForceLawProperty {

  public:
    RegularizedUnilateralConstraintProperty() : index(0) {} 
    virtual Property* clone() const {return new RegularizedUnilateralConstraintProperty(*this);}
    std::string getType() const { return "RegularizedUnilateralConstraint"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    void defineFunction(int);
    int index;
};

class GeneralizedImpactLawProperty : public Property {

  public:
    virtual std::string getType() const { return "GeneralizedImpactLaw"; }
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class BilateralImpactProperty : public GeneralizedImpactLawProperty {

  public:
    virtual Property* clone() const {return new BilateralImpactProperty(*this);}
    std::string getType() const { return "BilateralImpact"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class UnilateralNewtonImpactProperty : public GeneralizedImpactLawProperty {

  public:
    UnilateralNewtonImpactProperty();
    virtual Property* clone() const {return new UnilateralNewtonImpactProperty(*this);}
    std::string getType() const { return "UnilateralNewtonImpact"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element); 
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty restitutionCoefficient;
};

class FrictionForceLawProperty : public Property {

  public:
    FrictionForceLawProperty() : frictionForceFunc(0) {}
    FrictionForceLawProperty(const FrictionForceLawProperty &p);
    ~FrictionForceLawProperty();
    FrictionForceLawProperty& operator=(const FrictionForceLawProperty &p);
    virtual std::string getType() const { return "FrictionForceLaw"; }
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
   protected:
    FunctionProperty *frictionForceFunc;
};

class PlanarCoulombFrictionProperty : public FrictionForceLawProperty {

  public:
    PlanarCoulombFrictionProperty();
    virtual Property* clone() const {return new PlanarCoulombFrictionProperty(*this);}
    std::string getType() const { return "PlanarCoulombFriction"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element); 
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty frictionCoefficient;
};

class SpatialCoulombFrictionProperty : public FrictionForceLawProperty {

  public:
    SpatialCoulombFrictionProperty();
    virtual Property* clone() const {return new SpatialCoulombFrictionProperty(*this);}
    std::string getType() const { return "SpatialCoulombFriction"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element); 
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty frictionCoefficient;
};

class RegularizedPlanarFrictionProperty : public FrictionForceLawProperty {

  public:
    RegularizedPlanarFrictionProperty() : index(0) {defineFunction(0);}
    virtual Property* clone() const {return new RegularizedPlanarFrictionProperty(*this);}
    std::string getType() const { return "RegularizedPlanarFriction"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    void defineFunction(int);
    int index;
};

class RegularizedSpatialFrictionProperty : public FrictionForceLawProperty {

  public:
    RegularizedSpatialFrictionProperty() : index(0) {defineFunction(0);}
    virtual Property* clone() const {return new RegularizedSpatialFrictionProperty(*this);}
    std::string getType() const { return "RegularizedSpatialFriction"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    void defineFunction(int);
    int index;
};

class FrictionImpactLawProperty : public Property {

  public:
    FrictionImpactLawProperty() {}
    virtual std::string getType() const { return "FrictionImpactLaw"; }
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class PlanarCoulombImpactProperty : public FrictionImpactLawProperty {

  public:
    PlanarCoulombImpactProperty();
    virtual Property* clone() const {return new PlanarCoulombImpactProperty(*this);}
    std::string getType() const { return "PlanarCoulombImpact"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element); 
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty frictionCoefficient;
};

class SpatialCoulombImpactProperty : public FrictionImpactLawProperty {

  public:
    SpatialCoulombImpactProperty();
    virtual Property* clone() const {return new SpatialCoulombImpactProperty(*this);}
    std::string getType() const { return "SpatialCoulombImpact"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element); 
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty frictionCoefficient;
};

class GeneralizedForceLawChoiceProperty : public Property {

  public:
    GeneralizedForceLawChoiceProperty(const std::string &xmlName_) : generalizedForceLaw(0), index(0), xmlName(xmlName_) {defineForceLaw(0);}
    GeneralizedForceLawChoiceProperty(const GeneralizedForceLawChoiceProperty &p) : generalizedForceLaw(static_cast<GeneralizedForceLawProperty*>(p.generalizedForceLaw->clone())), index(p.index), xmlName(p.xmlName) {}
    ~GeneralizedForceLawChoiceProperty() {delete generalizedForceLaw;}
    GeneralizedForceLawChoiceProperty& operator=(const GeneralizedForceLawChoiceProperty &p) {delete generalizedForceLaw; generalizedForceLaw=static_cast<GeneralizedForceLawProperty*>(p.generalizedForceLaw->clone()); index=p.index; xmlName=p.xmlName;}
    virtual Property* clone() const {return new GeneralizedForceLawChoiceProperty(*this);}

    void defineForceLaw(int);
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    GeneralizedForceLawProperty *generalizedForceLaw;
    int index;
    std::string xmlName;
};

class GeneralizedImpactLawChoiceProperty : public Property {

  public:
    GeneralizedImpactLawChoiceProperty(const std::string &xmlName_) : generalizedImpactLaw(0), index(0), xmlName(xmlName_) {defineImpactLaw(0);}
    GeneralizedImpactLawChoiceProperty(const GeneralizedImpactLawChoiceProperty &p) : generalizedImpactLaw(static_cast<GeneralizedImpactLawProperty*>(p.generalizedImpactLaw->clone())), index(p.index), xmlName(p.xmlName) {}
    ~GeneralizedImpactLawChoiceProperty() {delete generalizedImpactLaw;}
    GeneralizedImpactLawChoiceProperty& operator=(const GeneralizedImpactLawChoiceProperty &p) {delete generalizedImpactLaw; generalizedImpactLaw=static_cast<GeneralizedImpactLawProperty*>(p.generalizedImpactLaw->clone()); index=p.index; xmlName=p.xmlName;}
    virtual Property* clone() const {return new GeneralizedImpactLawChoiceProperty(*this);}

    void defineImpactLaw(int);
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    GeneralizedImpactLawProperty *generalizedImpactLaw;
    int index;
    std::string xmlName;
};

class FrictionForceLawChoiceProperty : public Property {

  public:
    FrictionForceLawChoiceProperty(const std::string &xmlName_) : frictionForceLaw(0), index(0), xmlName(xmlName_) {defineFrictionLaw(0);}
    FrictionForceLawChoiceProperty(const FrictionForceLawChoiceProperty &p) : frictionForceLaw(static_cast<FrictionForceLawProperty*>(p.frictionForceLaw->clone())), index(p.index), xmlName(p.xmlName) {}
    ~FrictionForceLawChoiceProperty() {delete frictionForceLaw;}
    FrictionForceLawChoiceProperty& operator=(const FrictionForceLawChoiceProperty &p) {delete frictionForceLaw; frictionForceLaw=static_cast<FrictionForceLawProperty*>(p.frictionForceLaw->clone()); index=p.index; xmlName=p.xmlName;}
    virtual Property* clone() const {return new FrictionForceLawChoiceProperty(*this);}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

    void defineFrictionLaw(int);

  protected:
    FrictionForceLawProperty *frictionForceLaw;
    int index;
    std::string xmlName;
};

class FrictionImpactLawChoiceProperty : public Property {

  public:
    FrictionImpactLawChoiceProperty(const std::string &xmlName_) : frictionImpactLaw(0), index(0), xmlName(xmlName_) {defineFrictionImpactLaw(0);}
    FrictionImpactLawChoiceProperty(const FrictionImpactLawChoiceProperty &p) : frictionImpactLaw(static_cast<FrictionImpactLawProperty*>(p.frictionImpactLaw->clone())), index(p.index), xmlName(p.xmlName) {}
    ~FrictionImpactLawChoiceProperty() {delete frictionImpactLaw;}
    FrictionImpactLawChoiceProperty& operator=(const FrictionImpactLawChoiceProperty &p) {delete frictionImpactLaw; frictionImpactLaw=static_cast<FrictionImpactLawProperty*>(p.frictionImpactLaw->clone()); index=p.index; xmlName=p.xmlName;}
    virtual Property* clone() const {return new FrictionImpactLawChoiceProperty(*this);}

    void defineFrictionImpactLaw(int);
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    FrictionImpactLawProperty *frictionImpactLaw;
    int index;
    std::string xmlName;
};

class GeneralizedForceChoiceProperty : public Property {

  public:
    GeneralizedForceChoiceProperty(ExtProperty& arrow, const std::string &xmlName);
    virtual Property* clone() const {return new GeneralizedForceChoiceProperty(*this);}

    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty generalizedForceLaw, generalizedImpactLaw, mat, &arrow;
    std::string xmlName;
};

class ForceChoiceProperty : public Property {

  public:
    ForceChoiceProperty(ExtProperty &arrow, const std::string &xmlName);
    virtual Property* clone() const {return new ForceChoiceProperty(*this);}

    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty forceLaw, mat, &arrow;
    std::string xmlName;
};

class ForceDirectionProperty : public Property {

  public:
    ForceDirectionProperty(Element *element, const std::string &xmlName);
    virtual Property* clone() const {return new ForceDirectionProperty(*this);}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty refFrame, mat;
    std::string xmlName;
};

class GeneralizedForceDirectionProperty : public Property {

  public:
    GeneralizedForceDirectionProperty(const std::string &xmlName);
    virtual Property* clone() const {return new GeneralizedForceDirectionProperty(*this);}

    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty mat;
};

#endif

