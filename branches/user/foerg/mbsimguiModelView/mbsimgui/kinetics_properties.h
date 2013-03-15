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

class Function2Property;
class Function1ChoiceProperty;

class GeneralizedForceLawProperty : public Property {

  public:
    GeneralizedForceLawProperty() : forceFunc(0) {}
    virtual QString getType() const { return "GeneralizedForceLaw"; }
    TiXmlElement* writeXMLFile(TiXmlNode *element);
   protected:
    Function2Property *forceFunc;
};

class BilateralConstraintProperty : public GeneralizedForceLawProperty {

  public:
    BilateralConstraintProperty() {}
    QString getType() const { return "BilateralConstraint"; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RegularizedBilateralConstraintProperty : public GeneralizedForceLawProperty {

  public:
    RegularizedBilateralConstraintProperty() : index(0) {}
    QString getType() const { return "RegularizedBilateralConstraint"; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    void defineFunction(int);
    int index;
};

class UnilateralConstraintProperty : public GeneralizedForceLawProperty {

  public:
    UnilateralConstraintProperty() {}
    QString getType() const { return "UnilateralConstraint"; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RegularizedUnilateralConstraintProperty : public GeneralizedForceLawProperty {

  public:
    RegularizedUnilateralConstraintProperty() : index(0) {} 
    QString getType() const { return "RegularizedUnilateralConstraint"; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    void defineFunction(int);
    int index;
};

class GeneralizedImpactLawProperty : public Property {

  public:
    GeneralizedImpactLawProperty() {}
    virtual QString getType() const { return "GeneralizedImpactLaw"; }
    TiXmlElement* writeXMLFile(TiXmlNode *element);
};

class BilateralImpactProperty : public GeneralizedImpactLawProperty {

  public:
    BilateralImpactProperty() {}
    QString getType() const { return "BilateralImpact"; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class UnilateralNewtonImpactProperty : public GeneralizedImpactLawProperty {

  public:
    UnilateralNewtonImpactProperty();
    QString getType() const { return "UnilateralNewtonImpact"; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element); 
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty restitutionCoefficient;
};

class FrictionForceLawProperty : public Property {

  public:
    FrictionForceLawProperty() : frictionForceFunc(0) {}
    QString getType() const { return "FrictionForceLaw"; }
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
   protected:
    Function2Property *frictionForceFunc;
};

class PlanarCoulombFrictionProperty : public FrictionForceLawProperty {

  public:
    PlanarCoulombFrictionProperty();
    QString getType() const { return "PlanarCoulombFriction"; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element); 
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty frictionCoefficient;
};

class SpatialCoulombFrictionProperty : public FrictionForceLawProperty {

  public:
    SpatialCoulombFrictionProperty();
    QString getType() const { return "SpatialCoulombFriction"; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element); 
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty frictionCoefficient;
};

class RegularizedPlanarFrictionProperty : public FrictionForceLawProperty {

  public:
    RegularizedPlanarFrictionProperty() : index(0) {defineFunction(0);}
    QString getType() const { return "RegularizedPlanarFriction"; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    void defineFunction(int);
    int index;
};

class RegularizedSpatialFrictionProperty : public FrictionForceLawProperty {

  public:
    RegularizedSpatialFrictionProperty() : index(0) {defineFunction(0);}
    QString getType() const { return "RegularizedSpatialFriction"; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    void defineFunction(int);
    int index;
};

class FrictionImpactLawProperty : public Property {

  public:
    FrictionImpactLawProperty() {}
    QString getType() const { return "FrictionImpactLaw"; }
    TiXmlElement* writeXMLFile(TiXmlNode *element);
};

class PlanarCoulombImpactProperty : public FrictionImpactLawProperty {

  public:
    PlanarCoulombImpactProperty();
    QString getType() const { return "PlanarCoulombImpact"; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element); 
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty frictionCoefficient;
};

class SpatialCoulombImpactProperty : public FrictionImpactLawProperty {

  public:
    SpatialCoulombImpactProperty();
    QString getType() const { return "SpatialCoulombImpact"; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element); 
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty frictionCoefficient;
};

class GeneralizedForceLawChoiceProperty : public Property {

  public:
    GeneralizedForceLawChoiceProperty(const std::string &xmlName_) : generalizedForceLaw(0), index(0), xmlName(xmlName_) {defineForceLaw(0);}

    void defineForceLaw(int);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
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

    void defineImpactLaw(int);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
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
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
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

    void defineFrictionImpactLaw(int);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
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

    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty generalizedForceLaw, generalizedImpactLaw, mat, &arrow;
    std::string xmlName;
};

class ForceChoiceProperty : public Property {

  public:
    ForceChoiceProperty(ExtProperty &arrow, const std::string &xmlName);

    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty forceLaw, mat, &arrow;
    std::string xmlName;
};

class ForceDirectionProperty : public Property {

  public:
    ForceDirectionProperty(Element *element, const std::string &xmlName);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    Element *element;
    ExtProperty refFrame, mat;
    std::string xmlName;
};

class GeneralizedForceDirectionProperty : public Property {

  public:
    GeneralizedForceDirectionProperty(const std::string &xmlName);

    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty mat;
};

#endif

