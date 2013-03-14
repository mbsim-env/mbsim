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

//class FrictionForceLawWidget : public Widget {
//
//  public:
//    FrictionForceLawWidget() : frictionForceFunc(0) {}
//    QString getType() const { return "FrictionForceLaw"; }
//    void updateWidget() {}
//    void resizeVariables() {}
//   protected:
//    Function2 *frictionForceFunc;
//};
//
//class PlanarCoulombFriction : public FrictionForceLawWidget {
//
//  public:
//    PlanarCoulombFriction();
//    QString getType() const { return "PlanarCoulombFriction"; }
//  protected:
//    ExtProperty frictionCoefficient;
//};
//
//class SpatialCoulombFriction : public FrictionForceLawWidget {
//
//  public:
//    SpatialCoulombFriction();
//    QString getType() const { return "SpatialCoulombFriction"; }
//  protected:
//    ExtProperty frictionCoefficient;
//};
//
//class RegularizedPlanarFriction : public FrictionForceLawWidget {
//
//  public:
//    RegularizedPlanarFriction(); 
//    QString getType() const { return "RegularizedPlanarFriction"; }
//  protected:
//    QVBoxLayout *layout;
//    QComboBox *funcList;
//  protected slots:
//    void defineFunction(int);
//};
//
//class RegularizedSpatialFriction : public FrictionForceLawWidget {
//
//  public:
//    RegularizedSpatialFriction(); 
//    QString getType() const { return "RegularizedSpatialFriction"; }
//  protected:
//    QVBoxLayout *layout;
//    QComboBox *funcList;
//  protected slots:
//    void defineFunction(int);
//};
//
//class FrictionImpactLawWidget : public Widget {
//
//  public:
//    FrictionImpactLawWidget() {}
//    QString getType() const { return "FrictionImpactLaw"; }
//    void updateWidget() {}
//    void resizeVariables() {}
//};
//
//class PlanarCoulombImpact : public FrictionImpactLawWidget {
//
//  public:
//    PlanarCoulombImpact();
//    QString getType() const { return "PlanarCoulombImpact"; }
//  protected:
//    ExtProperty frictionCoefficient;
//};
//
//class SpatialCoulombImpact : public FrictionImpactLawWidget {
//
//  public:
//    SpatialCoulombImpact();
//    QString getType() const { return "SpatialCoulombImpact"; }
//  protected:
//    ExtProperty frictionCoefficient;
//};

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
//
//class FrictionForceLawChoiceWidget : public Widget {
//
//  public:
//    FrictionForceLawChoiceWidget();
//
//    int getForceLaw() {return comboBox->currentIndex();}
//    void updateWidget() {}
//    void resizeVariables() {}
//
//  protected slots:
//    void defineFrictionLaw(int);
//
//  protected:
//    QComboBox *comboBox;
//    QVBoxLayout *layout;
//    FrictionForceLawWidget *frictionForceLaw;
//};
//
//class FrictionImpactLawChoiceWidget : public Widget {
//
//  public:
//    FrictionImpactLawChoiceWidget();
//
//    int getImpactLaw() {return comboBox->currentIndex();}
//    void updateWidget() {}
//    void resizeVariables() {}
//
//  protected slots:
//    void defineFrictionImpactLaw(int);
//
//  protected:
//    QComboBox *comboBox;
//    QVBoxLayout *layout;
//    FrictionImpactLawWidget *frictionImpactLaw;
//};

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

//class ForceDirectionWidget : public Widget {
//
//  public:
//    ForceDirectionWidget(Element *element);
//
//    void updateWidget() {}
//    void resizeVariables() {}
//
//  protected:
//    QWidget *forceDirWidget;
//    FrameOfReferenceWidget* refFrame;
//    Element *element;
//    ExtPhysicalVarWidget *mat;
//};

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

