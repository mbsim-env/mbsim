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

#ifndef _KINETICS_WIDGETS_H_
#define _KINETICS_WIDGETS_H_

#include "widget.h"
#include "namespace.h"
#include <QComboBox>

class QVBoxLayout;

namespace MBSimGUI {

  class ExtWidget;
  class Element;

  class GeneralizedForceLawWidget : public Widget {

    public:
      GeneralizedForceLawWidget() : forceFunc(0) {}
      virtual QString getType() const { return "GeneralizedForceLaw"; }
      virtual MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIM; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *forceFunc;
  };

  class BilateralConstraintWidget : public GeneralizedForceLawWidget {

    public:
      BilateralConstraintWidget() {}
      virtual QString getType() const { return "BilateralConstraint"; }
  };

  class RegularizedBilateralConstraintWidget : public GeneralizedForceLawWidget {

    public:
      RegularizedBilateralConstraintWidget();
      virtual QString getType() const { return "RegularizedBilateralConstraint"; }
  };

  class UnilateralConstraintWidget : public GeneralizedForceLawWidget {

    public:
      UnilateralConstraintWidget() {}
      virtual QString getType() const { return "UnilateralConstraint"; }
  };

  class RegularizedUnilateralConstraintWidget : public GeneralizedForceLawWidget {

    public:
      RegularizedUnilateralConstraintWidget();
      virtual QString getType() const { return "RegularizedUnilateralConstraint"; }
  };

  class GeneralizedImpactLawWidget : public Widget {

    public:
      GeneralizedImpactLawWidget() {}
      virtual QString getType() const { return "GeneralizedImpactLaw"; }
      virtual MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIM; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
  };

  class BilateralImpactWidget : public GeneralizedImpactLawWidget {

    public:
      BilateralImpactWidget() {}
      virtual QString getType() const { return "BilateralImpact"; }
  };

  class UnilateralNewtonImpactWidget : public GeneralizedImpactLawWidget {

    public:
      UnilateralNewtonImpactWidget();
      virtual QString getType() const { return "UnilateralNewtonImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget* restitutionCoefficient;
  };

  class FrictionForceLawWidget : public Widget {

    public:
      FrictionForceLawWidget() : frictionForceFunc(0) {}
      virtual QString getType() const { return "FrictionForceLaw"; }
      virtual MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIM; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *frictionForceFunc;
  };

  class PlanarCoulombFrictionWidget : public FrictionForceLawWidget {

    public:
      PlanarCoulombFrictionWidget();
      virtual QString getType() const { return "PlanarCoulombFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget* frictionCoefficient;
  };

  class SpatialCoulombFrictionWidget : public FrictionForceLawWidget {

    public:
      SpatialCoulombFrictionWidget();
      virtual QString getType() const { return "SpatialCoulombFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget* frictionCoefficient;
  };

  class PlanarStribeckFrictionWidget : public FrictionForceLawWidget {

    public:
      PlanarStribeckFrictionWidget();
      virtual QString getType() const { return "PlanarStribeckFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget* frictionFunction;
  };

  class SpatialStribeckFrictionWidget : public FrictionForceLawWidget {

    public:
      SpatialStribeckFrictionWidget();
      virtual QString getType() const { return "SpatialStribeckFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget* frictionFunction;
  };

  class RegularizedPlanarFrictionWidget : public FrictionForceLawWidget {
    public:
      RegularizedPlanarFrictionWidget();
      virtual QString getType() const { return "RegularizedPlanarFriction"; }
  };

  class RegularizedSpatialFrictionWidget : public FrictionForceLawWidget {
    public:
      RegularizedSpatialFrictionWidget();
      virtual QString getType() const { return "RegularizedSpatialFriction"; }
  };

  class FrictionImpactLawWidget : public Widget {

    public:
      FrictionImpactLawWidget() {}
      virtual QString getType() const { return "FrictionImpactLaw"; }
      virtual MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIM; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
  };

  class PlanarCoulombImpactWidget : public FrictionImpactLawWidget {

    public:
      PlanarCoulombImpactWidget();
      virtual QString getType() const { return "PlanarCoulombImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget* frictionCoefficient;
  };

  class SpatialCoulombImpactWidget : public FrictionImpactLawWidget {

    public:
      SpatialCoulombImpactWidget();
      virtual QString getType() const { return "SpatialCoulombImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget* frictionCoefficient;
  };

  class PlanarStribeckImpactWidget : public FrictionImpactLawWidget {

    public:
      PlanarStribeckImpactWidget();
      virtual QString getType() const { return "PlanarStribeckImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget* frictionFunction;
  };

  class SpatialStribeckImpactWidget : public FrictionImpactLawWidget {

    public:
      SpatialStribeckImpactWidget();
      virtual QString getType() const { return "SpatialStribeckImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget* frictionFunction;
  };

  class GeneralizedForceLawWidgetFactory : public WidgetFactory {
    public:
      GeneralizedForceLawWidgetFactory();
      QWidget* createWidget(int i=0);
      static std::vector<QString> getNames();
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class FrictionForceLawWidgetFactory : public WidgetFactory {
    public:
      FrictionForceLawWidgetFactory();
      QWidget* createWidget(int i=0);
      static std::vector<QString> getNames();
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class GeneralizedImpactLawWidgetFactory : public WidgetFactory {
    public:
      GeneralizedImpactLawWidgetFactory();
      QWidget* createWidget(int i=0);
      static std::vector<QString> getNames();
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class FrictionImpactLawWidgetFactory : public WidgetFactory {
    public:
      FrictionImpactLawWidgetFactory();
      QWidget* createWidget(int i=0);
      static std::vector<QString> getNames();
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class RegularizedBilateralConstraintFunctionFactory : public WidgetFactory {
    public:
      RegularizedBilateralConstraintFunctionFactory();
      QWidget* createWidget(int i=0);
      static std::vector<QString> getNames();
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class RegularizedUnilateralConstraintFunctionFactory : public WidgetFactory {
    public:
      RegularizedUnilateralConstraintFunctionFactory();
      QWidget* createWidget(int i=0);
      static std::vector<QString> getNames();
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class FrictionFunctionFactory : public WidgetFactory {
    public:
      FrictionFunctionFactory();
      QWidget* createWidget(int i=0);
      static std::vector<QString> getNames();
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

}

#endif
