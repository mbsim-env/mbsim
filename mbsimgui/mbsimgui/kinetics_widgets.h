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
      GeneralizedForceLawWidget()  = default;
      virtual MBXMLUtils::FQN getXMLType() const { return MBSIM%"GeneralizedForceLaw"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *forceFunc{nullptr};
  };

  class BilateralConstraintWidget : public GeneralizedForceLawWidget {

    public:
      BilateralConstraintWidget() = default;
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"BilateralConstraint"; }
  };

  class RegularizedBilateralConstraintWidget : public GeneralizedForceLawWidget {

    public:
      RegularizedBilateralConstraintWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RegularizedBilateralConstraint"; }
  };

  class UnilateralConstraintWidget : public GeneralizedForceLawWidget {

    public:
      UnilateralConstraintWidget() = default;
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"UnilateralConstraint"; }
  };

  class RegularizedUnilateralConstraintWidget : public GeneralizedForceLawWidget {

    public:
      RegularizedUnilateralConstraintWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RegularizedUnilateralConstraint"; }
  };

  class GeneralizedImpactLawWidget : public Widget {

    public:
      GeneralizedImpactLawWidget() = default;
      virtual MBXMLUtils::FQN getXMLType() const { return MBSIM%"GeneralizedImpactLaw"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class BilateralImpactWidget : public GeneralizedImpactLawWidget {

    public:
      BilateralImpactWidget() = default;
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"BilateralImpact"; }
  };

  class UnilateralNewtonImpactWidget : public GeneralizedImpactLawWidget {

    public:
      UnilateralNewtonImpactWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"UnilateralNewtonImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget* restitutionCoefficient;
  };

  class FrictionForceLawWidget : public Widget {

    public:
      FrictionForceLawWidget()  = default;
      virtual MBXMLUtils::FQN getXMLType() const { return MBSIM%"FrictionForceLaw"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frictionForceFunc{nullptr};
  };

  class PlanarCoulombFrictionWidget : public FrictionForceLawWidget {

    public:
      PlanarCoulombFrictionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PlanarCoulombFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget* frictionCoefficient;
  };

  class SpatialCoulombFrictionWidget : public FrictionForceLawWidget {

    public:
      SpatialCoulombFrictionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SpatialCoulombFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget* frictionCoefficient;
  };

  class PlanarStribeckFrictionWidget : public FrictionForceLawWidget {

    public:
      PlanarStribeckFrictionWidget(QWidget *parent);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PlanarStribeckFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget* frictionFunction;
  };

  class SpatialStribeckFrictionWidget : public FrictionForceLawWidget {

    public:
      SpatialStribeckFrictionWidget(QWidget *parent);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SpatialStribeckFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget* frictionFunction;
  };

  class RegularizedPlanarFrictionWidget : public FrictionForceLawWidget {
    public:
      RegularizedPlanarFrictionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RegularizedPlanarFriction"; }
  };

  class RegularizedSpatialFrictionWidget : public FrictionForceLawWidget {
    public:
      RegularizedSpatialFrictionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RegularizedSpatialFriction"; }
  };

  class FrictionImpactLawWidget : public Widget {

    public:
      FrictionImpactLawWidget() = default;
      virtual MBXMLUtils::FQN getXMLType() const { return MBSIM%"FrictionImpactLaw"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class PlanarCoulombImpactWidget : public FrictionImpactLawWidget {

    public:
      PlanarCoulombImpactWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PlanarCoulombImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget* frictionCoefficient;
  };

  class SpatialCoulombImpactWidget : public FrictionImpactLawWidget {

    public:
      SpatialCoulombImpactWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SpatialCoulombImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget* frictionCoefficient;
  };

  class PlanarStribeckImpactWidget : public FrictionImpactLawWidget {

    public:
      PlanarStribeckImpactWidget(QWidget *parent);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PlanarStribeckImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget* frictionFunction;
  };

  class SpatialStribeckImpactWidget : public FrictionImpactLawWidget {

    public:
      SpatialStribeckImpactWidget(QWidget *parent);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SpatialStribeckImpact"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget* frictionFunction;
  };

  class GeneralizedForceLawWidgetFactory : public WidgetFactory {
    public:
      GeneralizedForceLawWidgetFactory();
      Widget* createWidget(int i=0) override;
      static std::vector<QString> getNames();
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class FrictionForceLawWidgetFactory : public WidgetFactory {
    public:
      FrictionForceLawWidgetFactory(QWidget *parent_);
      Widget* createWidget(int i=0) override;
      static std::vector<QString> getNames();
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QWidget *parent;
  };

  class GeneralizedImpactLawWidgetFactory : public WidgetFactory {
    public:
      GeneralizedImpactLawWidgetFactory();
      Widget* createWidget(int i=0) override;
      static std::vector<QString> getNames();
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class FrictionImpactLawWidgetFactory : public WidgetFactory {
    public:
      FrictionImpactLawWidgetFactory(QWidget *parent_);
      Widget* createWidget(int i=0) override;
      static std::vector<QString> getNames();
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QWidget *parent;
  };

  class RegularizedBilateralConstraintFunctionFactory : public WidgetFactory {
    public:
      RegularizedBilateralConstraintFunctionFactory();
      Widget* createWidget(int i=0) override;
      static std::vector<QString> getNames();
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class RegularizedUnilateralConstraintFunctionFactory : public WidgetFactory {
    public:
      RegularizedUnilateralConstraintFunctionFactory();
      Widget* createWidget(int i=0) override;
      static std::vector<QString> getNames();
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class FrictionFunctionFactory : public WidgetFactory {
    public:
      FrictionFunctionFactory();
      Widget* createWidget(int i=0) override;
      static std::vector<QString> getNames();
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

}

#endif
