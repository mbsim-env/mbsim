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
#include "basic_widgets.h"
#include <QComboBox>

class QVBoxLayout;

namespace MBSimGUI {

  class FunctionWidget;
  class ExtWidget;
  class ExtPhysicalVarWidget;
  class Function1ChoiceWidget;
  class Element;

  class GeneralizedForceLawWidget : public Widget {

    public:
      GeneralizedForceLawWidget() : forceFunc(0) {}
      virtual QString getType() const { return "GeneralizedForceLaw"; }
      virtual MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIM; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) { return element; }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      FunctionWidget *forceFunc;
  };

  class BilateralConstraintWidget : public GeneralizedForceLawWidget {

    public:
      BilateralConstraintWidget() {}
      virtual QString getType() const { return "BilateralConstraint"; }
  };

  class RegularizedBilateralConstraintWidget : public GeneralizedForceLawWidget {
    Q_OBJECT

      friend class RegularizedBilateralConstraint;

    public:
    RegularizedBilateralConstraintWidget(); 
    virtual QString getType() const { return "RegularizedBilateralConstraint"; }
    protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
    protected slots:
      void defineFunction(int);
  };

  class UnilateralConstraintWidget : public GeneralizedForceLawWidget {

    public:
      UnilateralConstraintWidget() {}
      virtual QString getType() const { return "UnilateralConstraint"; }
  };

  class RegularizedUnilateralConstraintWidget : public GeneralizedForceLawWidget {
    Q_OBJECT

      friend class RegularizedUnilateralConstraint;

    public:
    RegularizedUnilateralConstraintWidget(); 
    virtual QString getType() const { return "RegularizedUnilateralConstraint"; }
    protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
    protected slots:
      void defineFunction(int);
  };

  class GeneralizedImpactLawWidget : public Widget {

    public:
      GeneralizedImpactLawWidget() {}
      virtual QString getType() const { return "GeneralizedImpactLaw"; }
  };

  class BilateralImpactWidget : public GeneralizedImpactLawWidget {

    public:
      BilateralImpactWidget() {}
      virtual QString getType() const { return "BilateralImpact"; }
  };

  class UnilateralNewtonImpactWidget : public GeneralizedImpactLawWidget {

    friend class UnilateralNewtonImpact;

    public:
    UnilateralNewtonImpactWidget();
    virtual QString getType() const { return "UnilateralNewtonImpact"; }
    protected:
    ExtWidget* restitutionCoefficient;
  };

  class FrictionForceLawWidget : public Widget {

    public:
      FrictionForceLawWidget() : frictionForceFunc(0) {}
      virtual QString getType() const { return "FrictionForceLaw"; }
    protected:
      FunctionWidget *frictionForceFunc;
  };

  class PlanarCoulombFrictionWidget : public FrictionForceLawWidget {

    friend class PlanarCoulombFriction;

    public:
    PlanarCoulombFrictionWidget();
    virtual QString getType() const { return "PlanarCoulombFriction"; }
    protected:
    ExtWidget* frictionCoefficient;
  };

  class SpatialCoulombFrictionWidget : public FrictionForceLawWidget {

    friend class SpatialCoulombFriction;

    public:
    SpatialCoulombFrictionWidget();
    virtual QString getType() const { return "SpatialCoulombFriction"; }
    protected:
    ExtWidget* frictionCoefficient;
  };

  class PlanarStribeckFrictionWidget : public FrictionForceLawWidget {

    friend class PlanarStribeckFriction;

    public:
    PlanarStribeckFrictionWidget();
    virtual QString getType() const { return "PlanarStribeckFriction"; }
    protected:
    ExtWidget* frictionFunction;
  };

  class SpatialStribeckFrictionWidget : public FrictionForceLawWidget {

    friend class SpatialStribeckFriction;

    public:
    SpatialStribeckFrictionWidget();
    virtual QString getType() const { return "SpatialStribeckFriction"; }
    protected:
    ExtWidget* frictionFunction;
  };

  class RegularizedPlanarFrictionWidget : public FrictionForceLawWidget {
    Q_OBJECT

      friend class RegularizedPlanarFriction;

    public:
    RegularizedPlanarFrictionWidget(); 
    virtual QString getType() const { return "RegularizedPlanarFriction"; }
    protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
    protected slots:
      void defineFunction(int);
  };

  class RegularizedSpatialFrictionWidget : public FrictionForceLawWidget {
    Q_OBJECT

      friend class RegularizedSpatialFriction;

    public:
    RegularizedSpatialFrictionWidget(); 
    virtual QString getType() const { return "RegularizedSpatialFriction"; }
    protected:
    QVBoxLayout *layout;
    QComboBox *funcList;
    protected slots:
      void defineFunction(int);
  };

  class FrictionImpactLawWidget : public Widget {

    public:
      FrictionImpactLawWidget() {}
      virtual QString getType() const { return "FrictionImpactLaw"; }
  };

  class PlanarCoulombImpactWidget : public FrictionImpactLawWidget {

    friend class PlanarCoulombImpact;

    public:
    PlanarCoulombImpactWidget();
    virtual QString getType() const { return "PlanarCoulombImpact"; }
    protected:
    ExtWidget* frictionCoefficient;
  };

  class SpatialCoulombImpactWidget : public FrictionImpactLawWidget {

    friend class SpatialCoulombImpact;

    public:
    SpatialCoulombImpactWidget();
    virtual QString getType() const { return "SpatialCoulombImpact"; }
    protected:
    ExtWidget* frictionCoefficient;
  };

  class PlanarStribeckImpactWidget : public FrictionImpactLawWidget {

    friend class PlanarStribeckImpact;

    public:
    PlanarStribeckImpactWidget();
    virtual QString getType() const { return "PlanarStribeckImpact"; }
    protected:
    ExtWidget* frictionFunction;
  };

  class SpatialStribeckImpactWidget : public FrictionImpactLawWidget {

    friend class SpatialStribeckImpact;

    public:
    SpatialStribeckImpactWidget();
    virtual QString getType() const { return "SpatialStribeckImpact"; }
    protected:
    ExtWidget* frictionFunction;
  };

  class GeneralizedForceLawWidgetFactory : public WidgetFactory {
    public:
      GeneralizedForceLawWidgetFactory(Element *parent_);
      QWidget* createWidget(int i=0);
      static std::vector<QString> getNames();
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class GeneralizedImpactLawChoiceWidget : public Widget {
    Q_OBJECT

      friend class GeneralizedImpactLawChoiceProperty;

    public:
    GeneralizedImpactLawChoiceWidget();

    int getImpactLaw() {return comboBox->currentIndex();}

    protected slots:
      void defineImpactLaw(int);

    protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    GeneralizedImpactLawWidget *generalizedImpactLaw;
  };

  class FrictionForceLawChoiceWidget : public Widget {
    Q_OBJECT

      friend class FrictionForceLawChoiceProperty;

    public:
    FrictionForceLawChoiceWidget();

    int getForceLaw() {return comboBox->currentIndex();}

    protected slots:
      void defineFrictionLaw(int);

    protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    FrictionForceLawWidget *frictionForceLaw;
  };

  class FrictionImpactLawChoiceWidget : public Widget {
    Q_OBJECT

      friend class FrictionImpactLawChoiceProperty;

    public:
    FrictionImpactLawChoiceWidget();

    int getImpactLaw() {return comboBox->currentIndex();}

    protected slots:
      void defineFrictionImpactLaw(int);

    protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    FrictionImpactLawWidget *frictionImpactLaw;
  };

}

#endif

