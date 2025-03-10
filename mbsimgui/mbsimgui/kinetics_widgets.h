/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
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
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedForceLawWidget, Widget, MBSIM%"GeneralizedForceLaw", "Generalized force law");

    public:
      GeneralizedForceLawWidget(Element *parentElement=nullptr);
      virtual ~GeneralizedForceLawWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *forceFunc{nullptr};
      Element *generalizedForceLaw;
  };

  class BilateralConstraintWidget : public GeneralizedForceLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(BilateralConstraintWidget, GeneralizedForceLawWidget, MBSIM%"BilateralConstraint", "Bilateral constraint");

    public:
      BilateralConstraintWidget(Element *parentElement, QWidget *parent) : GeneralizedForceLawWidget(parentElement) { }
  };

  class RegularizedBilateralConstraintWidget : public GeneralizedForceLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RegularizedBilateralConstraintWidget, GeneralizedForceLawWidget, MBSIM%"RegularizedBilateralConstraint", "Regularized bilateral constraint");

    public:
      RegularizedBilateralConstraintWidget(Element *parentElement, QWidget *parent);
  };

  class UnilateralConstraintWidget : public GeneralizedForceLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(UnilateralConstraintWidget, GeneralizedForceLawWidget, MBSIM%"UnilateralConstraint", "Unilateral constraint");

    public:
      UnilateralConstraintWidget(Element *parentElement, QWidget *parent) : GeneralizedForceLawWidget(parentElement) { }
  };

  class RegularizedUnilateralConstraintWidget : public GeneralizedForceLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RegularizedUnilateralConstraintWidget, GeneralizedForceLawWidget, MBSIM%"RegularizedUnilateralConstraint", "Regularized unilateral constraint");

    public:
      RegularizedUnilateralConstraintWidget(Element *parentElement, QWidget *parent);
  };

  class GeneralizedImpactLawWidget : public Widget {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedImpactLawWidget, Widget, MBSIM%"GeneralizedImpactLaw", "Generalized impact law");

    public:
      GeneralizedImpactLawWidget(Element *parentElement=nullptr);
      virtual ~GeneralizedImpactLawWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      Element *generalizedImpactLaw;
  };

  class BilateralImpactWidget : public GeneralizedImpactLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(BilateralImpactWidget, GeneralizedImpactLawWidget, MBSIM%"BilateralImpact", "Bilateral impact");

    public:
      BilateralImpactWidget(Element *parentElement, QWidget *parent) : GeneralizedImpactLawWidget(parentElement) { }
  };

  class UnilateralNewtonImpactWidget : public GeneralizedImpactLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(UnilateralNewtonImpactWidget, GeneralizedImpactLawWidget, MBSIM%"UnilateralNewtonImpact", "Unilateral Newton impact");

    public:
      UnilateralNewtonImpactWidget(Element *parentElement, QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *restitutionCoefficient;
  };

  class FrictionForceLawWidget : public Widget {
    MBSIMGUI_OBJECTFACTORY_CLASS(FrictionForceLawWidget, Widget, MBSIM%"FrictionForceLaw", "Friction force law");

    public:
      FrictionForceLawWidget(Element *parentElement=nullptr);
      virtual ~FrictionForceLawWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frictionForceFunc{nullptr};
      Element *frictionForceLaw;
  };

  class PlanarCoulombFrictionWidget : public FrictionForceLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(PlanarCoulombFrictionWidget, FrictionForceLawWidget, MBSIM%"PlanarCoulombFriction", "Planar Coulomb friction");

    public:
      PlanarCoulombFrictionWidget(Element *parentElement, QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frictionCoefficient;
  };

  class SpatialCoulombFrictionWidget : public FrictionForceLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(SpatialCoulombFrictionWidget, FrictionForceLawWidget, MBSIM%"SpatialCoulombFriction", "Spatial Coulomb friction");

    public:
      SpatialCoulombFrictionWidget(Element *parentElement, QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frictionCoefficient;
  };

  class PlanarStribeckFrictionWidget : public FrictionForceLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(PlanarStribeckFrictionWidget, FrictionForceLawWidget, MBSIM%"PlanarStribeckFriction", "Planar Stribeck friction");

    public:
      PlanarStribeckFrictionWidget(Element *parentElement, QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frictionFunction;
  };

  class SpatialStribeckFrictionWidget : public FrictionForceLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(SpatialStribeckFrictionWidget, FrictionForceLawWidget, MBSIM%"SpatialStribeckFriction", "Spatial Stribeck friction");

    public:
      SpatialStribeckFrictionWidget(Element *parentElement, QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frictionFunction;
  };

  class RegularizedPlanarFrictionWidget : public FrictionForceLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RegularizedPlanarFrictionWidget, FrictionForceLawWidget, MBSIM%"RegularizedPlanarFriction", "Regularized planar friction");
    public:
      RegularizedPlanarFrictionWidget(Element *parentElement, QWidget *parent);
  };

  class RegularizedSpatialFrictionWidget : public FrictionForceLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(RegularizedSpatialFrictionWidget, FrictionForceLawWidget, MBSIM%"RegularizedSpatialFriction", "Regularized spatial friction");
    public:
      RegularizedSpatialFrictionWidget(Element *parentElement, QWidget *parent);
  };

  class FrictionImpactLawWidget : public Widget {
    MBSIMGUI_OBJECTFACTORY_CLASS(FrictionImpactLawWidget, Widget, MBSIM%"FrictionImpactLaw", "Friction impact law");

    public:
      FrictionImpactLawWidget(Element *parentElement=nullptr);
      virtual ~FrictionImpactLawWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      Element *frictionImpactLaw;
  };

  class PlanarCoulombImpactWidget : public FrictionImpactLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(PlanarCoulombImpactWidget, FrictionImpactLawWidget, MBSIM%"PlanarCoulombImpact", "Planar Coulomb impact");

    public:
      PlanarCoulombImpactWidget(Element *parentElement, QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frictionCoefficient;
  };

  class SpatialCoulombImpactWidget : public FrictionImpactLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(SpatialCoulombImpactWidget, FrictionImpactLawWidget, MBSIM%"SpatialCoulombImpact", "Spatial Coulomb impact");

    public:
      SpatialCoulombImpactWidget(Element *parentElement, QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frictionCoefficient;
  };

  class PlanarStribeckImpactWidget : public FrictionImpactLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(PlanarStribeckImpactWidget, FrictionImpactLawWidget, MBSIM%"PlanarStribeckImpact", "Planar Stribeck impact");

    public:
      PlanarStribeckImpactWidget(Element *parentElement, QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frictionFunction;
  };

  class SpatialStribeckImpactWidget : public FrictionImpactLawWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(SpatialStribeckImpactWidget, FrictionImpactLawWidget, MBSIM%"SpatialStribeckImpact", "Spatial Stribeck impact");

    public:
      SpatialStribeckImpactWidget(Element *parentElement, QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frictionFunction;
  };

  class RegularizedBilateralConstraintFunctionFactory : public WidgetFactory {
    public:
      RegularizedBilateralConstraintFunctionFactory();
      Widget* createWidget(int i=0) override;
      static std::vector<QString> getNames();
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getFallbackIndex() const override { return 1; }
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
      int getFallbackIndex() const override { return 1; }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class FrictionFunctionFactory : public WidgetFactory {
    public:
      FrictionFunctionFactory(Element *element_, int nd_, QWidget *parent_);
      Widget* createWidget(int i=0) override;
      static std::vector<QString> getNames();
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getFallbackIndex() const override { return 3; }
    protected:
      Element *element;
      int nd;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QWidget *parent;
  };

  class TyreModelWidget : public Widget {
    MBSIMGUI_OBJECTFACTORY_CLASS(TyreModelWidget, Widget, MBSIM%"TyreModel", "Tyre model");

    public:
      TyreModelWidget()  = default;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class LinearTyreModelWidget : public TyreModelWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(LinearTyreModelWidget, TyreModelWidget, MBSIM%"LinearTyreModel", "Linear tyre model");

    public:
      LinearTyreModelWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *cz, *dz, *cka, *cal, *cga, *cMzga, *t, *sfFLo, *sfFLa, *sfM;
  };

  class MagicFormulaSharpWidget : public TyreModelWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(MagicFormulaSharpWidget, TyreModelWidget, MBSIM%"MagicFormulaSharp", "Magic Formula Sharp");

    public:
      MagicFormulaSharpWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *cz, *dz, *Fz0, *R0, *pKy1, *pKy2, *pKy3, *pKy4, *pKy5, *pKy6, *pKy7, *pDx1, *pDx2, *pEx1, *pEx2, *pEx3, *pEx4, *pKx1, *pKx2, *pKx3, *Cx, *Cy, *rBx1, *rBx2, *Cxal, *pDy1, *pDy2, *pDy3, *pEy1, *pEy2, *pEy4, *Cga, *Ega, *rBy1, *rBy2, *rBy3, *Cyka, *qHz3, *qHz4, *qBz1, *qBz2, *qBz5, *qBz6, *qBz9, *qBz10, *qDz1, *qDz2, *qDz3, *qDz4, *qDz8, *qDz9, *qDz10, *qDz11, *qEz1, *qEz2, *qEz5, *Ct, *c1Rel, *c2Rel, *c3Rel, *sfKyga, *sfFLo, *sfFLa, *sfM;
  };

  class MagicFormula62Widget : public TyreModelWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(MagicFormula62Widget, TyreModelWidget, MBSIM%"MagicFormula62", "Magic Formula 62");

    public:
      MagicFormula62Widget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void inputFileChanged(const QString &fileName);
      ExtWidget *inputDataFile, *tyreSide, *mck, *cpt, *ts, *six, *siy, *p, *cz, *dz, *rtw, *sfFx, *sfFy, *sfMx, *sfMy, *sfMz, *sfs, *sfmux, *sfmuy, *sfkx, *sfky, *sfkg, *sfkm;
  };

}

#endif
