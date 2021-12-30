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

#ifndef _FUNCTION_WIDGETS_H_
#define _FUNCTION_WIDGETS_H_

#include "function_widget.h"

class QVBoxLayout;
class QComboBox;
class QListWidget;
class QSpinBox;

namespace MBSimGUI {

  class ExtPhysicalVarWidget;
  class ExtWidget;
  class ListWidget;
  class ChoiceWidget;
  class ChoiceWidget;
  class Element;

  class IdentityFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(IdentityFunctionWidget, FunctionWidget, MBSIM%"IdentityFunction", "dummy");

    public:
      IdentityFunctionWidget() = default;
  };

  class ConstantFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(ConstantFunctionWidget, FunctionWidget, MBSIM%"ConstantFunction", "dummy");

    public:
      ConstantFunctionWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a0;
  };

  class LinearFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(LinearFunctionWidget, FunctionWidget, MBSIM%"LinearFunction", "dummy");

    public:
      LinearFunctionWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a0, *a1;
  };

  class QuadraticFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(QuadraticFunctionWidget, FunctionWidget, MBSIM%"QuadraticFunction", "dummy");

    public:
      QuadraticFunctionWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a0, *a1, *a2;
  };

  class PolynomFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(PolynomFunctionWidget, FunctionWidget, MBSIM%"PolynomFunction", "dummy");

    public:
      PolynomFunctionWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a;
  };

  class SinusoidalFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(SinusoidalFunctionWidget, FunctionWidget, MBSIM%"SinusoidalFunction", "dummy");

    public:
      SinusoidalFunctionWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a, *f, *p, *o;
  };

  class AbsoluteValueFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(AbsoluteValueFunctionWidget, FunctionWidget, MBSIM%"AbsoluteValueFunction", "dummy");

    public:
      AbsoluteValueFunctionWidget() = default;
  };

  class ModuloFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(ModuloFunctionWidget, FunctionWidget, MBSIM%"ModuloFunction", "dummy");

    public:
      ModuloFunctionWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *denom;
  };

  class BoundedFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(BoundedFunctionWidget, FunctionWidget, MBSIM%"BoundedFunction", "dummy");

    public:
      BoundedFunctionWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *lowerBound, *upperBound;
  };

  class SignumFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(SignumFunctionWidget, FunctionWidget, MBSIM%"SignumFunction", "dummy");

    public:
      SignumFunctionWidget() = default;
  };

  class VectorValuedFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(VectorValuedFunctionWidget, FunctionWidget, MBSIM%"VectorValuedFunction", "dummy");

    public:
      VectorValuedFunctionWidget(WidgetFactory *factory, int retDim, VarType retType);
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *functions;
  };

  class CompositeFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(CompositeFunctionWidget, FunctionWidget, MBSIM%"CompositeFunction", "dummy");

    public:
      CompositeFunctionWidget(WidgetFactory *factoryo1_, WidgetFactory *factoryo2_, WidgetFactory *factoryi_, int defo1=0, int defo2=0, int defi=0);
      int getArg1Size() const override;
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void updateWidget();
      void updateFunctionFactory();
      QString ext;
      ExtWidget *fo;
      ChoiceWidget *fi;
      WidgetFactory *factoryo1, *factoryo2, *factoryi;
  };

  class LimitedFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(LimitedFunctionWidget, FunctionWidget, MBSIM%"LimitedFunction", "dummy");

    public:
      LimitedFunctionWidget(WidgetFactory *factory);
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function, *limit;
  };

  class PiecewiseDefinedFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(PiecewiseDefinedFunctionWidget, FunctionWidget, MBSIM%"PiecewiseDefinedFunction", "dummy");

    public:
      PiecewiseDefinedFunctionWidget(WidgetFactory *factory);
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *functions, *shiftAbscissa, *shiftOrdinate;
  };

  class SymbolicFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(SymbolicFunctionWidget, FunctionWidget, MBSIM%"SymbolicFunction", "Symbolic function");

    public:
      SymbolicFunctionWidget(const QStringList &argName, const std::vector<int> &argDim, const std::vector<VarType> &argType, int retDim, VarType retType);
      int getArg1Size() const override;
      int getArg2Size() const override;
      void setArg1Size(int i) override;
      void setArg2Size(int i) override;
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *f;
      std::vector<ExtWidget*> argname, argdim;
  };

  class TabularFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(TabularFunctionWidget, FunctionWidget, MBSIM%"TabularFunction", "dummy");

    public:
      TabularFunctionWidget(int retDim, VarType retType);
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void choiceChanged();
      void updateWidget() override;
      ChoiceWidget* choice;
  };

  class TwoDimensionalTabularFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(TwoDimensionalTabularFunctionWidget, FunctionWidget, MBSIM%"TwoDimensionalTabularFunction", "Two dimensional tabular function");

    public:
      TwoDimensionalTabularFunctionWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void choiceChanged();
      void updateWidget() override;
      ChoiceWidget* choice;
  };

  class PiecewisePolynomFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(PiecewisePolynomFunctionWidget, FunctionWidget, MBSIM%"PiecewisePolynomFunction", "dummy");

    public:
      PiecewisePolynomFunctionWidget(int retDim, VarType retType);
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void choiceChanged();
      void updateWidget() override;
      ChoiceWidget* choice;
      ExtWidget *method;
  };

  class TwoDimensionalPiecewisePolynomFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(TwoDimensionalPiecewisePolynomFunctionWidget, FunctionWidget, MBSIM%"TwoDimensionalPiecewisePolynomFunction", "dummy");

    public:
      TwoDimensionalPiecewisePolynomFunctionWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void choiceChanged();
      void updateWidget() override;
      ChoiceWidget* choice;
      ExtWidget *method;
  };

  class FourierFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(FourierFunctionWidget, FunctionWidget, MBSIM%"FourierFunction", "dummy");

    public:
      FourierFunctionWidget();
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *f, *a0, *amplitudePhaseAngleForm;
      ChoiceWidget* choice;
  };

  class BidirectionalFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(BidirectionalFunctionWidget, FunctionWidget, MBSIM%"BidirectionalFunction", "dummy");

    public:
      BidirectionalFunctionWidget(Element *element, const QString &argName, int argDim, VarType argType, int retDim, VarType retType, QWidget *parent);
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *fn, *fp;
  };

  class ContinuedFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(ContinuedFunctionWidget, FunctionWidget, MBSIM%"ContinuedFunction", "dummy");

    public:
      ContinuedFunctionWidget(WidgetFactory *factoryf, WidgetFactory *factoryr);
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *f, *r;
  };

  class LinearSpringDamperForceWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(LinearSpringDamperForceWidget, FunctionWidget, MBSIM%"LinearSpringDamperForce", "dummy");

    public:
      LinearSpringDamperForceWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *c, *d;
  };

  class NonlinearSpringDamperForceWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(NonlinearSpringDamperForceWidget, FunctionWidget, MBSIM%"NonlinearSpringDamperForce", "dummy");

    public:
      NonlinearSpringDamperForceWidget(Element *element, QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *s, *sd;
  };

  class LinearElasticFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(LinearElasticFunctionWidget, FunctionWidget, MBSIM%"LinearElasticFunction", "dummy");

    public:
      LinearElasticFunctionWidget(bool varSize=false);
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      void updateWidget();
      ExtWidget *K, *D;
  };

  class LinearRegularizedBilateralConstraintWidget: public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(LinearRegularizedBilateralConstraintWidget, FunctionWidget, MBSIM%"LinearRegularizedBilateralConstraint", "Linear regularized bilateral constraint");

    public:
      LinearRegularizedBilateralConstraintWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *c, *d;
  };

  class LinearRegularizedUnilateralConstraintWidget: public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(LinearRegularizedUnilateralConstraintWidget, FunctionWidget, MBSIM%"LinearRegularizedUnilateralConstraint", "Linear regularized unilateral constraint");

    public:
      LinearRegularizedUnilateralConstraintWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *c, *d;
  };

  class LinearRegularizedCoulombFrictionWidget: public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(LinearRegularizedCoulombFrictionWidget, FunctionWidget, MBSIM%"LinearRegularizedCoulombFriction", "Linear regularized Coulomb friction");

    public:
      LinearRegularizedCoulombFrictionWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *gd, *mu;
  };

  class LinearRegularizedStribeckFrictionWidget: public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(LinearRegularizedStribeckFrictionWidget, FunctionWidget, MBSIM%"LinearRegularizedStribeckFriction", "Linear regularized Stribeck friction");

    public:
      LinearRegularizedStribeckFrictionWidget(Element *element, QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *gd, *frictionFunction;
  };

  class SignalFunctionWidget: public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(SignalFunctionWidget, FunctionWidget, MBSIMCONTROL%"SignalFunction", "dummy");

    public:
      SignalFunctionWidget(Element *element, QWidget *parent);
      ~SignalFunctionWidget() override;
      void updateWidget() override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *sRef;
      Element *dummy;
  };

  class PolarContourFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(PolarContourFunctionWidget, FunctionWidget, MBSIM%"PolarContourFunction", "dummy");

    public:
      PolarContourFunctionWidget(Element *element, QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radiusFunction;
  };

  class GravityFunctionWidget : public FunctionWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(GravityFunctionWidget, FunctionWidget, MBSIMPHYSICS%"GravityFunction", "dummy");

    public:
      GravityFunctionWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *standardGravity, *meanRadius;
  };

}

#endif
