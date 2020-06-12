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

    public:
      IdentityFunctionWidget() = default;
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"IdentityFunction"; }
      QString getType() const override { return "Identity function"; }
  };

  class ConstantFunctionWidget : public FunctionWidget {

    public:
      ConstantFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ConstantFunction"; }
      QString getType() const override { return "Constant function"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a0;
  };

  class LinearFunctionWidget : public FunctionWidget {

    public:
      LinearFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearFunction"; }
      QString getType() const override { return "Linear function"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a0, *a1;
  };

  class QuadraticFunctionWidget : public FunctionWidget {

    public:
      QuadraticFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"QuadraticFunction"; }
      QString getType() const override { return "Quadratic function"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a0, *a1, *a2;
  };

  class PolynomFunctionWidget : public FunctionWidget {

    public:
      PolynomFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PolynomFunction"; }
      QString getType() const override { return "Polynom function"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a;
  };

  class SinusoidalFunctionWidget : public FunctionWidget {

    public:
      SinusoidalFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SinusoidalFunction"; }
      QString getType() const override { return "Sinusoidal function"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a, *f, *p, *o;
  };

  class AbsoluteValueFunctionWidget : public FunctionWidget {

    public:
      AbsoluteValueFunctionWidget() = default;
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"AbsoluteValueFunction"; }
      QString getType() const override { return "Absolute value function"; }
  };

  class ModuloFunctionWidget : public FunctionWidget {

    public:
      ModuloFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ModuloFunction"; }
      QString getType() const override { return "Modulo function"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *denom;
  };

  class BoundedFunctionWidget : public FunctionWidget {

    public:
      BoundedFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"BoundedFunction"; }
      QString getType() const override { return "Bounded function"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *lowerBound, *upperBound;
  };

  class SignumFunctionWidget : public FunctionWidget {

    public:
      SignumFunctionWidget() = default;
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SignumFunction"; }
      QString getType() const override { return "Signum function"; }
  };

  class VectorValuedFunctionWidget : public FunctionWidget {

    public:
      VectorValuedFunctionWidget(WidgetFactory *factory, int retDim, VarType retType);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"VectorValuedFunction"; }
      QString getType() const override { return "Vector valued function"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *functions;
  };

  class CompositeFunctionWidget : public FunctionWidget {

    public:
      CompositeFunctionWidget(WidgetFactory *factoryo1_, WidgetFactory *factoryo2_, WidgetFactory *factoryi_, int defo1=0, int defo2=0, int defi=0);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"CompositeFunction"; }
      QString getType() const override { return "Composite function"; }
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

    public:
      LimitedFunctionWidget(WidgetFactory *factory);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LimitedFunction"; }
      QString getType() const override { return "Limited function"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function, *limit;
  };

  class PiecewiseDefinedFunctionWidget : public FunctionWidget {

    public:
      PiecewiseDefinedFunctionWidget(WidgetFactory *factory);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PiecewiseDefinedFunction"; }
      QString getType() const override { return "Piecewise defined function"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *functions, *shiftAbscissa, *shiftOrdinate;
  };

  class SymbolicFunctionWidget : public FunctionWidget {

    public:
      SymbolicFunctionWidget(const QStringList &argName, const std::vector<int> &argDim, const std::vector<VarType> &argType, int retDim, VarType retType);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SymbolicFunction"; }
      QString getType() const override { return "Symbolic function"; }
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

    public:
      TabularFunctionWidget(int retDim, VarType retType);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TabularFunction"; }
      QString getType() const override { return "Tabular function"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void choiceChanged();
      void updateWidget() override;
      ChoiceWidget* choice;
  };

  class TwoDimensionalTabularFunctionWidget : public FunctionWidget {

    public:
      TwoDimensionalTabularFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TwoDimensionalTabularFunction"; }
      QString getType() const override { return "Two dimensional tabular function"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void choiceChanged();
      void updateWidget() override;
      ChoiceWidget* choice;
  };

  class PiecewisePolynomFunctionWidget : public FunctionWidget {

    public:
      PiecewisePolynomFunctionWidget(int retDim, VarType retType);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PiecewisePolynomFunction"; }
      QString getType() const override { return "Piecewise polynom function"; }
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

    public:
      TwoDimensionalPiecewisePolynomFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TwoDimensionalPiecewisePolynomFunction"; }
      QString getType() const override { return "Two dimensional piecewise polynom function"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void choiceChanged();
      void updateWidget() override;
      ChoiceWidget* choice;
      ExtWidget *method;
  };

  class FourierFunctionWidget : public FunctionWidget {

    public:
      FourierFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"FourierFunction"; }
      QString getType() const override { return "Fourier function"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *f, *a0, *amplitudePhaseAngleForm;
      ChoiceWidget* choice;
  };

  class BidirectionalFunctionWidget : public FunctionWidget {

    public:
      BidirectionalFunctionWidget(Element *element, const QString &argName, int argDim, VarType argType, int retDim, VarType retType, QWidget *parent);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"BidirectionalFunction"; }
      QString getType() const override { return "Bidirectional function"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *fn, *fp;
  };

  class ContinuedFunctionWidget : public FunctionWidget {

    public:
      ContinuedFunctionWidget(WidgetFactory *factoryf, WidgetFactory *factoryr);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ContinuedFunction"; }
      QString getType() const override { return "Continued function"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *f, *r;
  };

  class LinearSpringDamperForceWidget : public FunctionWidget {

    public:
      LinearSpringDamperForceWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearSpringDamperForce"; }
      QString getType() const override { return "Linear spring damper force"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *c, *d;
  };

  class NonlinearSpringDamperForceWidget : public FunctionWidget {

    public:
      NonlinearSpringDamperForceWidget(Element *element, QWidget *parent);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"NonlinearSpringDamperForce"; }
      QString getType() const override { return "Nonlinear spring damper force"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *s, *sd;
  };

  class LinearElasticFunctionWidget : public FunctionWidget {

    public:
      LinearElasticFunctionWidget(bool varSize=false);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearElasticFunction"; }
      QString getType() const override { return "Linear elastic function"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      void updateWidget();
      ExtWidget *K, *D;
  };

  class LinearRegularizedBilateralConstraintWidget: public FunctionWidget {

    public:
      LinearRegularizedBilateralConstraintWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearRegularizedBilateralConstraint"; }
      QString getType() const override { return "Linear regularized bilateral constraint"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *c, *d;
  };

  class LinearRegularizedUnilateralConstraintWidget: public FunctionWidget {

    public:
      LinearRegularizedUnilateralConstraintWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearRegularizedUnilateralConstraint"; }
      QString getType() const override { return "Linear regularized unilaterali constraint"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *c, *d;
  };

  class LinearRegularizedCoulombFrictionWidget: public FunctionWidget {

    public:
      LinearRegularizedCoulombFrictionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearRegularizedCoulombFriction"; }
      QString getType() const override { return "Linear regularized coulomb friction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *gd, *mu;
  };

  class LinearRegularizedStribeckFrictionWidget: public FunctionWidget {

    public:
      LinearRegularizedStribeckFrictionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearRegularizedStribeckFriction"; }
      QString getType() const override { return "Linear regularized stribeck friction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *gd, *mu;
  };

  class SignalFunctionWidget: public FunctionWidget {

    public:
      SignalFunctionWidget(Element *element, QWidget *parent);
      ~SignalFunctionWidget() override;
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"SignalFunction"; }
      QString getType() const override { return "Signal function"; }
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMCONTROL; }
      void updateWidget() override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *sRef;
      Element *dummy;
  };

  class PolarContourFunctionWidget : public FunctionWidget {

    public:
      PolarContourFunctionWidget(Element *element, QWidget *parent);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PolarContourFunction"; }
      QString getType() const override { return "Polar contour function"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radiusFunction;
  };

  class GravityFunctionWidget : public FunctionWidget {

    public:
      GravityFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIMPHYSICS%"GravityFunction"; }
      QString getType() const override { return "Gravity function"; }
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMPHYSICS; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *standardGravity, *meanRadius;
  };

}

#endif
