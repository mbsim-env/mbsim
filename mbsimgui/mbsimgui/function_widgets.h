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
  };

  class ConstantFunctionWidget : public FunctionWidget {

    public:
      ConstantFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ConstantFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a0;
  };

  class LinearFunctionWidget : public FunctionWidget {

    public:
      LinearFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a0, *a1;
  };

  class QuadraticFunctionWidget : public FunctionWidget {

    public:
      QuadraticFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"QuadraticFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a0, *a1, *a2;
  };

  class PolynomFunctionWidget : public FunctionWidget {

    public:
      PolynomFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PolynomFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a;
  };

  class SinusoidalFunctionWidget : public FunctionWidget {

    public:
      SinusoidalFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SinusoidalFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a, *f, *p, *o;
  };

  class AbsoluteValueFunctionWidget : public FunctionWidget {

    public:
      AbsoluteValueFunctionWidget() = default;
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"AbsoluteValueFunction"; }
  };

  class ModuloFunctionWidget : public FunctionWidget {

    public:
      ModuloFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ModuloFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *denom;
  };

  class BoundedFunctionWidget : public FunctionWidget {

    public:
      BoundedFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"BoundedFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *lowerBound, *upperBound;
  };

  class SignumFunctionWidget : public FunctionWidget {

    public:
      SignumFunctionWidget() = default;
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SignumFunction"; }
  };

  class VectorValuedFunctionWidget : public FunctionWidget {

    public:
      VectorValuedFunctionWidget(WidgetFactory *factory, int retDim, VarType retType);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"VectorValuedFunction"; }
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
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *c, *d;
  };

  class NonlinearSpringDamperForceWidget : public FunctionWidget {

    public:
      NonlinearSpringDamperForceWidget(Element *element, QWidget *parent);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"NonlinearSpringDamperForce"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *s, *sd;
  };

  class LinearElasticFunctionWidget : public FunctionWidget {

    public:
      LinearElasticFunctionWidget(bool varSize=false);
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearElasticFunction"; }
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
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *c, *d;
  };

  class LinearRegularizedUnilateralConstraintWidget: public FunctionWidget {

    public:
      LinearRegularizedUnilateralConstraintWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearRegularizedUnilateralConstraint"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *c, *d;
  };

  class LinearRegularizedCoulombFrictionWidget: public FunctionWidget {

    public:
      LinearRegularizedCoulombFrictionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearRegularizedCoulombFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *gd, *mu;
  };

  class LinearRegularizedStribeckFrictionWidget: public FunctionWidget {

    public:
      LinearRegularizedStribeckFrictionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LinearRegularizedStribeckFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *gd, *frictionFunction;
  };

  class SignalFunctionWidget: public FunctionWidget {

    public:
      SignalFunctionWidget(Element *element, QWidget *parent);
      ~SignalFunctionWidget() override;
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"SignalFunction"; }
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
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radiusFunction;
  };

  class GravityFunctionWidget : public FunctionWidget {

    public:
      GravityFunctionWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIMPHYSICS%"GravityFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *standardGravity, *meanRadius;
  };

}

#endif
