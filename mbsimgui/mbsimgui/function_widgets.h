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
  class ChoiceWidget2;
  class Element;

  class IdentityFunctionWidget : public FunctionWidget {

    public:
      IdentityFunctionWidget() = default;
      QString getType() const override { return "IdentityFunction"; }
  };

  class ConstantFunctionWidget : public FunctionWidget {

    public:
      ConstantFunctionWidget();
      QString getType() const override { return "ConstantFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a0;
  };

  class LinearFunctionWidget : public FunctionWidget {

    public:
      LinearFunctionWidget();
      QString getType() const override { return "LinearFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a0, *a1;
  };

  class QuadraticFunctionWidget : public FunctionWidget {

    public:
      QuadraticFunctionWidget();
      QString getType() const override { return "QuadraticFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a0, *a1, *a2;
  };

  class PolynomFunctionWidget : public FunctionWidget {

    public:
      PolynomFunctionWidget();
      QString getType() const override { return "PolynomFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a;
  };

  class SinusoidalFunctionWidget : public FunctionWidget {

    public:
      SinusoidalFunctionWidget();
      QString getType() const override { return "SinusoidalFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *a, *f, *p, *o;
  };

  class AbsoluteValueFunctionWidget : public FunctionWidget {

    public:
      AbsoluteValueFunctionWidget() = default;
      QString getType() const override { return "AbsoluteValueFunction"; }
  };

  class ModuloFunctionWidget : public FunctionWidget {

    public:
      ModuloFunctionWidget();
      QString getType() const override { return "ModuloFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *denom;
  };

  class BoundedFunctionWidget : public FunctionWidget {

    public:
      BoundedFunctionWidget();
      QString getType() const override { return "BoundedFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *lowerBound, *upperBound;
  };

  class SignumFunctionWidget : public FunctionWidget {

    public:
      SignumFunctionWidget() = default;
      QString getType() const override { return "SignumFunction"; }
  };

  class VectorValuedFunctionWidget : public FunctionWidget {

    public:
      VectorValuedFunctionWidget(WidgetFactory *factory, int retDim, VarType retType);
      QString getType() const override { return "VectorValuedFunction"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *functions;
  };

  class CompositeFunctionWidget : public FunctionWidget {

    public:
      CompositeFunctionWidget(WidgetFactory *factoryo1_, WidgetFactory *factoryo2_, WidgetFactory *factoryi_, int defo1=0, int defo2=0, int defi=0);
      QString getType() const override { return "CompositeFunction"; }
      int getArg1Size() const override;
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void updateWidget();
      void updateFunctionFactory();
      QString ext;
      ExtWidget *fo;
      ChoiceWidget2 *fi;
      WidgetFactory *factoryo1, *factoryo2, *factoryi;
  };

  class LimitedFunctionWidget : public FunctionWidget {

    public:
      LimitedFunctionWidget(WidgetFactory *factory);
      QString getType() const override { return "LimitedFunction"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function, *limit;
  };

  class PiecewiseDefinedFunctionWidget : public FunctionWidget {

    public:
      PiecewiseDefinedFunctionWidget(WidgetFactory *factory);
      QString getType() const override { return "PiecewiseDefinedFunction"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *functions, *shiftAbscissa, *shiftOrdinate;
  };

  class SymbolicFunctionWidget : public FunctionWidget {

    public:
      SymbolicFunctionWidget(const QStringList &argName, const std::vector<int> &argDim, const std::vector<VarType> &argType, int retDim, VarType retType);
      QString getType() const override { return "SymbolicFunction"; }
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
      QString getType() const override { return "TabularFunction"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void choiceChanged();
      void updateWidget() override;
      ChoiceWidget2* choice;
  };

  class TwoDimensionalTabularFunctionWidget : public FunctionWidget {

    public:
      TwoDimensionalTabularFunctionWidget();
      QString getType() const override { return "TwoDimensionalTabularFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void choiceChanged();
      void updateWidget() override;
      ChoiceWidget2* choice;
  };

  class PiecewisePolynomFunctionWidget : public FunctionWidget {

    public:
      PiecewisePolynomFunctionWidget(int retDim, VarType retType);
      QString getType() const override { return "PiecewisePolynomFunction"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void choiceChanged();
      void updateWidget() override;
      ChoiceWidget2* choice;
      ExtWidget *method;
  };

  class TwoDimensionalPiecewisePolynomFunctionWidget : public FunctionWidget {

    public:
      TwoDimensionalPiecewisePolynomFunctionWidget();
      QString getType() const override { return "TwoDimensionalPiecewisePolynomFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void choiceChanged();
      void updateWidget() override;
      ChoiceWidget2* choice;
      ExtWidget *method;
  };

  class FourierFunctionWidget : public FunctionWidget {

    public:
      FourierFunctionWidget();
      QString getType() const override { return "FourierFunction"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *f, *a0, *amplitudePhaseAngleForm;
      ChoiceWidget2* choice;
  };

  class BidirectionalFunctionWidget : public FunctionWidget {

    public:
      BidirectionalFunctionWidget(Element *element, const QString &argName, int argDim, VarType argType, int retDim, VarType retType, QWidget *parent);
      QString getType() const override { return "BidirectionalFunction"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *fn, *fp;
  };

  class ContinuedFunctionWidget : public FunctionWidget {

    public:
      ContinuedFunctionWidget(WidgetFactory *factoryf, WidgetFactory *factoryr);
      QString getType() const override { return "ContinuedFunction"; }
      void resize_(int m, int n) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *f, *r;
  };

  class LinearSpringDamperForceWidget : public FunctionWidget {

    public:
      LinearSpringDamperForceWidget();
      QString getType() const override { return "LinearSpringDamperForce"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *c, *d;
  };

  class NonlinearSpringDamperForceWidget : public FunctionWidget {

    public:
      NonlinearSpringDamperForceWidget(Element *element, QWidget *parent);
      QString getType() const override { return "NonlinearSpringDamperForce"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *s, *sd;
  };

  class LinearElasticFunctionWidget : public FunctionWidget {

    public:
      LinearElasticFunctionWidget(bool varSize=false);
      QString getType() const override { return "LinearElasticFunction"; }
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
      QString getType() const override { return "LinearRegularizedBilateralConstraint"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *c, *d;
  };

  class LinearRegularizedUnilateralConstraintWidget: public FunctionWidget {

    public:
      LinearRegularizedUnilateralConstraintWidget();
      QString getType() const override { return "LinearRegularizedUnilateralConstraint"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *c, *d;
  };

  class LinearRegularizedCoulombFrictionWidget: public FunctionWidget {

    public:
      LinearRegularizedCoulombFrictionWidget();
      QString getType() const override { return "LinearRegularizedCoulombFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *gd, *mu;
  };

  class LinearRegularizedStribeckFrictionWidget: public FunctionWidget {

    public:
      LinearRegularizedStribeckFrictionWidget();
      QString getType() const override { return "LinearRegularizedStribeckFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    private:
      ExtWidget *gd, *mu;
  };

  class SignalFunctionWidget: public FunctionWidget {

    public:
      SignalFunctionWidget(Element *element, QWidget *parent);
      ~SignalFunctionWidget() override;
      QString getType() const override { return "SignalFunction"; }
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
      QString getType() const override { return "PolarContourFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radiusFunction;
  };

  class GravityFunctionWidget : public FunctionWidget {

    public:
      GravityFunctionWidget();
      QString getType() const override { return "GravityFunction"; }
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMPHYSICS; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *standardGravity, *meanRadius;
  };

}

#endif
