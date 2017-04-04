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
      IdentityFunctionWidget(int m=1) { }
      QString getType() const { return "IdentityFunction"; }
  };

  class ConstantFunctionWidget : public FunctionWidget {

    public:
      ConstantFunctionWidget(int m=1);
      QString getType() const { return "ConstantFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *a0;
  };

  class LinearFunctionWidget : public FunctionWidget {

    public:
      LinearFunctionWidget(int m=1);
      QString getType() const { return "LinearFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *a0, *a1;
  };

  class QuadraticFunctionWidget : public FunctionWidget {

    public:
      QuadraticFunctionWidget(int m=1);
      QString getType() const { return "QuadraticFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *a0, *a1, *a2;
  };

  class PolynomFunctionWidget : public FunctionWidget {

    public:
      PolynomFunctionWidget(int m=1);
      QString getType() const { return "PolynomFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *a;
  };

  class SinusoidalFunctionWidget : public FunctionWidget {

    public:
      SinusoidalFunctionWidget(int m=1);
      QString getType() const { return "SinusoidalFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *a, *f, *p, *o;
  };

  class AbsoluteValueFunctionWidget : public FunctionWidget {

    public:
      AbsoluteValueFunctionWidget(int m=0) { }
      QString getType() const { return "AbsoluteValueFunction"; }
  };

  class ModuloFunctionWidget : public FunctionWidget {

    public:
      ModuloFunctionWidget(int m=0);
      QString getType() const { return "ModuloFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *denom;
  };

  class BoundedFunctionWidget : public FunctionWidget {

    public:
      BoundedFunctionWidget(int m=0);
      QString getType() const { return "BoundedFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *lowerBound, *upperBound;
  };

  class SignumFunctionWidget : public FunctionWidget {

    public:
      SignumFunctionWidget(int m=0) { }
      QString getType() const { return "SignumFunction"; }
  };

  class VectorValuedFunctionWidget : public FunctionWidget {

    public:
      VectorValuedFunctionWidget(Element *parent, int m=0, bool fixedSize=false);
      QString getType() const { return "VectorValuedFunction"; }
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *functions;
  };

  class CompositeFunctionWidget : public FunctionWidget {
    Q_OBJECT

    public:
      CompositeFunctionWidget(WidgetFactory *factoryo, WidgetFactory *factoryi);
      QString getType() const { return "CompositeFunction"; }
      int getArg1Size() const;
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      QString ext;
      ExtWidget *fo, *fi;
    public slots:
      void resizeVariables();
  };

  class LimitedFunctionWidget : public FunctionWidget {

    public:
      LimitedFunctionWidget(Element *parent);
      QString getType() const { return "LimitedFunction"; }
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *function, *limit;
  };

  class PiecewiseDefinedFunctionWidget : public FunctionWidget {

    public:
      PiecewiseDefinedFunctionWidget(Element *parent, int n=0);
      QString getType() const { return "PiecewiseDefinedFunction"; }
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *functions, *shiftAbscissa, *shiftOrdinate;
  };

  class SymbolicFunctionWidget : public FunctionWidget {
    Q_OBJECT

    public:
      SymbolicFunctionWidget(const QStringList &var, int m, int max);
      QString getType() const { return "SymbolicFunction"; }
      int getArg1Size() const;
      int getArg2Size() const;
      void setArg1Size(int i);
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *f;
      std::vector<ExtWidget*> argname, argdim;
    signals:
      void arg1SizeChanged(int i);
  };

  class TabularFunctionWidget : public FunctionWidget {

    public:
      TabularFunctionWidget(int n);
      QString getType() const { return "TabularFunction"; }
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ChoiceWidget2* choice;
  };

  class TwoDimensionalTabularFunctionWidget : public FunctionWidget {

    public:
      TwoDimensionalTabularFunctionWidget(int n);
      QString getType() const { return "TwoDimensionalTabularFunction"; }
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ChoiceWidget2* choice;
  };

  class PiecewisePolynomFunctionWidget : public FunctionWidget {

    public:
      PiecewisePolynomFunctionWidget(int n);
      QString getType() const { return "PiecewisePolynomFunction"; }
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ChoiceWidget2* choice;
      ExtWidget *method;
  };

  class TwoDimensionalPiecewisePolynomFunctionWidget : public FunctionWidget {

    public:
      TwoDimensionalPiecewisePolynomFunctionWidget(int n);
      QString getType() const { return "TwoDimensionalPiecewisePolynomFunction"; }
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ChoiceWidget2* choice;
      ExtWidget *method;
  };

  class FourierFunctionWidget : public FunctionWidget {

    public:
      FourierFunctionWidget(int n);
      QString getType() const { return "FourierFunction"; }
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *f, *a0, *amplitudePhaseAngleForm;
      ChoiceWidget2* choice;
  };

  class BidirectionalFunctionWidget : public FunctionWidget {
    Q_OBJECT

    public:
      BidirectionalFunctionWidget();
      QString getType() const { return "BidirectionalFunction"; }
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *fn, *fp;
  };

  class ContinuedFunctionWidget : public FunctionWidget {
    Q_OBJECT

    public:
      ContinuedFunctionWidget(WidgetFactory *factoryf, WidgetFactory *factoryr);
      QString getType() const { return "ContinuedFunction"; }
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *f, *r;
  };

  class LinearSpringDamperForceWidget : public FunctionWidget {

    public:
      LinearSpringDamperForceWidget();
      QString getType() const { return "LinearSpringDamperForce"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *c, *d;
  };

  class NonlinearSpringDamperForceWidget : public FunctionWidget {

    public:
      NonlinearSpringDamperForceWidget(Element *parent);
      QString getType() const { return "NonlinearSpringDamperForce"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *s, *sd;
  };

  class LinearElasticFunctionWidget : public FunctionWidget {

    public:
      LinearElasticFunctionWidget();
      QString getType() const { return "LinearElasticFunction"; }
      void resize_(int m, int n);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *K, *D;
  };

  class LinearRegularizedBilateralConstraintWidget: public FunctionWidget {

    public:
      LinearRegularizedBilateralConstraintWidget();
      QString getType() const { return "LinearRegularizedBilateralConstraint"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    private:
      ExtWidget *c, *d;
  };

  class LinearRegularizedUnilateralConstraintWidget: public FunctionWidget {

    public:
      LinearRegularizedUnilateralConstraintWidget();
      QString getType() const { return "LinearRegularizedUnilateralConstraint"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    private:
      ExtWidget *c, *d;
  };

  class LinearRegularizedCoulombFrictionWidget: public FunctionWidget {

    public:
      LinearRegularizedCoulombFrictionWidget();
      QString getType() const { return "LinearRegularizedCoulombFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    private:
      ExtWidget *gd, *mu;
  };

  class LinearRegularizedStribeckFrictionWidget: public FunctionWidget {

    public:
      LinearRegularizedStribeckFrictionWidget();
      QString getType() const { return "LinearRegularizedStribeckFriction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    private:
      ExtWidget *gd, *mu;
  };

  class SignalFunctionWidget: public FunctionWidget {

    public:
      SignalFunctionWidget(Element *element);
      ~SignalFunctionWidget();
      QString getType() const { return "SignalFunction"; }
      virtual MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIMCONTROL; }
      void updateWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    private:
      ExtWidget *sRef;
      Element *dummy;
  };

  class PolarContourFunctionWidget : public FunctionWidget {

    public:
      PolarContourFunctionWidget();
      QString getType() const { return "PolarContourFunction"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *radiusFunction;
  };

}

#endif
