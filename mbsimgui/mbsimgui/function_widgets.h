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

    friend class IdentityFunction;

    public:
    IdentityFunctionWidget(int m=1) { }
  };

  class ConstantFunctionWidget : public FunctionWidget {

    friend class ConstantFunction;

    public:
    ConstantFunctionWidget(int m=1);
    void resize_(int m, int n);
    protected:
    ExtWidget *a0;
  };

  class LinearFunctionWidget : public FunctionWidget {

    friend class LinearFunction;

    public:
    LinearFunctionWidget(int m=1);
    void resize_(int m, int n);
    protected:
    ExtWidget *a0, *a1;
  };

  class QuadraticFunctionWidget : public FunctionWidget {

    friend class QuadraticFunction;

    public:
    QuadraticFunctionWidget(int m=1);
    void resize_(int m, int n);

    protected:
    ExtWidget *a0, *a1, *a2;
  };

  class PolynomFunctionWidget : public FunctionWidget {

    friend class PolynomFunction;

    public:
    PolynomFunctionWidget(int m=1);
    void resize_(int m, int n);

    protected:
    ExtWidget *a;
  };

  class SinusoidalFunctionWidget : public FunctionWidget {

    friend class SinusoidalFunction;

    public:
    SinusoidalFunctionWidget(int m=1);

    protected:
    ExtWidget *a, *f, *p, *o;
  };

  //class StepFunctionWidget : public FunctionWidget {
  //}

  //class PositiveValueFunctionWidget : public FunctionWidget {
  //}

  class AbsoluteValueFunctionWidget : public FunctionWidget {

    friend class AbsoluteValueFunction;

    public:
    AbsoluteValueFunctionWidget(int m=0) { }
  };

  class ModuloFunctionWidget : public FunctionWidget {

    friend class ModuloFunction;

    public:
    ModuloFunctionWidget(int m=0);

    protected:
    ExtWidget *denom;
  };

  class SignumFunctionWidget : public FunctionWidget {

    friend class SignumFunction;

    public:
    SignumFunctionWidget(int m=0) { }
  };

  class AdditionFunctionWidget : public FunctionWidget {

    friend class AdditionFunction;

    public:
    AdditionFunctionWidget(Element* parent, int m=0);
    void resize_(int m, int n);

    protected:
    ExtWidget *f1, *f2;
  };

  class MultiplicationFunctionWidget : public FunctionWidget {

    friend class MultiplicationFunction;

    public:
    MultiplicationFunctionWidget(Element* parent, int m=0);
    void resize_(int m, int n);

    protected:
    ExtWidget *f1, *f2;
  };

  class VectorValuedFunctionWidget : public FunctionWidget {

    friend class VectorValuedFunction;

    public:
    VectorValuedFunctionWidget(Element *parent, int m=0, bool fixedSize=false);
    void resize_(int m, int n);

    protected:
    ExtWidget *functions;
  };

  class NestedFunctionWidget : public FunctionWidget {
    Q_OBJECT

    friend class NestedFunction;

    public:
    NestedFunctionWidget(WidgetFactory *factoryo, WidgetFactory *factoryi);
    int getArg1Size() const;
    void resize_(int m, int n);
    protected:
    QString ext;
    ExtWidget *fo, *fi;
    public slots:
    void resizeVariables();
  };

  class BinaryNestedFunctionWidget : public FunctionWidget {
    Q_OBJECT

    friend class BinaryNestedFunction;

    public:
    BinaryNestedFunctionWidget(WidgetFactory *factoryo, WidgetFactory *factoryi1, WidgetFactory *factoryi2);
    int getArg1Size() const;
    int getArg2Size() const;
    void resize_(int m, int n);
    protected:
    QString ext;
    ExtWidget *fo, *fi1, *fi2;
    public slots:
    void resizeVariables();
  };

  class PiecewiseDefinedFunctionWidget : public FunctionWidget {

    friend class PiecewiseDefinedFunction;

    public:
    PiecewiseDefinedFunctionWidget(Element *parent, int n=0);
    void resize_(int m, int n);

    protected:
    ExtWidget *functions, *contDiff, *shiftAbscissa, *shiftOrdinate;
  };

  class SymbolicFunctionWidget : public FunctionWidget {
    Q_OBJECT

    friend class SymbolicFunction;

    public:
    SymbolicFunctionWidget(const QStringList &var, int m, int max);
    int getArg1Size() const;
    int getArg2Size() const;
    void setArg1Size(int i);
    void resize_(int m, int n);

    protected:
    ExtWidget *f;
    std::vector<ExtWidget*> argname, argdim;

    signals:
    void arg1SizeChanged(int i);
  };

  class TabularFunctionWidget : public FunctionWidget {

    friend class TabularFunction;

    public:
    TabularFunctionWidget(int n);
    void resize_(int m, int n);

    protected:
    ChoiceWidget2* choice;
  };

  class TwoDimensionalTabularFunctionWidget : public FunctionWidget {

    friend class TwoDimensionalTabularFunction;

    public:
    TwoDimensionalTabularFunctionWidget(int n);
    void resize_(int m, int n);

    protected:
    ExtWidget *x, *y, *xy;
  };

  class PiecewisePolynomFunctionWidget : public FunctionWidget {

    friend class PiecewisePolynomFunction;

    public:
    PiecewisePolynomFunctionWidget(int n);
    void resize_(int m, int n);

    protected:
    ChoiceWidget2* choice;
    ExtWidget *method;
  };

  class FourierFunctionWidget : public FunctionWidget {

    friend class FourierFunction;

    public:
    FourierFunctionWidget(int n);
    void resize_(int m, int n);

    protected:
    ExtWidget *f, *a0, *amplitudePhaseAngleForm;
    ChoiceWidget2* choice;
  };

  class BidirectionalFunctionWidget : public FunctionWidget {
    Q_OBJECT

    friend class BidirectionalFunction;

    public:
    BidirectionalFunctionWidget();
    void resize_(int m, int n);
    protected:
    ExtWidget *fn, *fp;
  };

  class PeriodicFunctionWidget : public FunctionWidget {
    Q_OBJECT

    friend class PeriodicFunction;

    public:
    PeriodicFunctionWidget(WidgetFactory *factory);
    void resize_(int m, int n);
    protected:
    ExtWidget *f, *T;
  };

  class LinearSpringDamperForceWidget : public FunctionWidget {

    friend class LinearSpringDamperForce;

    public:
    LinearSpringDamperForceWidget();
    protected:
    ExtWidget *c, *d, *l0;
  };

  class NonlinearSpringDamperForceWidget : public FunctionWidget {

    friend class NonlinearSpringDamperForce;

    public:
    NonlinearSpringDamperForceWidget(Element *parent);
    protected:
    ExtWidget *g, *gd;
  };

  class LinearRegularizedBilateralConstraintWidget: public FunctionWidget {

    friend class LinearRegularizedBilateralConstraint;

    public:
    LinearRegularizedBilateralConstraintWidget(); 

    private:
    ExtWidget *c, *d;
  };

  class LinearRegularizedUnilateralConstraintWidget: public FunctionWidget {

    friend class LinearRegularizedUnilateralConstraint;

    public:
    LinearRegularizedUnilateralConstraintWidget(); 

    private:
    ExtWidget *c, *d;
  };

  class LinearRegularizedCoulombFrictionWidget: public FunctionWidget {

    friend class LinearRegularizedCoulombFriction;

    public:
    LinearRegularizedCoulombFrictionWidget(); 

    private:
    ExtWidget *gd, *mu;
  };

  class LinearRegularizedStribeckFrictionWidget: public FunctionWidget {

    friend class LinearRegularizedStribeckFriction;

    public:
    LinearRegularizedStribeckFrictionWidget();

    private:
    ExtWidget *gd, *mu;
  };

  class SignalFunctionWidget: public FunctionWidget {

    friend class SignalFunction;

    public:
    SignalFunctionWidget(Element *element); 
    ~SignalFunctionWidget();
    void updateWidget();

    private:
    ExtWidget *sRef;
    Element *dummy;
  };

  class PolarContourFunctionWidget : public FunctionWidget {

    friend class PolarContourFunction;

    public:
    PolarContourFunctionWidget();
    void resize_(int m, int n);
    protected:
    ExtWidget *radiusFunction;
  };

}

#endif
