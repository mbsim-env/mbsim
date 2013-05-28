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

#include "widget.h"

class ExtPhysicalVarWidget;
class ExtWidget;
class QVBoxLayout;
class QComboBox;
class Function1ChoiceWidget;
class WidgetChoiceWidget;
class QStackedWidget;
class QListWidget;

class Function1Widget : public Widget {
  Q_OBJECT
  public:
    Function1Widget(const QString& ext_="") : ext(ext_) {}
    virtual ~Function1Widget() {}
    virtual QString getType() const { return "Function1_"+ext; }
    virtual QString getExt() const { return ext; }
  public slots:
    virtual void resize(int m, int n) {}
  protected:
    QString ext;
};

class Function2Widget : public Widget {
  Q_OBJECT
  public:
    Function2Widget(const QString& ext_="") : ext(ext_) {}
    virtual ~Function2Widget() {}
    virtual QString getType() const { return "Function2_"+ext; }
    virtual QString getExt() const { return ext; }
  public slots:
    virtual void resize(int m, int n) {}
  protected:
    QString ext;
};

class SymbolicFunction1Widget : public Function1Widget {

  friend class SymbolicFunction1Property;

  public:
    SymbolicFunction1Widget(const QString &ext);
    inline QString getType() const { return QString("SymbolicFunction1_")+ext; }
    int getArgDim() const;
  protected:
    ExtWidget *f;
    std::vector<ExtWidget*> argname, argdim;
};

class DifferentiableFunction1Widget : public Function1Widget {
  public:
    DifferentiableFunction1Widget(const QString &ext="") : Function1Widget(ext), order(0) {}
    //virtual ~DifferentiableFunction1() { delete derivatives[0]; derivatives.erase(derivatives.begin()); }
    const Function1Widget& getDerivative(int degree) const { return *(derivatives[degree]); }
    Function1Widget& getDerivative(int degree) { return *(derivatives[degree]); }
    void addDerivative(Function1Widget *diff) { derivatives.push_back(diff); }
    void setDerivative(Function1Widget *diff,size_t degree);

    void setOrderOfDerivative(int i) { order=i; }

    QString getType() const { return "DifferentiableFunction1"; }

  protected:
    std::vector<Function1Widget*> derivatives;
    int order;
};

class ConstantFunction1Widget : public Function1Widget {

  friend class ConstantFunction1Property;

  public:
    ConstantFunction1Widget(const QString &ext, int n);
    inline QString getType() const { return QString("ConstantFunction1_")+ext; }
    void resize(int m, int n);
  protected:
    ExtWidget *c;
};

class QuadraticFunction1Widget : public DifferentiableFunction1Widget {

  friend class QuadraticFunction1Property;

  public:
    QuadraticFunction1Widget(int n);
    inline QString getType() const { return QString("QuadraticFunction1_VS"); }
    void resize(int m, int n);

  protected:
    ExtWidget *a0, *a1, *a2;
};

class SinusFunction1Widget : public DifferentiableFunction1Widget {

  friend class SinusFunction1Property;

  public:
    SinusFunction1Widget(int n);
    inline QString getType() const { return QString("SinusFunction1_VS"); }
    void resize(int m, int n);

 //   class ZerothDerivative : public Function1 {
 //      public:
 //       ZerothDerivative(SinusFunction1 *sin) : Function1(), parent(sin) {}
 //       Vector<Col,double> operator()(const double& x, const void * =NULL);
 //     private:
 //       SinusFunction1 *parent;
 //   };

 //   class FirstDerivative : public Function1 {
 //      public:
 //       FirstDerivative(SinusFunction1 *sin) : Function1(), parent(sin) {}
 //       Vector<Col,double> operator()(const double& x, const void * =NULL);
 //     private:
 //       SinusFunction1 *parent;
 //   };
 //   
 //   class SecondDerivative : public Function1 {
 //      public:
 //       SecondDerivative(SinusFunction1 *sin) : Function1(), parent(sin) {}
 //       Vector<Col,double> operator()(const double& x, const void * =NULL);
 //     private:
 //       SinusFunction1 *parent;
 //   };
  protected:
    ExtWidget *a, *f, *p, *o;
};

class TabularFunction1Widget : public Function1Widget {

  friend class TabularFunction1Property;

  public:
    TabularFunction1Widget(int n);
    inline QString getType() const { return QString("TabularFunction1_VS"); }

  protected:
    WidgetChoiceWidget* choice;
};

class SummationFunction1Widget : public Function1Widget {
  Q_OBJECT

  friend class SummationFunction1Property;

  public:
    SummationFunction1Widget(int n);
    inline QString getType() const { return QString("SummationFunction1_VS"); }
    void resize(int m, int n);

  protected:
    std::vector<Function1ChoiceWidget*> functionChoice;
    std::vector<ExtWidget*> factor;
    QStackedWidget *stackedWidget; 
    QListWidget *functionList; 
    int n;

  protected slots:
    void updateList();
    void addFunction();
    void removeFunction();
    void openContextMenu(const QPoint &pos);
    void changeCurrent(int idx);
  signals:
    void resize();
};

class SymbolicFunction2Widget : public Function2Widget {

  friend class SymbolicFunction2Property;

  public:
    SymbolicFunction2Widget(const QString &ext);
    inline QString getType() const { return QString("SymbolicFunction2_")+ext; }
  protected:
    ExtWidget *f;
    std::vector<ExtWidget*> argname, argdim;
};

class LinearSpringDamperForceWidget : public Function2Widget {

  friend class LinearSpringDamperForceProperty;

  public:
    LinearSpringDamperForceWidget();
    inline QString getType() const { return QString("LinearSpringDamperForce")+ext; }
  protected:
    ExtWidget *c, *d, *l0;
};

class LinearRegularizedBilateralConstraintWidget: public Function2Widget {

  friend class LinearRegularizedBilateralConstraintProperty;

  public:
    LinearRegularizedBilateralConstraintWidget(); 

    virtual QString getType() const { return "LinearRegularizedBilateralConstraint"; }

  private:
    ExtWidget *c, *d;
};

class LinearRegularizedUnilateralConstraintWidget: public Function2Widget {

  friend class LinearRegularizedUnilateralConstraintProperty;

  public:
    LinearRegularizedUnilateralConstraintWidget(); 

    virtual QString getType() const { return "LinearRegularizedUnilateralConstraint"; }

  private:
    ExtWidget *c, *d;
};

class LinearRegularizedCoulombFrictionWidget: public Function2Widget {

  friend class LinearRegularizedCoulombFrictionProperty;

  public:
    LinearRegularizedCoulombFrictionWidget(); 

    virtual QString getType() const { return "LinearRegularizedCoulombFriction"; }

  private:
    ExtWidget *gd, *mu;
};

class Function1ChoiceWidget : public Widget {
  Q_OBJECT

  friend class Function1ChoiceProperty;

  public:
    Function1ChoiceWidget(bool withFactor=false, int n=0, const QString& ext="VS");

    void resize(int m, int n);
    Function1Widget* getFunction(int i);
    Function1Widget* getFunction();

  protected slots:
    void defineFunction(int);

  protected:
    QComboBox *comboBox;
    QStackedWidget *stackedWidget;
    ExtWidget* factor;
    int n;
    QString ext;

  signals:
    void resize();
    void functionChanged();
};

class Function2ChoiceWidget : public Widget {
  Q_OBJECT

  friend class Function2ChoiceProperty;

  public:
    Function2ChoiceWidget(const QString& ext="VVS");

    void resize(int m, int n);
    Function2Widget* getFunction(int i);
    Function2Widget* getFunction();

  protected slots:
      void defineFunction(int);

  protected:
    QComboBox *comboBox;
    QStackedWidget *stackedWidget;
    QString ext;

  signals:
    void resize();
    void functionChanged();
};


#endif

