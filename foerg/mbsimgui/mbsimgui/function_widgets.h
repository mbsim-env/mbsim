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
class ChoiceWidget;
class QStackedWidget;
class QListWidget;
class QSpinBox;

class FunctionWidget : public Widget {
  Q_OBJECT
  public:
    FunctionWidget(const QString &ext_="") : ext(ext_) {}
    virtual ~FunctionWidget() {}
    virtual int getArg1Size() const {return 0;}
    virtual int getArg2Size() const {return 0;}
  protected:
    QString ext;
  public slots:
    virtual void resize_(int m, int n) {}
  signals:
    void arg1SizeChanged(int);
};

//class Function2Widget : public Widget {
//  Q_OBJECT
//  public:
//    Function2Widget() {}
//    virtual ~Function2Widget() {}
//  public slots:
//    virtual void resize_(int m, int n) {}
//};

class SymbolicFunction1Widget : public FunctionWidget {

  friend class SymbolicFunction1Property;

  public:
    SymbolicFunction1Widget(const QString &ext, const QString &var, int max=99);
    int getArg1Size() const;
  protected:
    ExtWidget *f;
    std::vector<ExtWidget*> argname, argdim;
};

class ConstantFunction1Widget : public FunctionWidget {

  friend class ConstantFunction1Property;

  public:
    ConstantFunction1Widget(const QString &ext, int m);
    void resize_(int m, int n);
  protected:
    ExtWidget *c;
};

class LinearFunctionTestWidget : public FunctionWidget {

  friend class LinearFunctionTestProperty;

  public:
    LinearFunctionTestWidget(bool vec, int n);
  protected:
    ExtWidget *choice;
    ExtWidget *a, *b;
};

class LinearFunction1Widget : public FunctionWidget {

  friend class LinearFunction1Property;

  public:
    LinearFunction1Widget(const QString &ext, int m, int n);
    int getArg1Size() const;
    void resize_(int m, int n);
  protected:
    ExtWidget *a, *b;
};

class RotationAboutFixedAxisWidget : public FunctionWidget {

  friend class RotationAboutFixedAxisProperty;

  public:
    RotationAboutFixedAxisWidget(const QString &ext);
    int getArg1Size() const {return ext[0]=='V'?1:0;}
  protected:
    ExtWidget *a;
};

class QuadraticFunction1Widget : public FunctionWidget {

  friend class QuadraticFunction1Property;

  public:
    QuadraticFunction1Widget(int n);
    void resize_(int m, int n);

  protected:
    ExtWidget *a0, *a1, *a2;
};

class SinusFunction1Widget : public FunctionWidget {

  friend class SinusFunction1Property;

  public:
    SinusFunction1Widget(int n);
    void resize_(int m, int n);

  protected:
    ExtWidget *a, *f, *p, *o;
};

class TabularFunction1Widget : public FunctionWidget {

  friend class TabularFunction1Property;

  public:
    TabularFunction1Widget(int n);

  protected:
    ChoiceWidget* choice;
};

class SummationFunction1Widget : public FunctionWidget {
  Q_OBJECT

  friend class SummationFunction1Property;

  public:
    SummationFunction1Widget(int n);
    void resize_(int m, int n);

  protected:
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
    void resize_();
};

class SymbolicFunction2Widget : public FunctionWidget {

  friend class SymbolicFunction2Property;

  public:
    SymbolicFunction2Widget(const QString &ext, const QStringList &var, int max=99);
    int getArg1Size() const;
    int getArg2Size() const;
  protected:
    ExtWidget *f;
    std::vector<ExtWidget*> argname, argdim;
};

class LinearSpringDamperForceWidget : public FunctionWidget {

  friend class LinearSpringDamperForceProperty;

  public:
    LinearSpringDamperForceWidget();
  protected:
    ExtWidget *c, *d, *l0;
};

class LinearRegularizedBilateralConstraintWidget: public FunctionWidget {

  friend class LinearRegularizedBilateralConstraintProperty;

  public:
    LinearRegularizedBilateralConstraintWidget(); 


  private:
    ExtWidget *c, *d;
};

class LinearRegularizedUnilateralConstraintWidget: public FunctionWidget {

  friend class LinearRegularizedUnilateralConstraintProperty;

  public:
    LinearRegularizedUnilateralConstraintWidget(); 


  private:
    ExtWidget *c, *d;
};

class LinearRegularizedCoulombFrictionWidget: public FunctionWidget {

  friend class LinearRegularizedCoulombFrictionProperty;

  public:
    LinearRegularizedCoulombFrictionWidget(); 


  private:
    ExtWidget *gd, *mu;
};

#endif
