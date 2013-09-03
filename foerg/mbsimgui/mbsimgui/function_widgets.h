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

//class SymbolicFunction1Widget : public FunctionWidget {
//
//  friend class SymbolicFunctionProperty;
//
//  public:
//    SymbolicFunction1Widget(const QString &ext, const QString &var, int max=99);
//    int getArg1Size() const;
//  protected:
//    ExtWidget *f;
//    std::vector<ExtWidget*> argname, argdim;
//};

class ConstantFunctionWidget : public FunctionWidget {

  friend class ConstantFunctionProperty;

  public:
    ConstantFunctionWidget(const QString &ext, int m);
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

class LinearFunctionWidget : public FunctionWidget {

  friend class LinearFunctionProperty;

  public:
    LinearFunctionWidget(const QString &ext, int m, int n);
    int getArg1Size() const;
    void resize_(int m, int n);
  protected:
    ExtWidget *a, *b;
};

class NestedFunctionWidget : public FunctionWidget {

  friend class NestedFunctionProperty;

  public:
    NestedFunctionWidget(const QString &ext, const std::vector<QWidget*> &widget, const std::vector<QString> &name);
    int getArg1Size() const;
  protected:
    ExtWidget *fo, *fi;
};

class TranslationAlongXAxisWidget : public FunctionWidget {

  public:
    TranslationAlongXAxisWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?1:0;}
};

class TranslationAlongYAxisWidget : public FunctionWidget {

  public:
    TranslationAlongYAxisWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?1:0;}
};

class TranslationAlongZAxisWidget : public FunctionWidget {

  public:
    TranslationAlongZAxisWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?1:0;}
};

class TranslationAlongAxesXYWidget : public FunctionWidget {

  public:
    TranslationAlongAxesXYWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?2:0;}
};

class TranslationAlongAxesYZWidget : public FunctionWidget {

  public:
    TranslationAlongAxesYZWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?2:0;}
};

class TranslationAlongAxesXZWidget : public FunctionWidget {

  public:
    TranslationAlongAxesXZWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?2:0;}
};

class TranslationAlongAxesXYZWidget : public FunctionWidget {

  public:
    TranslationAlongAxesXYZWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?3:0;}
};

class RotationAboutXAxisWidget : public FunctionWidget {

  public:
    RotationAboutXAxisWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?1:0;}
};

class RotationAboutYAxisWidget : public FunctionWidget {

  public:
    RotationAboutYAxisWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?1:0;}
};

class RotationAboutZAxisWidget : public FunctionWidget {

  public:
    RotationAboutZAxisWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?1:0;}
};

class RotationAboutAxesXYWidget : public FunctionWidget {

  public:
    RotationAboutAxesXYWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?2:0;}
};

class RotationAboutAxesYZWidget : public FunctionWidget {

  public:
    RotationAboutAxesYZWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?2:0;}
};

class RotationAboutAxesXZWidget : public FunctionWidget {

  public:
    RotationAboutAxesXZWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?2:0;}
};

class RotationAboutAxesXYZWidget : public FunctionWidget {

  public:
    RotationAboutAxesXYZWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?3:0;}
};

class RotationAboutFixedAxisWidget : public FunctionWidget {

  friend class RotationAboutFixedAxisProperty;

  public:
    RotationAboutFixedAxisWidget(const QString &ext);
    int getArg1Size() const {return ext[0]=='V'?1:0;}
  protected:
    ExtWidget *a;
};

class TCardanAnglesWidget : public FunctionWidget {

  public:
    TCardanAnglesWidget(const QString &ext) : FunctionWidget(ext) { }
    int getArg1Size() const {return ext[0]=='V'?3:0;}
};

class QuadraticFunctionWidget : public FunctionWidget {

  friend class QuadraticFunctionProperty;

  public:
    QuadraticFunctionWidget(int n);
    void resize_(int m, int n);

  protected:
    ExtWidget *a0, *a1, *a2;
};

class SinusFunctionWidget : public FunctionWidget {

  friend class SinusFunctionProperty;

  public:
    SinusFunctionWidget(const QString &ext, int m=3);
    void resize_(int m, int n);

  protected:
    ExtWidget *a, *f, *p, *o;
};

class TabularFunctionWidget : public FunctionWidget {

  friend class TabularFunctionProperty;

  public:
    TabularFunctionWidget(int n);

  protected:
    ChoiceWidget* choice;
};

class SummationFunctionWidget : public FunctionWidget {
  Q_OBJECT

  friend class SummationFunctionProperty;

  public:
    SummationFunctionWidget(int n);
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

class SymbolicFunctionWidget : public FunctionWidget {

  friend class SymbolicFunctionProperty;

  public:
    SymbolicFunctionWidget(const QString &ext, const QStringList &var, int max=99);
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
