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

#include "xml_widget.h"

class ExtPhysicalVarWidget;
class ExtXMLWidget;
class QVBoxLayout;
class QComboBox;
class Function1ChoiceWidget;
class QStackedWidget;
class QListWidget;

class Function1 : public XMLWidget {
  Q_OBJECT
  public:
    Function1(const QString& ext_="") : ext(ext_) {}
    virtual ~Function1() {}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
    virtual QString getType() const { return "Function1_"+ext; }
    virtual QString getExt() const { return ext; }
  public slots:
    virtual void resize(int m, int n) {}
  protected:
    QString ext;
};

class Function2 : public XMLWidget {
  Q_OBJECT
  public:
    Function2(const QString& ext_="") : ext(ext_) {}
    virtual ~Function2() {}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
    virtual QString getType() const { return "Function2_"+ext; }
    virtual QString getExt() const { return ext; }
  public slots:
    virtual void resize(int m, int n) {}
  protected:
    QString ext;
};

class DifferentiableFunction1 : public Function1 {
  public:
    DifferentiableFunction1(const QString &ext="") : Function1(ext), order(0) {}
    //virtual ~DifferentiableFunction1() { delete derivatives[0]; derivatives.erase(derivatives.begin()); }
    const Function1& getDerivative(int degree) const { return *(derivatives[degree]); }
    Function1& getDerivative(int degree) { return *(derivatives[degree]); }
    void addDerivative(Function1 *diff) { derivatives.push_back(diff); }
    void setDerivative(Function1 *diff,size_t degree);

    void setOrderOfDerivative(int i) { order=i; }

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    QString getType() const { return "DifferentiableFunction1"; }

  protected:
    std::vector<Function1*> derivatives;
    int order;
};

class ConstantFunction1 : public Function1 {
  public:
    ConstantFunction1(const QString &ext);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    inline QString getType() const { return QString("ConstantFunction1_")+ext; }
    void resize(int m, int n);
  protected:
    ExtPhysicalVarWidget *c;
};

class QuadraticFunction1 : public DifferentiableFunction1 {
  public:
    QuadraticFunction1();
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    inline QString getType() const { return QString("QuadraticFunction1_VS"); }
    void resize(int m, int n);

  protected:
    std::vector<ExtPhysicalVarWidget*> var;
    std::vector<ExtXMLWidget*> widget;
};

class SinusFunction1 : public DifferentiableFunction1 {
  public:
    SinusFunction1();
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
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
    std::vector<ExtPhysicalVarWidget*> var;
    std::vector<ExtXMLWidget*> widget;
};

class TabularFunction1 : public Function1 {
  public:
    TabularFunction1();
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    inline QString getType() const { return QString("TabularFunction1_VS"); }

  protected:
    XMLWidget* widget;
};

class SummationFunction1 : public Function1 {
  Q_OBJECT

  public:
    SummationFunction1();
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    inline QString getType() const { return QString("SummationFunction1_VS"); }
    void resize(int m, int n);

  protected:
    std::vector<Function1ChoiceWidget*> functionChoice;
    std::vector<ExtXMLWidget*> factor;
    QStackedWidget *stackedWidget; 
    QListWidget *functionList; 

  protected slots:
    void updateList();
    void addFunction();
    void removeFunction();
    void openContextMenu(const QPoint &pos);
  signals:
    void resize();
};

class LinearSpringDamperForce : public Function2 {
  public:
    LinearSpringDamperForce();
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *parent);
    inline QString getType() const { return QString("LinearSpringDamperForce")+ext; }
  protected:
    std::vector<ExtPhysicalVarWidget*> var;
};

class LinearRegularizedBilateralConstraint: public Function2 {
  public:
    LinearRegularizedBilateralConstraint(); 

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
    virtual QString getType() const { return "LinearRegularizedBilateralConstraint"; }

  private:
    std::vector<ExtXMLWidget*> var;
};

class LinearRegularizedUnilateralConstraint: public Function2 {
  public:
    LinearRegularizedUnilateralConstraint(); 

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
    virtual QString getType() const { return "LinearRegularizedUnilateralConstraint"; }

  private:
    std::vector<ExtXMLWidget*> var;
};

class LinearRegularizedCoulombFriction: public Function2 {
  public:
    LinearRegularizedCoulombFriction(); 

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
    virtual QString getType() const { return "LinearRegularizedCoulombFriction"; }

  private:
    std::vector<ExtXMLWidget*> var;
};

class Function1ChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    Function1ChoiceWidget(const std::string &xmlName, bool withFactor=false);

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void resize(int m, int n) {if(function) function->resize(m,n);}
    Function1* getFunction() {return function;}

  protected slots:
    void defineForceLaw(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    Function1 *function;
    std::string xmlName;
    ExtXMLWidget *factor;
  signals:
    void resize();
    void functionChanged();
};

class Function2ChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    Function2ChoiceWidget(const std::string &xmlName);

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void resize(int m, int n) {if(function) function->resize(m,n);}

  protected slots:
      void defineForceLaw(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    Function2 *function;
    std::string xmlName;
  signals:
    void resize();
};


#endif

