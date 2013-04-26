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

#ifndef _FUNCTION_PROPERTIES_H_
#define _FUNCTION_PROPERTIES_H_

#include "extended_properties.h"

class Function1ChoiceProperty;

class Function1Property : public Property {
  public:
    Function1Property(const std::string &ext_="") : ext(ext_) {}
    virtual ~Function1Property() {}
    virtual std::string getType() const { return "Function1_"+ext; }
    virtual std::string getExt() const { return ext; }
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  protected:
    std::string ext;
};

class Function2Property : public Property {
  public:
    Function2Property(const std::string &ext_="") : ext(ext_) {}
    virtual ~Function2Property() {}
    virtual std::string getType() const { return "Function2_"+ext; }
    virtual std::string getExt() const { return ext; }
    virtual void resize(int m, int n) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  protected:
    std::string ext;
};

class DifferentiableFunction1Property : public Function1Property {
  public:
    DifferentiableFunction1Property(const std::string &ext="") : Function1Property(ext), order(0) {}
    //virtual ~DifferentiableFunction1() { delete derivatives[0]; derivatives.erase(derivatives.begin()); }
    const Function1Property& getDerivative(int degree) const { return *(derivatives[degree]); }
    Function1Property& getDerivative(int degree) { return *(derivatives[degree]); }
    void addDerivative(Function1Property *diff) { derivatives.push_back(diff); }
    void setDerivative(Function1Property *diff,size_t degree);

    void setOrderOfDerivative(int i) { order=i; }

    std::string getType() const { return "DifferentiableFunction1"; }

  protected:
    std::vector<Function1Property*> derivatives;
    int order;
};

class ConstantFunction1Property : public Function1Property {
  public:
    ConstantFunction1Property(const std::string &ext);
    inline std::string getType() const { return "ConstantFunction1_"+ext; }
    void resize(int m, int n);
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty c;
};

class QuadraticFunction1Property : public DifferentiableFunction1Property {
  public:
    QuadraticFunction1Property();
    inline std::string getType() const { return "QuadraticFunction1_VS"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty a0, a1, a2;
};

class SinusFunction1Property : public DifferentiableFunction1Property {
  public:
    SinusFunction1Property();
    inline std::string getType() const { return "SinusFunction1_VS"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

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
    ExtProperty a, f, p, o;
};

class TabularFunction1Property : public Function1Property {
  public:
    TabularFunction1Property();
    inline std::string getType() const { return "TabularFunction1_VS"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    PropertyChoiceProperty *choice;
};

class SummationFunction1Property : public Function1Property {

  public:
    SummationFunction1Property() {}
    inline std::string getType() const { return "SummationFunction1_VS"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::vector<Function1ChoiceProperty*> functionChoice;
    std::vector<ExtProperty> factor;
};

class LinearSpringDamperForceProperty : public Function2Property {
  public:
    LinearSpringDamperForceProperty();
    inline std::string getType() const { return "LinearSpringDamperForce"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty c, d, l0;
};

class LinearRegularizedBilateralConstraintProperty: public Function2Property {
  public:
    LinearRegularizedBilateralConstraintProperty();
    std::string getType() const { return "LinearRegularizedBilateralConstraint"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  private:
    ExtProperty c, d;
};

class LinearRegularizedUnilateralConstraintProperty: public Function2Property {
  public:
    LinearRegularizedUnilateralConstraintProperty(); 

    virtual std::string getType() const { return "LinearRegularizedUnilateralConstraint"; }

    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  private:
    ExtProperty c, d;
};

class LinearRegularizedCoulombFrictionProperty: public Function2Property {
  public:
    LinearRegularizedCoulombFrictionProperty(); 

    virtual std::string getType() const { return "LinearRegularizedCoulombFriction"; }

    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  private:
    ExtProperty gd, mu;
};

class Function1ChoiceProperty : public Property {

  public:
    Function1ChoiceProperty(const std::string &xmlName, bool withFactor=false);

    void defineForceLaw(int);

    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    Function1Property *function;
    ExtProperty factor;
    int index;
    std::string xmlName;
};

class Function2ChoiceProperty : public Property {

  public:
    Function2ChoiceProperty(const std::string &xmlName);

    void defineForceLaw(int);

    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    Function2Property *function;
    int index;
    std::string xmlName;
};


#endif

