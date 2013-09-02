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

class ChoiceProperty;

class FunctionProperty : public Property {
  public:
    FunctionProperty(const std::string &ext_="") : ext(ext_) {}
    virtual ~FunctionProperty() {}
    virtual int getArg1Size() const {return 0;}
    virtual int getArg2Size() const {return 0;}
    virtual std::string getType() const { return "Function_"+ext; }
    virtual std::string getExt() const { return ext; }
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  protected:
    std::string ext;
};

//class Function2Property : public Property {
//  public:
//    Function2Property(const std::string &ext_="") : ext(ext_) {}
//    virtual ~Function2Property() {}
//    virtual std::string getType() const { return "Function2_"+ext; }
//    virtual std::string getExt() const { return ext; }
//    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
//  protected:
//    std::string ext;
//};

class SymbolicFunction1Property : public FunctionProperty {
  public:
    SymbolicFunction1Property(const std::string &ext, const std::string &var);
    virtual Property* clone() const {return new SymbolicFunction1Property(*this);}
    int getArg1Size() const;
    inline std::string getType() const { return "SymbolicFunction_"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    int getArgDim() const;
  protected:
    ExtProperty f;
    std::vector<ExtProperty> argname, argdim;
};

class ConstantFunction1Property : public FunctionProperty {
  public:
    ConstantFunction1Property(const std::string &ext, int m=1);
    virtual Property* clone() const {return new ConstantFunction1Property(*this);}
    inline std::string getType() const { return "ConstantFunction_"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty c;
};

class LinearFunctionTestProperty : public FunctionProperty {
  public:
    LinearFunctionTestProperty(const std::string &ext, int m=0, int n=0);
    virtual Property* clone() const {return new LinearFunctionTestProperty(*this);}
    inline std::string getType() const { return "LinearFunction_"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty choice;
    ExtProperty a, b;
};

class LinearFunction1Property : public FunctionProperty {
  public:
    LinearFunction1Property(const std::string &ext, int m=0, int n=0);
    virtual Property* clone() const {return new LinearFunction1Property(*this);}
    int getArg1Size() const;
    inline std::string getType() const { return "LinearFunction_"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty a, b;
};

class RotationAboutFixedAxisProperty : public FunctionProperty {
  public:
    RotationAboutFixedAxisProperty(const std::string &ext);
    virtual Property* clone() const {return new RotationAboutFixedAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "RotationAboutFixedAxis_"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty a;
};

class QuadraticFunction1Property : public FunctionProperty {
  public:
    QuadraticFunction1Property();
    virtual Property* clone() const {return new QuadraticFunction1Property(*this);}
    inline std::string getType() const { return "QuadraticFunction1_VS"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty a0, a1, a2;
};

class SinusFunction1Property : public FunctionProperty {
  public:
    SinusFunction1Property();
    virtual Property* clone() const {return new SinusFunction1Property(*this);}
    inline std::string getType() const { return "SinusFunction1_VS"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty a, f, p, o;
};

class TabularFunction1Property : public FunctionProperty {
  public:
    TabularFunction1Property();
    virtual Property* clone() const {return new TabularFunction1Property(*this);}
    inline std::string getType() const { return "TabularFunction1_VS"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ChoiceProperty choice;
};

class SummationFunction1Property : public FunctionProperty {

  public:
    SummationFunction1Property() {}
    virtual Property* clone() const {return new SummationFunction1Property(*this);}
    inline std::string getType() const { return "SummationFunction1_VS"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::vector<ContainerProperty> function;
};

class SymbolicFunction2Property : public FunctionProperty {
  public:
    SymbolicFunction2Property(const std::string &ext, const std::vector<std::string> &var);
    virtual Property* clone() const {return new SymbolicFunction2Property(*this);}
    int getArg1Size() const;
    int getArg2Size() const;
    inline std::string getType() const { return "SymbolicFunction_"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty f;
    std::vector<ExtProperty> argname, argdim;
};

class LinearSpringDamperForceProperty : public FunctionProperty {
  public:
    LinearSpringDamperForceProperty();
    virtual Property* clone() const {return new LinearSpringDamperForceProperty(*this);}
    inline std::string getType() const { return "LinearSpringDamperForce"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty c, d, l0;
};

class LinearRegularizedBilateralConstraintProperty: public FunctionProperty {
  public:
    LinearRegularizedBilateralConstraintProperty();
    virtual Property* clone() const {return new LinearRegularizedBilateralConstraintProperty(*this);}
    std::string getType() const { return "LinearRegularizedBilateralConstraint"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  private:
    ExtProperty c, d;
};

class LinearRegularizedUnilateralConstraintProperty: public FunctionProperty {
  public:
    LinearRegularizedUnilateralConstraintProperty(); 

    virtual Property* clone() const {return new LinearRegularizedUnilateralConstraintProperty(*this);}
    virtual std::string getType() const { return "LinearRegularizedUnilateralConstraint"; }

    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  private:
    ExtProperty c, d;
};

class LinearRegularizedCoulombFrictionProperty: public FunctionProperty {
  public:
    LinearRegularizedCoulombFrictionProperty(); 

    virtual Property* clone() const {return new LinearRegularizedCoulombFrictionProperty(*this);}
    virtual std::string getType() const { return "LinearRegularizedCoulombFriction"; }

    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  private:
    ExtProperty gd, mu;
};

#endif

