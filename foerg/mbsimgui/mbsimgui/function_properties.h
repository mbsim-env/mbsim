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
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) { return element; }
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
    void fromWidget(QWidget *widget) { }
    void toWidget(QWidget *widget) { }
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

//class SymbolicFunction1Property : public FunctionProperty {
//  public:
//    SymbolicFunction1Property(const std::string &ext, const std::string &var);
//    virtual Property* clone() const {return new SymbolicFunction1Property(*this);}
//    int getArg1Size() const;
//    inline std::string getType() const { return "SymbolicFunction_"+ext; }
//    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
//    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
//    void fromWidget(QWidget *widget);
//    void toWidget(QWidget *widget);
//    int getArgDim() const;
//  protected:
//    ExtProperty f;
//    std::vector<ExtProperty> argname, argdim;
//};

class ConstantFunctionProperty : public FunctionProperty {
  public:
    ConstantFunctionProperty(const std::string &ext, int m=1);
    virtual Property* clone() const {return new ConstantFunctionProperty(*this);}
    inline std::string getType() const { return "ConstantFunction_"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty c;
};

class LinearFunctionProperty : public FunctionProperty {
  public:
    LinearFunctionProperty(const std::string &ext, int m=1, int n=1);
    virtual Property* clone() const {return new LinearFunctionProperty(*this);}
    int getArg1Size() const;
    inline std::string getType() const { return "LinearFunction_"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty a, b;
};

class NestedFunctionProperty : public FunctionProperty {
  public:
    NestedFunctionProperty(const std::string &ext, const std::vector<Property*> &property);
    virtual Property* clone() const {return new NestedFunctionProperty(*this);}
    int getArg1Size() const;
    inline std::string getType() const { return "NestedFunction_"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty fo, fi;
};

class TranslationAlongXAxisProperty: public FunctionProperty {
  public:
    TranslationAlongXAxisProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongXAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "TranslationAlongXAxis_"+ext; }
};

class TranslationAlongYAxisProperty: public FunctionProperty {
  public:
    TranslationAlongYAxisProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongYAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "TranslationAlongYAxis_"+ext; }
};

class TranslationAlongZAxisProperty: public FunctionProperty {
  public:
    TranslationAlongZAxisProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongZAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "TranslationAlongZAxis_"+ext; }
};

class TranslationAlongAxesXYProperty: public FunctionProperty {
  public:
    TranslationAlongAxesXYProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongAxesXYProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?2:0;}
    inline std::string getType() const { return "TranslationAlongAxesXY_"+ext; }
};

class TranslationAlongAxesYZProperty: public FunctionProperty {
  public:
    TranslationAlongAxesYZProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongAxesYZProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?2:0;}
    inline std::string getType() const { return "TranslationAlongAxesYZ_"+ext; }
};

class TranslationAlongAxesXZProperty: public FunctionProperty {
  public:
    TranslationAlongAxesXZProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongAxesXZProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?2:0;}
    inline std::string getType() const { return "TranslationAlongAxesXZ_"+ext; }
};

class TranslationAlongAxesXYZProperty: public FunctionProperty {
  public:
    TranslationAlongAxesXYZProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongAxesXYZProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?3:0;}
    inline std::string getType() const { return "TranslationAlongAxesXYZ_"+ext; }
};

class RotationAboutXAxisProperty : public FunctionProperty {
  public:
    RotationAboutXAxisProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutXAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "RotationAboutXAxis_"+ext; }
};

class RotationAboutYAxisProperty : public FunctionProperty {
  public:
    RotationAboutYAxisProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutYAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "RotationAboutYAxis_"+ext; }
};

class RotationAboutZAxisProperty : public FunctionProperty {
  public:
    RotationAboutZAxisProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutZAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "RotationAboutZAxis_"+ext; }
};

class RotationAboutAxesXYProperty : public FunctionProperty {
  public:
    RotationAboutAxesXYProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutAxesXYProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?2:0;}
    inline std::string getType() const { return "RotationAboutAxesXY_"+ext; }
};

class RotationAboutAxesYZProperty : public FunctionProperty {
  public:
    RotationAboutAxesYZProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutAxesYZProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?2:0;}
    inline std::string getType() const { return "RotationAboutAxesYZ_"+ext; }
};

class RotationAboutAxesXZProperty : public FunctionProperty {
  public:
    RotationAboutAxesXZProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutAxesXZProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?2:0;}
    inline std::string getType() const { return "RotationAboutAxesXZ_"+ext; }
};

class RotationAboutAxesXYZProperty : public FunctionProperty {
  public:
    RotationAboutAxesXYZProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutAxesXYZProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?3:0;}
    inline std::string getType() const { return "RotationAboutAxesXYZ_"+ext; }
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

class TCardanAnglesProperty : public FunctionProperty {
  public:
    TCardanAnglesProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TCardanAnglesProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?3:0;}
    inline std::string getType() const { return "TCardanAngles_"+ext; }
};

class QuadraticFunctionProperty : public FunctionProperty {
  public:
    QuadraticFunctionProperty(const std::string &ext, int m=1);
    virtual Property* clone() const {return new QuadraticFunctionProperty(*this);}
    inline std::string getType() const { return "QuadraticFunction_"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty a0, a1, a2;
};

class SinusFunctionProperty : public FunctionProperty {
  public:
    SinusFunctionProperty(const std::string &ext, int m=1);
    virtual Property* clone() const {return new SinusFunctionProperty(*this);}
    inline std::string getType() const { return "SinusFunction_"+ext; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty a, f, p, o;
};

class TabularFunctionProperty : public FunctionProperty {
  public:
    TabularFunctionProperty();
    virtual Property* clone() const {return new TabularFunctionProperty(*this);}
    inline std::string getType() const { return "TabularFunction_V"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ChoiceProperty choice;
};

class SummationFunctionProperty : public FunctionProperty {

  public:
    SummationFunctionProperty() {}
    virtual Property* clone() const {return new SummationFunctionProperty(*this);}
    inline std::string getType() const { return "SummationFunction_VS"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::vector<ContainerProperty> function;
};

class SymbolicFunctionProperty : public FunctionProperty {
  public:
    SymbolicFunctionProperty(const std::string &ext, const std::vector<std::string> &var);
    virtual Property* clone() const {return new SymbolicFunctionProperty(*this);}
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

