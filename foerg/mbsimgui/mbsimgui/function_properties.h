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
    virtual std::string getType() const { return "Function"; }
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
    inline std::string getType() const { return "ConstantFunction"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty c;
};

class LinearFunctionProperty : public FunctionProperty {
  public:
    LinearFunctionProperty(const std::string &ext, int m=1);
    virtual Property* clone() const {return new LinearFunctionProperty(*this);}
    inline std::string getType() const { return "LinearFunction"; }
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
    inline std::string getType() const { return "NestedFunction"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty fo, fi;
};

class VectorValuedFunctionProperty : public FunctionProperty {
  public:
    VectorValuedFunctionProperty(int m=0);
    virtual Property* clone() const {return new VectorValuedFunctionProperty(*this);}
    inline std::string getType() const { return "VectorValuedFunction"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ListProperty functions;
};

class PiecewiseDefinedFunctionProperty : public FunctionProperty {
  public:
    PiecewiseDefinedFunctionProperty(const std::string &ext);
    virtual Property* clone() const {return new PiecewiseDefinedFunctionProperty(*this);}
    inline std::string getType() const { return "PiecewiseDefinedFunction"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ListProperty functions;
    ExtProperty contDiff;
};

class TranslationAlongXAxisProperty: public FunctionProperty {
  public:
    TranslationAlongXAxisProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongXAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "TranslationAlongXAxis"; }
};

class TranslationAlongYAxisProperty: public FunctionProperty {
  public:
    TranslationAlongYAxisProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongYAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "TranslationAlongYAxis"; }
};

class TranslationAlongZAxisProperty: public FunctionProperty {
  public:
    TranslationAlongZAxisProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongZAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "TranslationAlongZAxis"; }
};

class TranslationAlongAxesXYProperty: public FunctionProperty {
  public:
    TranslationAlongAxesXYProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongAxesXYProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?2:0;}
    inline std::string getType() const { return "TranslationAlongAxesXY"; }
};

class TranslationAlongAxesYZProperty: public FunctionProperty {
  public:
    TranslationAlongAxesYZProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongAxesYZProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?2:0;}
    inline std::string getType() const { return "TranslationAlongAxesYZ"; }
};

class TranslationAlongAxesXZProperty: public FunctionProperty {
  public:
    TranslationAlongAxesXZProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongAxesXZProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?2:0;}
    inline std::string getType() const { return "TranslationAlongAxesXZ"; }
};

class TranslationAlongAxesXYZProperty: public FunctionProperty {
  public:
    TranslationAlongAxesXYZProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new TranslationAlongAxesXYZProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?3:0;}
    inline std::string getType() const { return "TranslationAlongAxesXYZ"; }
};

class TranslationAlongFixedAxisProperty : public FunctionProperty {
  public:
    TranslationAlongFixedAxisProperty(const std::string &ext);
    virtual Property* clone() const {return new TranslationAlongFixedAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "TranslationAlongFixedAxis"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty a;
};

class LinearTranslationProperty : public FunctionProperty {
  public:
    LinearTranslationProperty(const std::string &ext, int m=1, int n=1);
    virtual Property* clone() const {return new LinearTranslationProperty(*this);}
    int getArg1Size() const;
    inline std::string getType() const { return "LinearTranslation"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty A, b;
};

class RotationAboutXAxisProperty : public FunctionProperty {
  public:
    RotationAboutXAxisProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutXAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "RotationAboutXAxis"; }
};

class RotationAboutYAxisProperty : public FunctionProperty {
  public:
    RotationAboutYAxisProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutYAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "RotationAboutYAxis"; }
};

class RotationAboutZAxisProperty : public FunctionProperty {
  public:
    RotationAboutZAxisProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutZAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "RotationAboutZAxis"; }
};

class RotationAboutAxesXYProperty : public FunctionProperty {
  public:
    RotationAboutAxesXYProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutAxesXYProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?2:0;}
    inline std::string getType() const { return "RotationAboutAxesXY"; }
};

class RotationAboutAxesYZProperty : public FunctionProperty {
  public:
    RotationAboutAxesYZProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutAxesYZProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?2:0;}
    inline std::string getType() const { return "RotationAboutAxesYZ"; }
};

class RotationAboutAxesXZProperty : public FunctionProperty {
  public:
    RotationAboutAxesXZProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutAxesXZProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?2:0;}
    inline std::string getType() const { return "RotationAboutAxesXZ"; }
};

class RotationAboutAxesXYZProperty : public FunctionProperty {
  public:
    RotationAboutAxesXYZProperty(const std::string &ext) : FunctionProperty(ext) { }
    virtual Property* clone() const {return new RotationAboutAxesXYZProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?3:0;}
    inline std::string getType() const { return "RotationAboutAxesXYZ"; }
};

class RotationAboutFixedAxisProperty : public FunctionProperty {
  public:
    RotationAboutFixedAxisProperty(const std::string &ext);
    virtual Property* clone() const {return new RotationAboutFixedAxisProperty(*this);}
    int getArg1Size() const {return ext[0]=='V'?1:0;}
    inline std::string getType() const { return "RotationAboutFixedAxis"; }
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
    inline std::string getType() const { return "TCardanAngles"; }
};

class QuadraticFunctionProperty : public FunctionProperty {
  public:
    QuadraticFunctionProperty(const std::string &ext, int m=1);
    virtual Property* clone() const {return new QuadraticFunctionProperty(*this);}
    inline std::string getType() const { return "QuadraticFunction"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty a0, a1, a2;
};

class SinusoidalFunctionProperty : public FunctionProperty {
  public:
    SinusoidalFunctionProperty(const std::string &ext, int m=1);
    virtual Property* clone() const {return new SinusoidalFunctionProperty(*this);}
    inline std::string getType() const { return "SinusoidalFunction"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty a, f, p, o;
};

class TabularFunctionProperty : public FunctionProperty {
  public:
    TabularFunctionProperty(const std::string &ext);
    virtual Property* clone() const {return new TabularFunctionProperty(*this);}
    inline std::string getType() const { return "TabularFunction"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ChoiceProperty choice;
};

class LinearCombinationFunctionProperty : public FunctionProperty {

  public:
    LinearCombinationFunctionProperty(const std::string &ext);
    virtual Property* clone() const {return new LinearCombinationFunctionProperty(*this);}
    inline std::string getType() const { return "LinearCombinationFunction"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ListProperty functions;
};

class SymbolicFunctionProperty : public FunctionProperty {
  public:
    SymbolicFunctionProperty(const std::string &ext, const std::vector<std::string> &var);
    virtual Property* clone() const {return new SymbolicFunctionProperty(*this);}
    int getArg1Size() const;
    int getArg2Size() const;
    inline std::string getType() const { return "SymbolicFunction"; }
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
    inline std::string getType() const { return "LinearSpringDamperForce"; }
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

