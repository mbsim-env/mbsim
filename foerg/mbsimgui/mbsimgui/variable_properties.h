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

#ifndef _STRING_PROPERTIES_H_
#define _STRING_PROPERTIES_H_

#include <string>
#include <vector>
#include "utils.h"
#include "property.h"
#include "variable_widgets.h"
#include "basic_widgets.h"
#include "property_property_dialog.h"

namespace MBXMLUtils {
  class TiXmlElement;
  class TiXmlNode;
}

class VariableProperty : public PhysicalProperty {
  public:
    VariableProperty(const std::string &name="", const std::string &value="", const Units &unit=NoUnitUnits()) : PhysicalProperty(name,value,unit) {}
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class OctaveExpressionProperty : public VariableProperty {
  public:
    OctaveExpressionProperty(const std::string &name="", const std::string &scalar="1", const Units &unit=NoUnitUnits()) : VariableProperty(name,scalar,unit) {}
    virtual Property* clone() const {return new OctaveExpressionProperty(*this);}
//    std::string getValue() const { return value; }
//    void setValue(const std::string &str) { value = str; }
//    const std::string& getExpression() const {return expression;}
//    void setExpression(const std::string &x) {expression = x; setValue(expression);}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    Widget* createWidget() { return new OctaveExpressionWidget; }

//  private:
//    std::string expression;
};

class ScalarProperty : public VariableProperty {
  protected:
    std::string scalar;
  public:
    ScalarProperty(const std::string &name="", const std::string &scalar="1", const Units &unit=NoUnitUnits()) : VariableProperty(name,scalar,unit) {}
    virtual Property* clone() const {return new ScalarProperty(*this);}
//    const std::string& getScalar() const {return scalar;}
//    void setScalar(const std::string &x) {scalar = x; setValue(scalar);}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    //PropertyPropertyDialog* createPropertyDialog() { return new PropertyPropertyDialog(this,new ScalarWidget); }
    //PropertyPropertyDialog* createUnitDialog() { return new PropertyPropertyDialog(this,new UnitWidget(angleUnits(),1)); }
    Widget* createWidget() { return new ScalarWidget("1",units); }
};

class VecProperty : public VariableProperty {
  private:
    std::vector<std::string> value;
  public:
    VecProperty(int size, const Units &unit=NoUnitUnits());
    VecProperty(const std::vector<std::string> &x) : VariableProperty("",toStr(x)), value(x) {}
    VecProperty(const std::string &name, const std::vector<std::string> &x, const Units &unit=NoUnitUnits()) : VariableProperty(name,toStr(x),unit), value(x) {}
    ~VecProperty();
    virtual Property* clone() const {return new VecProperty(*this);}
    std::vector<std::string> getVec() const {return value;}
    void setVec(const std::vector<std::string> &x) {value = x; setValue(toStr(value));}
    int size() const {return value.size();}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    Widget* createWidget() { return new VecWidget(1,false,units); }
};

class MatProperty : public VariableProperty {

  private:
    std::vector<std::vector<std::string> > value;
  public:
    MatProperty(int rows, int cols);
    MatProperty(const std::vector<std::vector<std::string> > &A, const Units &unit=NoUnitUnits()) : VariableProperty("",toStr(A),unit), value(A) {}
    MatProperty(const std::string &name, const std::vector<std::vector<std::string> > &A, const Units &unit=NoUnitUnits()) : VariableProperty(name,toStr(A),unit), value(A) {}
    virtual Property* clone() const {return new MatProperty(*this);}
    std::vector<std::vector<std::string> > getMat() const {return value;}
    void setMat(const std::vector<std::vector<std::string> > &A) {value = A; setValue(toStr(value));}
//    std::string getValue() const {return toStr(getMat());}
//    void setValue(const std::string &str) {setMat(strToMat(str));}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    int rows() const {return value.size();}
    int cols() const {return value[0].size();}
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    Widget* createWidget() { return new MatWidget; }
};

class CardanProperty : public VariableProperty {

  private:
    std::vector<std::string> angles;
  public:
    CardanProperty(const std::string &name="");
    ~CardanProperty();
    virtual Property* clone() const {return new CardanProperty(*this);}
    const std::vector<std::string>& getAngles() const {return angles;}
    void setAngles(const std::vector<std::string> &x) {angles = x; setValue(toStr(angles));}
    int size() const {return angles.size();}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    Widget* createWidget() { return new CardanWidget; }
};

class PhysicalVariableProperty : public Property {
  protected:
    VariableProperty* value;
    std::string xmlName;
  public:
    PhysicalVariableProperty(VariableProperty *value_=0, const std::string &unit_="", const std::string &xmlName_="") : Property("",""), value(value_), xmlName(xmlName_) {}
    PhysicalVariableProperty(const PhysicalVariableProperty &p) : value(static_cast<VariableProperty*>(p.value->clone())), xmlName(p.xmlName) {}
    ~PhysicalVariableProperty() {delete value;}
    PhysicalVariableProperty& operator=(const PhysicalVariableProperty &p) {delete value; value=static_cast<VariableProperty*>(p.value->clone()); xmlName=p.xmlName;}
    virtual Property* clone() const {return new PhysicalVariableProperty(*this);}
//    std::string getValue() const {return value->getValue();}
//    void setValue(const std::string &str) {value->setValue(str);}
//    std::string getUnit() const {return unit;}
//    void setUnit(const std::string &unit_) {unit = unit_;}
    virtual VariableProperty* getProperty() {return value;}
    const std::string& getXmlName() const {return xmlName;}
    void setXmlName(const std::string &name) {xmlName = name;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

//class VecFromFileProperty : public VariableProperty {
//
//  public:
//    VecFromFileProperty(const std::string &file_="") : file(file_) {}
//    virtual Property* clone() const {return new VecFromFileProperty(*this);}
//    std::string getValue() const;
//    void setValue(const std::string &str) {}
//    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
//    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
//    void fromWidget(QWidget *widget);
//    void toWidget(QWidget *widget);
//
//  protected:
//    std::string file;
//};

class FromFileProperty : public VariableProperty {

  public:
    FromFileProperty(const std::string &file_="") : file(file_) {}
    virtual Property* clone() const {return new FromFileProperty(*this);}
//    std::string getValue() const; 
//    void setValue(const std::string &str) {}
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::string file;
};

class ScalarPropertyFactory: public PropertyFactory {
  public:
    ScalarPropertyFactory(const std::string &value, const Units &unit=NoUnitUnits());
    Property* createProperty(int i=0);
    std::string getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
    WidgetFactory* createWidgetFactory() { return new ScalarWidgetFactory(QString::fromStdString(value),unit); }
  protected:
    std::string value;
    std::vector<std::string> name;
    Units unit;
};

class VecPropertyFactory: public PropertyFactory {
  public:
    VecPropertyFactory(int m, const Units &unit=NoUnitUnits());
    VecPropertyFactory(const std::vector<std::string> &x, const Units &unit=NoUnitUnits());
    Property* createProperty(int i=0);
    std::string getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
    WidgetFactory* createWidgetFactory() { return new VecWidgetFactory(x.size(),unit); }
  protected:
    std::vector<std::string> x;
    std::vector<std::string> name;
    Units unit;
};

class RotMatPropertyFactory: public PropertyFactory {
  public:
    RotMatPropertyFactory();
    Property* createProperty(int i=0);
    std::string getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
    WidgetFactory* createWidgetFactory() { return new RotMatWidgetFactory; }
  protected:
    std::vector<std::string> name;
};

class MatPropertyFactory: public PropertyFactory {
  public:
    MatPropertyFactory(const Units &unit=NoUnitUnits());
    MatPropertyFactory(const std::vector<std::vector<std::string> > &A, const Units &unit=NoUnitUnits());
    Property* createProperty(int i=0);
    std::string getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
    WidgetFactory* createWidgetFactory() { return new MatWidgetFactory(fromStdMat(A),unit); }
  protected:
    std::vector<std::vector<std::string> > A;
    std::vector<std::string> name;
    Units unit;
};

class SymMatPropertyFactory: public MatPropertyFactory {
  public:
    SymMatPropertyFactory(const Units &unit=NoUnitUnits()) : MatPropertyFactory(unit) { }
    SymMatPropertyFactory(const std::vector<std::vector<std::string> > &A, const Units &unit=NoUnitUnits()) : MatPropertyFactory(A,unit) { }
    WidgetFactory* createWidgetFactory() { return new SymMatWidgetFactory(fromStdMat(A),unit); }
};


#endif

