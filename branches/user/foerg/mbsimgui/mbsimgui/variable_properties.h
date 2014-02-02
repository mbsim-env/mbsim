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

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

class OctaveExpressionProperty : public PhysicalProperty {
  public:
    OctaveExpressionProperty(const std::string &name="", const std::string &scalar="1", const Units &unit=NoUnitUnits()) : PhysicalProperty(name,scalar,unit) {}
    virtual Property* clone() const {return new OctaveExpressionProperty(*this);}
    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    Widget* createWidget() { return new OctaveExpressionWidget(this); }
};

class ScalarProperty : public PhysicalProperty {
  protected:
    std::string scalar;
  public:
    ScalarProperty(const std::string &name="", const std::string &scalar="1", const Units &unit=NoUnitUnits()) : PhysicalProperty(name,scalar,unit) {}
    virtual Property* clone() const {return new ScalarProperty(*this);}
//    const std::string& getScalar() const {return scalar;}
//    void setScalar(const std::string &x) {scalar = x; setValue(scalar);}
    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    //PropertyPropertyDialog* createPropertyDialog() { return new PropertyPropertyDialog(this,new ScalarWidget); }
    //PropertyPropertyDialog* createUnitDialog() { return new PropertyPropertyDialog(this,new UnitWidget(angleUnits(),1)); }
    Widget* createWidget() { return new ScalarWidget(this); }
};

class VecProperty : public PhysicalProperty {
  private:
    std::vector<std::string> value;
  public:
    VecProperty(int size, const Units &unit=NoUnitUnits());
    VecProperty(const std::vector<std::string> &x) : PhysicalProperty("",toStr(x)), value(x) {}
    VecProperty(const std::string &name, const std::vector<std::string> &x, const Units &unit=NoUnitUnits()) : PhysicalProperty(name,toStr(x),unit), value(x) {}
    ~VecProperty();
    virtual Property* clone() const {return new VecProperty(*this);}
    std::vector<std::string> getVec() const {return value;}
    void setVec(const std::vector<std::string> &x) {value = x; setValue(toStr(value));}
    int size(int i=0) const {return value.size();}
    void setSize(int i=0, int size=0) { if(value.size()!=size) setVec(::getVec<std::string>(size,"0")); }
    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    Widget* createWidget() { return new VecWidget(this); }
};

class MatProperty : public PhysicalProperty {

  private:
    std::vector<std::vector<std::string> > value;
  public:
    MatProperty(int rows, int cols);
    MatProperty(const std::vector<std::vector<std::string> > &A, const Units &unit=NoUnitUnits()) : PhysicalProperty("",toStr(A),unit), value(A) {}
    MatProperty(const std::string &name, const std::vector<std::vector<std::string> > &A, const Units &unit=NoUnitUnits()) : PhysicalProperty(name,toStr(A),unit), value(A) {}
    virtual Property* clone() const {return new MatProperty(*this);}
    std::vector<std::vector<std::string> > getMat() const {return value;}
    void setMat(const std::vector<std::vector<std::string> > &A) {value = A; setValue(toStr(value));}
//    std::string getValue() const {return toStr(getMat());}
//    void setValue(const std::string &str) {setMat(strToMat(str));}
    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    int rows() const {return value.size();}
    int cols() const {return value[0].size();}
    int size(int i=0) { return (i==0)?rows():cols(); }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    Widget* createWidget() { return new MatWidget(this); }
};

class VarMatProperty : public MatProperty {
  public:
    VarMatProperty(int rows, int cols) : MatProperty(rows,cols) { }
    VarMatProperty(const std::vector<std::vector<std::string> > &A, const Units &unit=NoUnitUnits()) : MatProperty(A,unit) { }
    VarMatProperty(const std::string &name, const std::vector<std::vector<std::string> > &A, const Units &unit=NoUnitUnits()) : MatProperty(name,A,unit) { }
    Widget* createWidget() { return new MatColsVarWidget(this); }
};

class SymMatProperty : public MatProperty {

  public:
    SymMatProperty(const std::vector<std::vector<std::string> > &A, const Units &unit=NoUnitUnits()) : MatProperty(A,unit) {}
    Widget* createWidget() { return new SymMatWidget(this); }
};

class CardanProperty : public PhysicalProperty {

  private:
    std::vector<std::string> angles;
  public:
    CardanProperty(const std::string &name="");
    ~CardanProperty();
    virtual Property* clone() const {return new CardanProperty(*this);}
    const std::vector<std::string>& getAngles() const {return angles;}
    void setAngles(const std::vector<std::string> &x) {angles = x; setValue(toStr(angles));}
    int size() const {return angles.size();}
    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    Widget* createWidget() { return new CardanWidget(this); }
};

class PhysicalVariableProperty : public Property {
  protected:
    PhysicalProperty* value;
    MBXMLUtils::FQN xmlName;
  public:
    PhysicalVariableProperty(PhysicalProperty *value_=0, const std::string &unit_="", const MBXMLUtils::FQN &xmlName_="") : Property("",""), value(value_), xmlName(xmlName_) {}
    PhysicalVariableProperty(const PhysicalVariableProperty &p) : value(static_cast<PhysicalProperty*>(p.value->clone())), xmlName(p.xmlName) {}
    ~PhysicalVariableProperty() {delete value;}
    PhysicalVariableProperty& operator=(const PhysicalVariableProperty &p) {delete value; value=static_cast<PhysicalProperty*>(p.value->clone()); xmlName=p.xmlName;}
    virtual Property* clone() const {return new PhysicalVariableProperty(*this);}
//    std::string getValue() const {return value->getValue();}
//    void setValue(const std::string &str) {value->setValue(str);}
//    std::string getUnit() const {return unit;}
//    void setUnit(const std::string &unit_) {unit = unit_;}
    virtual PhysicalProperty* getProperty() {return value;}
    const MBXMLUtils::FQN& getXmlName() const {return xmlName;}
    void setXmlName(const std::string &name) {xmlName = name;}
    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

//class VecFromFileProperty : public PhysicalProperty {
//
//  public:
//    VecFromFileProperty(const std::string &file_="") : file(file_) {}
//    virtual Property* clone() const {return new VecFromFileProperty(*this);}
//    std::string getValue() const;
//    void setValue(const std::string &str) {}
//    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
//    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
//    void fromWidget(QWidget *widget);
//    void toWidget(QWidget *widget);
//
//  protected:
//    std::string file;
//};

class FromFileProperty : public PhysicalProperty {

  public:
    FromFileProperty(const std::string &file_="") : file(file_) {}
    virtual Property* clone() const {return new FromFileProperty(*this);}
//    std::string getValue() const; 
//    void setValue(const std::string &str) {}
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::string file;
};

class ScalarPropertyFactory: public PropertyFactory {
  public:
    ScalarPropertyFactory(const std::string &value, const Units &unit=NoUnitUnits());
    Property* createProperty(int i=0);
    MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
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
    MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
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
    MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
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
    MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
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

class VariableProperty : public PhysicalProperty {

  protected:
    std::vector<PhysicalProperty*> property;
    int index;
    std::vector<MBXMLUtils::FQN> name;
  public:
    VariableProperty(const std::string &name, std::vector<PhysicalProperty*> property_=std::vector<PhysicalProperty*>(0), int index_=0) : PhysicalProperty(name), property(property_), index(index_) { }
    const std::string& getValue() const {return property[index]->getValue();}
    const std::string& getUnit() const {return property[index]->getUnit();}
    const std::string& getEvaluation() const {return property[index]->getEvaluation();}
    const Units& getUnits() const { return property[index]->getUnits(); }
    virtual Property* clone() const {return new VariableProperty(*this);}
    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    Widget* createWidget() { return property[index]->createWidget(); }
    QMenu* createContextMenu() {return new VariableChoiceContextMenu(this);}
    int getIndex() const { return index; }
    void setIndex(int index_) { index = index_; }
    const MBXMLUtils::FQN& getName(int i) const { return name[i]; }
    int getNumberOfInputs() const { return name.size(); }
    int size(int i=0) { return property[index]->size(i); }
    void setSize(int i=0, int size=0) { property[index]->setSize(i,size); }
};

class Scalar_Property : public VariableProperty {

  public:
    Scalar_Property(const std::string &name, const Units &unit=NoUnitUnits());
};

class Vec_Property : public VariableProperty {

  public:
    Vec_Property(const std::string &name, const Units &unit=NoUnitUnits());
    Vec_Property(const std::string &name, const std::vector<std::string> &x, const Units &unit=NoUnitUnits());
};

class Mat_Property : public VariableProperty {

  public:
    Mat_Property(const std::string &name, const Units &unit=NoUnitUnits());
    Mat_Property(const std::string &name, const std::vector<std::vector<std::string> > &A, const Units &unit=NoUnitUnits());
};

class SymMat_Property : public VariableProperty {

  public:
    SymMat_Property(const std::string &name, const Units &unit=NoUnitUnits());
};

class RotMatProperty : public VariableProperty {

  public:
    RotMatProperty(const std::string &name);
};

class VarMat_Property : public VariableProperty {

  public:
    VarMat_Property(const std::string &name, const Units &unit=NoUnitUnits());
    VarMat_Property(const std::string &name, const std::vector<std::vector<std::string> > &A, const Units &unit=NoUnitUnits());
};

#endif

