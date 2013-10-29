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

namespace MBXMLUtils {
  class TiXmlElement;
  class TiXmlNode;
}

class VariableProperty : public Property {

  public:
    virtual std::string getValue() const = 0;
    virtual void setValue(const std::string &str) = 0;
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class OctaveExpressionProperty : public VariableProperty {
  public:
    OctaveExpressionProperty() {}
    virtual Property* clone() const {return new OctaveExpressionProperty(*this);}
    std::string getValue() const { return value; }
    void setValue(const std::string &str) { value = str; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);

  private:
    std::string value;
};

class ScalarProperty : public VariableProperty {
  protected:
    std::string scalar;
  public:
    ScalarProperty(const std::string &scalar_="1") : scalar(scalar_) {}
    virtual Property* clone() const {return new ScalarProperty(*this);}
    std::string getValue() const {return scalar;}
    void setValue(const std::string &scalar_) {scalar = scalar_;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class VecProperty : public VariableProperty {
  private:
    std::vector<std::string> value;
  public:
    VecProperty(int size);
    VecProperty(const std::vector<std::string> &x) : value(x) {}
    ~VecProperty();
    virtual Property* clone() const {return new VecProperty(*this);}
    std::vector<std::string> getVec() const {return value;}
    void setVec(const std::vector<std::string> &x) {value = x;}
    std::string getValue() const {return toStr(getVec());}
    void setValue(const std::string &str) {setVec(strToVec(str));}
    int size() const {return value.size();}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class MatProperty : public VariableProperty {

  private:
    std::vector<std::vector<std::string> > value;
  public:
    MatProperty(int rows, int cols);
    MatProperty(const std::vector<std::vector<std::string> > &A) : value(A) {}
    virtual Property* clone() const {return new MatProperty(*this);}
    std::vector<std::vector<std::string> > getMat() const {return value;}
    void setMat(const std::vector<std::vector<std::string> > &A) {value = A;}
    std::string getValue() const {return toStr(getMat());}
    void setValue(const std::string &str) {setMat(strToMat(str));}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    int rows() const {return value.size();}
    int cols() const {return value[0].size();}
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class CardanProperty : public VariableProperty {

  private:
    std::vector<std::string> angles;
  public:
    CardanProperty();
    ~CardanProperty();
    virtual Property* clone() const {return new CardanProperty(*this);}
    std::vector<std::string> getAngles() const {return angles;}
    void setAngles(const std::vector<std::string> &x) {angles = x;}
    std::string getValue() const {return toStr(getAngles());}
    void setValue(const std::string &str) {setAngles(strToVec(str));}
    int size() const {return angles.size();}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class PhysicalVariableProperty : public Property {
  protected:
    VariableProperty* value;
    std::string unit, xmlName;
  public:
    PhysicalVariableProperty(VariableProperty *value_=0, const std::string &unit_="", const std::string &xmlName_="") : value(value_), unit(unit_), xmlName(xmlName_) {}
    PhysicalVariableProperty(const PhysicalVariableProperty &p) : value(static_cast<VariableProperty*>(p.value->clone())), unit(p.unit), xmlName(p.xmlName) {}
    ~PhysicalVariableProperty() {delete value;}
    PhysicalVariableProperty& operator=(const PhysicalVariableProperty &p) {delete value; value=static_cast<VariableProperty*>(p.value->clone()); unit=p.unit; xmlName=p.xmlName;}
    virtual Property* clone() const {return new PhysicalVariableProperty(*this);}
    std::string getValue() const {return value->getValue();}
    void setValue(const std::string &str) {value->setValue(str);}
    std::string getUnit() const {return unit;}
    void setUnit(const std::string &unit_) {unit = unit_;}
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
    std::string getValue() const; 
    void setValue(const std::string &str) {}
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::string file;
};

class ScalarPropertyFactory: public PropertyFactory {
  public:
    ScalarPropertyFactory(const std::string &value, const std::string &xmlName);
    ScalarPropertyFactory(const std::string &value, const std::string &xmlName, const std::vector<std::string> &unit);
    Property* createProperty(int i=0);
    std::string getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::string value;
    std::vector<std::string> name;
    std::string xmlName;
    std::vector<std::string> unit;
};

class VecPropertyFactory: public PropertyFactory {
  public:
    VecPropertyFactory(int m, const std::string &xmlName);
    VecPropertyFactory(int m, const std::string &xmlName, const std::vector<std::string> &unit);
    VecPropertyFactory(const std::vector<std::string> &x, const std::string &xmlName, const std::vector<std::string> &unit);
    Property* createProperty(int i=0);
    std::string getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<std::string> x;
    std::vector<std::string> name;
    std::string xmlName;
    std::vector<std::string> unit;
};

class RotMatPropertyFactory: public PropertyFactory {
  public:
    RotMatPropertyFactory(const std::string &xmlName);
    RotMatPropertyFactory(const std::string &xmlName, const std::vector<std::string> &unit);
    Property* createProperty(int i=0);
    std::string getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<std::string> name;
    std::string xmlName;
    std::vector<std::string> unit;
};

class MatPropertyFactory: public PropertyFactory {
  public:
    MatPropertyFactory(const std::string &xmlName);
    MatPropertyFactory(const std::vector<std::vector<std::string> > &A, const std::string &xmlName, const std::vector<std::string> &unit);
    Property* createProperty(int i=0);
    std::string getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<std::vector<std::string> > A;
    std::vector<std::string> name;
    std::string xmlName;
    std::vector<std::string> unit;
};

#endif

