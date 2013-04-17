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

class TiXmlElement;
class TiXmlNode;

class StringProperty : public Property {

  public:
    virtual std::string getValue() const = 0;
    virtual void setValue(const std::string &str) = 0;
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

//class TextProperty : public StringProperty {
//
//  public:
//    ChoiceProperty(const std::string &value_) : value(value_) {}
//    std::string getValue() const {return value;}
//    void setValue(const std::string &str) {value=str;}
//    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
//    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
//
//  protected:
//    std::string value;
//};
//

class OctaveExpressionProperty : public StringProperty {
  public:
    OctaveExpressionProperty() {}
    std::string getValue() const { return value; }
    void setValue(const std::string &str) { value = str; }
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);

  private:
    std::string value;
};

class ScalarProperty : public StringProperty {
  protected:
    std::string scalar;
  public:
    ScalarProperty(const std::string &scalar_="1") : scalar(scalar_) {}
    std::string getValue() const {return scalar;}
    void setValue(const std::string &scalar_) {scalar = scalar_;}
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
};

class VecProperty : public StringProperty {
  private:
    std::vector<std::string> value;
  public:
    VecProperty(int size);
    VecProperty(const std::vector<std::string> &x) : value(x) {}
    ~VecProperty();
    std::vector<std::string> getVec() const {return value;}
    void setVec(const std::vector<std::string> &x) {value = x;}
    std::string getValue() const {return toStr(getVec());}
    void setValue(const std::string &str) {setVec(strToVec(str));}
    int size() const {return value.size();}
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class MatProperty : public StringProperty {

  private:
    std::vector<std::vector<std::string> > value;
  public:
    MatProperty(int rows, int cols);
    MatProperty(const std::vector<std::vector<std::string> > &A) : value(A) {}
    std::vector<std::vector<std::string> > getMat() const {return value;}
    void setMat(const std::vector<std::vector<std::string> > &A) {value = A;}
    std::string getValue() const {return toStr(getMat());}
    void setValue(const std::string &str) {setMat(strToMat(str));}
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    int rows() const {return value.size();}
    int cols() const {return value[0].size();}
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

//class SymMatProperty : public StringProperty {
//
//  private:
//    std::vector<std::vector<std::string> > value;
//  public:
//    SymMatProperty(int rows);
//    SymMatProperty(const std::vector<std::vector<std::string> > &A) : value(A) {}
//    std::vector<std::vector<std::string> > getMat() const {return value;}
//    void setMat(const std::vector<std::vector<std::string> > &A) {value = A;}
//    std::string getValue() const {return toStr(getMat());}
//    void setValue(const std::string &str) {setMat(strToMat(str));}
//    TiXmlElement* initializeUsingXML(TiXmlElement *element);
//    TiXmlElement* writeXMLFile(TiXmlNode *element);
//    int rows() const {return value.size();}
//    int cols() const {return value[0].size();}
//    void fromWidget(QWidget *widget);
//    void toWidget(QWidget *widget);
//};

class PhysicalStringProperty : public Property {
  protected:
    StringProperty* value;
    std::string unit, xmlName;
  public:
    PhysicalStringProperty(StringProperty *value_=0, const std::string &unit_="", const std::string &xmlName_="") : value(value_), unit(unit_), xmlName(xmlName_) {}
    ~PhysicalStringProperty() {delete value;}
    std::string getValue() const {return value->getValue();}
    void setValue(const std::string &str) {value->setValue(str);}
    std::string getUnit() const {return unit;}
    void setUnit(const std::string &unit_) {unit = unit_;}
    virtual StringProperty* getProperty() {return value;}
    const std::string& getXmlName() const {return xmlName;}
    void setXmlName(const std::string &name) {xmlName = name;}
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class VecFromFileProperty : public StringProperty {

  public:
    VecFromFileProperty(const QString &fileName_="", const QString &absoluteFilePath_="") : fileName(fileName_), absoluteFilePath(absoluteFilePath_) {}
    std::string getValue() const;
    void setValue(const std::string &str) {}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    QString fileName, absoluteFilePath;
};

class MatFromFileProperty : public StringProperty {

  public:
    MatFromFileProperty(const QString &fileName_="", const QString &absoluteFilePath_="") : fileName(fileName_), absoluteFilePath(absoluteFilePath_) {}
    std::string getValue() const; 
    void setValue(const std::string &str) {}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    QString fileName, absoluteFilePath;
};

#endif

