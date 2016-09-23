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
#include <QFileInfo>

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

namespace MBSimGUI {

  class VariableProperty : public Property {

    public:
      virtual std::string getValue() const = 0;
      virtual void setValue(const std::string &str) = 0;
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      virtual int rows() const { return 1; }
      virtual int cols() const { return 1; }
  };

  class ExpressionProperty : public VariableProperty {
    public:
      ExpressionProperty(const std::string &value_="") : value(value_) {}
      virtual PropertyInterface* clone() const {return new ExpressionProperty(*this);}
      std::string getValue() const { return value; }
      void setValue(const std::string &str) { value = str; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      std::vector<std::vector<std::string> > getMat() const;
      int rows() const { return getMat().size(); }
      int cols() const { return getMat().size()?getMat()[0].size():0; }

    private:
      std::string value;
  };

  class ScalarProperty : public VariableProperty {
    protected:
      std::string scalar;
    public:
      ScalarProperty(const std::string &scalar_="1") : scalar(scalar_) {}
      virtual PropertyInterface* clone() const {return new ScalarProperty(*this);}
      std::string getValue() const {return scalar;}
      void setValue(const std::string &scalar_) {scalar = scalar_;}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
  };

  class VecProperty : public VariableProperty {
    private:
      std::vector<std::string> value;
    public:
      VecProperty(int size);
      VecProperty(const std::vector<std::string> &x) : value(x) {}
      ~VecProperty();
      virtual PropertyInterface* clone() const {return new VecProperty(*this);}
      const std::vector<std::string>& getVec() const {return value;}
      void setVec(const std::vector<std::string> &x) {value = x;}
      std::string getValue() const {return toStr(getVec());}
      void setValue(const std::string &str) {setVec(strToVec(str));}
      int size() const {return value.size();}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
  };

  class MatProperty : public VariableProperty {

    private:
      std::vector<std::vector<std::string> > value;
    public:
      MatProperty(int rows, int cols);
      MatProperty(const std::vector<std::vector<std::string> > &A) : value(A) {}
      virtual PropertyInterface* clone() const {return new MatProperty(*this);}
      const std::vector<std::vector<std::string> >& getMat() const {return value;}
      void setMat(const std::vector<std::vector<std::string> > &A) {value = A;}
      std::string getValue() const {return toStr(getMat());}
      void setValue(const std::string &str) {setMat(strToMat(str));}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      int rows() const {return value.size();}
      int cols() const {return value[0].size();}
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
  };

  class CardanProperty : public VariableProperty {

    private:
      std::vector<std::string> angles;
      std::string unit; 
    public:
      CardanProperty();
      ~CardanProperty();
      virtual PropertyInterface* clone() const {return new CardanProperty(*this);}
      const std::vector<std::string>& getAngles() const {return angles;}
      void setAngles(const std::vector<std::string> &x) {angles = x;}
      std::string getValue() const {return toStr(getAngles());}
      void setValue(const std::string &str) {setAngles(strToVec(str));}
      int size() const {return angles.size();}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
  };

  class AboutZProperty : public VariableProperty {

    private:
      std::string angle;
      std::string unit; 
    public:
      AboutZProperty();
      ~AboutZProperty();
      virtual PropertyInterface* clone() const {return new AboutZProperty(*this);}
      std::string getValue() const {return angle;}
      void setValue(const std::string &angle_) {angle = angle_;}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
  };

  class PhysicalVariableProperty : public Property {
    protected:
      VariableProperty* value;
      std::string unit;
      MBXMLUtils::FQN xmlName;
    public:
      PhysicalVariableProperty(VariableProperty *value_=0, const std::string &unit_="", const MBXMLUtils::FQN &xmlName_="") : value(value_), unit(unit_), xmlName(xmlName_) {}
      PhysicalVariableProperty(const PhysicalVariableProperty &p) : value(static_cast<VariableProperty*>(p.value->clone())), unit(p.unit), xmlName(p.xmlName) {}
      ~PhysicalVariableProperty() {delete value;}
      PhysicalVariableProperty& operator=(const PhysicalVariableProperty &p) {delete value; value=static_cast<VariableProperty*>(p.value->clone()); unit=p.unit; xmlName=p.xmlName; return *this; }
      virtual PropertyInterface* clone() const {return new PhysicalVariableProperty(*this);}
      std::string getValue() const {return value->getValue();}
      void setValue(const std::string &str) {value->setValue(str);}
      std::string getUnit() const {return unit;}
      void setUnit(const std::string &unit_) {unit = unit_;}
      virtual VariableProperty* getProperty() {return value;}
      const MBXMLUtils::FQN& getXmlName() const {return xmlName;}
      void setXmlName(const MBXMLUtils::FQN &name) {xmlName = name;}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      int rows() const { return value->rows(); }
      int cols() const { return value->cols(); }
  };

  //class VecFromFileProperty : public VariableProperty {
  //
  //  public:
  //    VecFromFileProperty(const std::string &file_="") : file(file_) {}
  //    virtual PropertyInterface* clone() const {return new VecFromFileProperty(*this);}
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

  class FromFileProperty : public VariableProperty {

    public:
      FromFileProperty(const std::string &file_="") : file(file_) {}
      virtual PropertyInterface* clone() const {return new FromFileProperty(*this);}
      void setValue(const std::string &str) {}
      std::string getValue() const { return ""; }
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      std::string getFile() const { return file; }
      void setFile(const std::string &str) { file = str; }

    protected:
      std::string file;
  };

  class ScalarPropertyFactory: public PropertyFactory {
    public:
      ScalarPropertyFactory(const std::string &value, const MBXMLUtils::FQN &xmlName);
      ScalarPropertyFactory(const std::string &value, const MBXMLUtils::FQN &xmlName, const std::vector<std::string> &unit);
      Property* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::string value;
      std::vector<std::string> name;
      MBXMLUtils::FQN xmlName;
      std::vector<std::string> unit;
  };

  class VecPropertyFactory: public PropertyFactory {
    public:
      VecPropertyFactory(int m, const MBXMLUtils::FQN &xmlName);
      VecPropertyFactory(int m, const MBXMLUtils::FQN &xmlName, const std::vector<std::string> &unit);
      VecPropertyFactory(const std::vector<std::string> &x, const MBXMLUtils::FQN &xmlName, const std::vector<std::string> &unit);
      Property* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<std::string> x;
      std::vector<std::string> name;
      MBXMLUtils::FQN xmlName;
      std::vector<std::string> unit;
  };

  class RotMatPropertyFactory: public PropertyFactory {
    public:
      RotMatPropertyFactory(const MBXMLUtils::FQN &xmlName);
      RotMatPropertyFactory(const MBXMLUtils::FQN &xmlName, const std::vector<std::string> &unit);
      Property* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<std::string> name;
      MBXMLUtils::FQN xmlName;
      std::vector<std::string> unit;
  };

  class MatPropertyFactory: public PropertyFactory {
    public:
      MatPropertyFactory(const MBXMLUtils::FQN &xmlName);
      MatPropertyFactory(const std::vector<std::vector<std::string> > &A, const MBXMLUtils::FQN &xmlName, const std::vector<std::string> &unit);
      Property* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<std::vector<std::string> > A;
      std::vector<std::string> name;
      MBXMLUtils::FQN xmlName;
      std::vector<std::string> unit;
  };

}

#endif

