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

#ifndef _KINEMATIC_FUNCTIONS_PROPERTIES_H_
#define _KINEMATIC_FUNCTIONS_PROPERTIES_H_

#include "function_property.h"
#include "extended_properties.h"

class TranslationAlongXAxisProperty: public FunctionProperty {
  public:
    TranslationAlongXAxisProperty(const std::string &name="") : FunctionProperty(name) { }
    virtual Property* clone() const {return new TranslationAlongXAxisProperty(*this);}
    int getArgSize(int i=0) const {return 1;}
    inline std::string getType() const { return "TranslationAlongXAxis"; }
};

class TranslationAlongYAxisProperty: public FunctionProperty {
  public:
    TranslationAlongYAxisProperty(const std::string &name="") : FunctionProperty(name) { }
    virtual Property* clone() const {return new TranslationAlongYAxisProperty(*this);}
    int getArgSize(int i=0) const {return 1;}
    inline std::string getType() const { return "TranslationAlongYAxis"; }
};

class TranslationAlongZAxisProperty: public FunctionProperty {
  public:
    TranslationAlongZAxisProperty(const std::string &name="") : FunctionProperty(name) { }
    virtual Property* clone() const {return new TranslationAlongZAxisProperty(*this);}
    int getArgSize(int i=0) const {return 1;}
    inline std::string getType() const { return "TranslationAlongZAxis"; }
};

class TranslationAlongAxesXYProperty: public FunctionProperty {
  public:
    TranslationAlongAxesXYProperty(const std::string &name="") : FunctionProperty(name) { }
    virtual Property* clone() const {return new TranslationAlongAxesXYProperty(*this);}
    int getArgSize(int i=0) const {return 2;}
    inline std::string getType() const { return "TranslationAlongAxesXY"; }
};

class TranslationAlongAxesYZProperty: public FunctionProperty {
  public:
    TranslationAlongAxesYZProperty(const std::string &name="") : FunctionProperty(name) { }
    virtual Property* clone() const {return new TranslationAlongAxesYZProperty(*this);}
    int getArgSize(int i=0) const {return 2;}
    inline std::string getType() const { return "TranslationAlongAxesYZ"; }
};

class TranslationAlongAxesXZProperty: public FunctionProperty {
  public:
    TranslationAlongAxesXZProperty(const std::string &name="") : FunctionProperty(name) { }
    virtual Property* clone() const {return new TranslationAlongAxesXZProperty(*this);}
    int getArgSize(int i=0) const {return 2;}
    inline std::string getType() const { return "TranslationAlongAxesXZ"; }
};

class TranslationAlongAxesXYZProperty: public FunctionProperty {
  public:
    TranslationAlongAxesXYZProperty(const std::string &name="") : FunctionProperty(name) { }
    virtual Property* clone() const {return new TranslationAlongAxesXYZProperty(*this);}
    int getArgSize(int i=0) const {return 3;}
    inline std::string getType() const { return "TranslationAlongAxesXYZ"; }
};

class TranslationAlongFixedAxisProperty : public FunctionProperty {
  public:
    TranslationAlongFixedAxisProperty(const std::string &name="");
    virtual Property* clone() const {return new TranslationAlongFixedAxisProperty(*this);}
    int getArgSize(int i=0) const {return 1;}
    inline std::string getType() const { return "TranslationAlongFixedAxis"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class LinearTranslationProperty : public FunctionProperty {
  public:
    LinearTranslationProperty(const std::string &name="");
    virtual Property* clone() const {return new LinearTranslationProperty(*this);}
    int getArgSize(int i=0) const;
    inline std::string getType() const { return "LinearTranslation"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
};

class RotationAboutXAxisProperty : public FunctionProperty {
  public:
    RotationAboutXAxisProperty() { }
    virtual Property* clone() const {return new RotationAboutXAxisProperty(*this);}
    int getArgSize(int i=0) const {return 1;}
    inline std::string getType() const { return "RotationAboutXAxis"; }
};

class RotationAboutYAxisProperty : public FunctionProperty {
  public:
    RotationAboutYAxisProperty() { }
    virtual Property* clone() const {return new RotationAboutYAxisProperty(*this);}
    int getArgSize(int i=0) const {return 1;}
    inline std::string getType() const { return "RotationAboutYAxis"; }
};

class RotationAboutZAxisProperty : public FunctionProperty {
  public:
    RotationAboutZAxisProperty() { }
    virtual Property* clone() const {return new RotationAboutZAxisProperty(*this);}
    int getArgSize(int i=0) const {return 1;}
    inline std::string getType() const { return "RotationAboutZAxis"; }
};

class RotationAboutAxesXYProperty : public FunctionProperty {
  public:
    RotationAboutAxesXYProperty() { }
    virtual Property* clone() const {return new RotationAboutAxesXYProperty(*this);}
    int getArgSize(int i=0) const {return 2;}
    inline std::string getType() const { return "RotationAboutAxesXY"; }
};

class RotationAboutAxesYZProperty : public FunctionProperty {
  public:
    RotationAboutAxesYZProperty() { }
    virtual Property* clone() const {return new RotationAboutAxesYZProperty(*this);}
    int getArgSize(int i=0) const {return 2;}
    inline std::string getType() const { return "RotationAboutAxesYZ"; }
};

class RotationAboutAxesXZProperty : public FunctionProperty {
  public:
    RotationAboutAxesXZProperty() { }
    virtual Property* clone() const {return new RotationAboutAxesXZProperty(*this);}
    int getArgSize(int i=0) const {return 2;}
    inline std::string getType() const { return "RotationAboutAxesXZ"; }
};

class RotationAboutAxesXYZProperty : public FunctionProperty {
  public:
    RotationAboutAxesXYZProperty() { }
    virtual Property* clone() const {return new RotationAboutAxesXYZProperty(*this);}
    int getArgSize(int i=0) const {return 3;}
    inline std::string getType() const { return "RotationAboutAxesXYZ"; }
};

class RotationAboutFixedAxisProperty : public FunctionProperty {
  public:
    RotationAboutFixedAxisProperty();
    virtual Property* clone() const {return new RotationAboutFixedAxisProperty(*this);}
    int getArgSize(int i=0) const {return 1;}
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
    TCardanAnglesProperty() { }
    virtual Property* clone() const {return new TCardanAnglesProperty(*this);}
    int getArgSize(int i=0) const {return 3;}
    inline std::string getType() const { return "TCardanAngles"; }
};

#endif
