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

#ifndef _KINEMATICS_PROPERTIES_H_
#define _KINEMATICS_PROPERTIES_H_

#include <string>
#include "utils.h"
#include "extended_properties.h"

namespace MBXMLUtils {
  class TiXmlElement;
  class TiXmlNode;
}

class TranslationProperty : public Property {

  public:
    TranslationProperty() {}
    virtual int getSize() const = 0;
};

class TranslationInXDirectionProperty : public TranslationProperty {

  public:
    TranslationInXDirectionProperty() {}
    int getSize() const {return 1;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class TranslationInYDirectionProperty : public TranslationProperty {

  public:
    TranslationInYDirectionProperty() {}
    int getSize() const {return 1;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class TranslationInZDirectionProperty : public TranslationProperty {

  public:
    TranslationInZDirectionProperty() {}
    int getSize() const {return 1;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class TranslationInXYDirectionProperty : public TranslationProperty {

  public:
    TranslationInXYDirectionProperty() {}
    int getSize() const {return 2;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class TranslationInXZDirectionProperty : public TranslationProperty {

  public:
    TranslationInXZDirectionProperty() {}
    int getSize() const {return 2;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class TranslationInYZDirectionProperty : public TranslationProperty {

  public:
    TranslationInYZDirectionProperty() {}
    int getSize() const {return 2;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class TranslationInXYZDirectionProperty : public TranslationProperty {

  public:
    TranslationInXYZDirectionProperty() {}
    int getSize() const {return 3;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class LinearTranslationProperty : public TranslationProperty {

  public:
    LinearTranslationProperty();
    int getSize() const;
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty mat;
};

class TimeDependentTranslationProperty : public TranslationProperty {

  public:
    TimeDependentTranslationProperty();
    int getSize() const {return 0;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty function;
};

class StateDependentTranslationProperty : public TranslationProperty {

  public:
    StateDependentTranslationProperty();
    int getSize() const {return 0;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty function;
};

class GeneralTranslationProperty : public TranslationProperty {

  public:
    GeneralTranslationProperty();
    int getSize() const {return 0;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty function;
};

class TranslationChoiceProperty : public Property {

  public:
    TranslationChoiceProperty(int index, const std::string &xmlName_): translation(0), xmlName(xmlName_) {defineTranslation(index);}

    int getSize() const { return translation->getSize(); }

    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void defineTranslation(int);

  protected:
    TranslationProperty *translation;
    std::string xmlName;
    int index;
};

class RotationProperty : public Property {

  public:
    RotationProperty() {}
    virtual int getSize() const = 0;
};

class RotationAboutXAxisProperty : public RotationProperty {

  public:
    RotationAboutXAxisProperty() {}
    int getSize() const {return 1;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RotationAboutYAxisProperty : public RotationProperty {

  public:
    RotationAboutYAxisProperty() {}
    int getSize() const {return 1;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RotationAboutZAxisProperty : public RotationProperty {

  public:
    RotationAboutZAxisProperty() {}
    int getSize() const {return 1;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RotationAboutFixedAxisProperty : public RotationProperty {

  public:
    RotationAboutFixedAxisProperty();
    int getSize() const {return 1;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget); 
    void toWidget(QWidget *widget); 
   protected:
    ExtProperty vec;
};

class RotationAboutAxesXYProperty : public RotationProperty {

  public:
    RotationAboutAxesXYProperty() {}
    int getSize() const {return 2;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RotationAboutAxesXZProperty : public RotationProperty {

  public:
    RotationAboutAxesXZProperty() {}
    int getSize() const {return 2;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RotationAboutAxesYZProperty : public RotationProperty {

  public:
    RotationAboutAxesYZProperty() {}
    int getSize() const {return 2;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RotationAboutAxesXYZProperty : public RotationProperty {

  public:
    RotationAboutAxesXYZProperty() {}
    int getSize() const {return 3;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class CardanAnglesProperty : public RotationProperty {

  public:
    CardanAnglesProperty() {}
    int getSize() const {return 3;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class EulerAnglesProperty : public RotationProperty {

  public:
    EulerAnglesProperty() {}
    int getSize() const {return 3;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RotationChoiceProperty : public Property {

  public:
    RotationChoiceProperty(int index, const std::string &xmlName_): rotation(0), xmlName(xmlName_) {defineRotation(index);}

    int getSize() const { return rotation->getSize(); }

    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void defineRotation(int);

  protected:
    RotationProperty *rotation;
    std::string xmlName;
    int index;
};

#endif

