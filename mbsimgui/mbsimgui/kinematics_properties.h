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
    virtual int getqSize() const {return 0;}
    virtual int getuSize() const {return getqSize();}
    virtual int getqTSize() const {return 0;}
    virtual int getuTSize() const {return getqTSize();}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RotationIndependentTranslationProperty : public TranslationProperty {
};

class TranslationInXDirectionProperty : public RotationIndependentTranslationProperty {

  public:
    int getqTSize() const {return 1;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class TranslationInYDirectionProperty : public RotationIndependentTranslationProperty {

  public:
    int getqTSize() const {return 1;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class TranslationInZDirectionProperty : public RotationIndependentTranslationProperty {

  public:
    int getqTSize() const {return 1;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class TranslationInXYDirectionProperty : public RotationIndependentTranslationProperty {

  public:
    int getqTSize() const {return 2;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class TranslationInXZDirectionProperty : public RotationIndependentTranslationProperty {

  public:
    int getqTSize() const {return 2;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class TranslationInYZDirectionProperty : public RotationIndependentTranslationProperty {

  public:
    int getqTSize() const {return 2;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class TranslationInXYZDirectionProperty : public RotationIndependentTranslationProperty {

  public:
    int getqTSize() const {return 3;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class LinearTranslationProperty : public RotationIndependentTranslationProperty {

  public:
    LinearTranslationProperty();
    int getqTSize() const;
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty mat;
};

class TimeDependentTranslationProperty : public RotationIndependentTranslationProperty {

  public:
    TimeDependentTranslationProperty();
    int getqTSize() const {return 0;}
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
    int getqSize() const {return 0;}
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
    int getqSize() const {return 0;}
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

    int getqSize() const { return translation->getqSize(); }
    int getuSize() const { return translation->getuSize(); }
    int getqTSize() const { return translation->getqTSize(); }
    int getuTSize() const { return translation->getuTSize(); }

    bool isIndependent() const {return dynamic_cast<RotationIndependentTranslationProperty*>(translation)!=NULL;}

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
    virtual int getqSize() const {return 0;}
    virtual int getuSize() const {return getqSize();}
    virtual int getqRSize() const {return 0;}
    virtual int getuRSize() const {return getqRSize();}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class TranslationIndependentRotationProperty: public RotationProperty {
};

class RotationAboutXAxisProperty : public TranslationIndependentRotationProperty {

  public:
    int getqRSize() const {return 1;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class RotationAboutYAxisProperty : public TranslationIndependentRotationProperty {

  public:
    int getqRSize() const {return 1;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class RotationAboutZAxisProperty : public TranslationIndependentRotationProperty {

  public:
    int getqRSize() const {return 1;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class RotationAboutFixedAxisProperty : public TranslationIndependentRotationProperty {

  public:
    RotationAboutFixedAxisProperty();
    int getqRSize() const {return 1;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget); 
    void toWidget(QWidget *widget); 
   protected:
    ExtProperty vec;
};

class RotationAboutAxesXYProperty : public TranslationIndependentRotationProperty {

  public:
    int getqRSize() const {return 2;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class RotationAboutAxesXZProperty : public TranslationIndependentRotationProperty {

  public:
    int getqRSize() const {return 2;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class RotationAboutAxesYZProperty : public TranslationIndependentRotationProperty {

  public:
    int getqRSize() const {return 2;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class RotationAboutAxesXYZProperty : public TranslationIndependentRotationProperty {

  public:
    int getqRSize() const {return 3;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class CardanAnglesProperty : public TranslationIndependentRotationProperty {

  public:
    int getqRSize() const {return 3;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class EulerAnglesProperty : public TranslationIndependentRotationProperty {

  public:
    int getqRSize() const {return 3;}
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
};

class TimeDependentRotationAboutFixedAxisProperty : public TranslationIndependentRotationProperty {

  public:
    TimeDependentRotationAboutFixedAxisProperty();
    int getqRSize() const {return 0;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty vec, function;
};

class StateDependentRotationAboutFixedAxisProperty : public RotationProperty {

  public:
    StateDependentRotationAboutFixedAxisProperty();
    int getqSize() const {return 0;}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty vec, function;
};

class RotationChoiceProperty : public Property {

  public:
    RotationChoiceProperty(int index, const std::string &xmlName_): rotation(0), xmlName(xmlName_) {defineRotation(index);}

    int getqSize() const { return rotation->getqSize(); }
    int getuSize() const { return rotation->getuSize(); }
    int getqRSize() const { return rotation->getqRSize(); }
    int getuRSize() const { return rotation->getuRSize(); }

    bool isIndependent() const {return dynamic_cast<TranslationIndependentRotationProperty*>(rotation)!=NULL;}

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

