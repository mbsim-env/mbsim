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

class TiXmlElement;
class TiXmlNode;

class TranslationProperty : public Property {

  public:
    TranslationProperty() {}
    virtual int getSize() const = 0;
};

class LinearTranslationProperty : public TranslationProperty {

  public:
    LinearTranslationProperty();
    int getSize() const;
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty mat;
};

class TranslationChoiceProperty : public Property {

  public:
    TranslationChoiceProperty(TranslationProperty* translation_, const std::string &xmlName_): translation(translation_), xmlName(xmlName_) {}

    int getSize() const { return translation->getSize(); }

    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    TranslationProperty *translation;
    std::string xmlName;
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
    TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RotationAboutYAxisProperty : public RotationProperty {

  public:
    RotationAboutYAxisProperty() {}
    int getSize() const {return 1;}
    TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RotationAboutZAxisProperty : public RotationProperty {

  public:
    RotationAboutZAxisProperty() {}
    int getSize() const {return 1;}
    TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RotationAboutFixedAxisProperty : public RotationProperty {

  public:
    RotationAboutFixedAxisProperty();
    int getSize() const {return 1;}
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget); 
    void toWidget(QWidget *widget); 
   protected:
    ExtProperty vec;
};

class RotationAboutAxesXYProperty : public RotationProperty {

  public:
    RotationAboutAxesXYProperty() {}
    int getSize() const {return 2;}
    TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class CardanAnglesProperty : public RotationProperty {

  public:
    CardanAnglesProperty() {}
    int getSize() const {return 2;}
    TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
};

class RotationChoiceProperty : public Property {

  public:
    RotationChoiceProperty(RotationProperty* rotation_, const std::string &xmlName_): rotation(rotation_), xmlName(xmlName_), index(0) {}

    int getSize() const { return rotation->getSize(); }

    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void defineRotation(int);

  protected:
    RotationProperty *rotation;
    std::string xmlName;
    int index;
};

#endif

