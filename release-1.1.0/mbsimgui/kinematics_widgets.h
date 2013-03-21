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

#ifndef _KINEMATICS_WIDGETS_H_
#define _KINEMATICS_WIDGETS_H_

#include "xml_widget.h"

class ExtPhysicalVarWidget;
class ExtXMLWidget;
class QVBoxLayout;
class QComboBox;

class TranslationWidget : public XMLWidget {

  public:
    TranslationWidget() {}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual int getSize() const = 0;
   protected:
};

class LinearTranslation : public TranslationWidget {
  Q_OBJECT

  public:
    LinearTranslation();
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const;
  protected:
    ExtPhysicalVarWidget *mat;
  signals:
    void translationChanged();
};

class TranslationChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    TranslationChoiceWidget(const std::string &xmlName);

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const { return translation->getSize(); }

  protected slots:
    void defineTranslation(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    TranslationWidget *translation;
    std::string xmlName;
  signals:
    void translationChanged();
};

class RotationWidget : public XMLWidget {

  public:
    RotationWidget() {}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element) = 0;
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element) = 0;
    virtual int getSize() const = 0;
};

class RotationAboutXAxis : public RotationWidget {

  public:
    RotationAboutXAxis() {}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
};

class RotationAboutYAxis : public RotationWidget {

  public:
    RotationAboutYAxis() {}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
};

class RotationAboutZAxis : public RotationWidget {

  public:
    RotationAboutZAxis() {}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
};

class RotationAboutFixedAxis : public RotationWidget {

  public:
    RotationAboutFixedAxis();
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 1;}
   protected:
    ExtXMLWidget *vec;
};

class RotationAboutAxesXY : public RotationWidget {

  public:
    RotationAboutAxesXY() {}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 2;}
};

class CardanAngles : public RotationWidget {

  public:
    CardanAngles() {}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element) {}
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual int getSize() const {return 3;}
};

class RotationChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    RotationChoiceWidget(const std::string &xmlName);

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    int getSize() const { return rotation->getSize(); }

  protected slots:
   void defineRotation(int);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    RotationWidget *rotation;
    std::string xmlName;
  signals:
    void rotationChanged();
};

#endif

