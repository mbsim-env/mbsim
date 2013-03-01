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

#include <config.h>
#include "kinematics_widgets.h"
#include "string_widgets.h"
#include "extended_widgets.h"
#include "octaveutils.h"
#include <QtGui>

using namespace std;

LinearTranslation::LinearTranslation() {
  vector<PhysicalStringWidget*> input;
  MatColsVarWidget* m = new MatColsVarWidget(3,1,1,3);
  input.push_back(new PhysicalStringWidget(m,MBSIMNS"translationVectors",noUnitUnits(),1));
  mat = new ExtPhysicalVarWidget(input);
  QWidget *widget = new ExtXMLWidget("Translation vectors",mat);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(widget);
  QObject::connect(m, SIGNAL(sizeChanged(int)), this, SIGNAL(translationChanged()));
  QObject::connect(mat, SIGNAL(inputDialogChanged(int)), this, SIGNAL(translationChanged()));
}

int LinearTranslation::getSize() const {
  string str = evalOctaveExpression(mat->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

TiXmlElement* LinearTranslation::initializeUsingXML(TiXmlElement *element) {
  return mat->initializeUsingXML(element);
}

TiXmlElement* LinearTranslation::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"LinearTranslation" );
  mat->writeXMLFile(ele2);
  parent->LinkEndChild(ele2);
  return ele2;
}

TranslationChoiceWidget::TranslationChoiceWidget(const string &xmlName_) : translation(0), xmlName(xmlName_) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("LinearTranslation"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineTranslation(int)));
  defineTranslation(0);
}

void TranslationChoiceWidget::defineTranslation(int index) {
//  if(index==0) {
//    layout->removeWidget(translation);
//    delete translation;
//    translation = 0;
//  } 
  if(index==0) {
    translation = new LinearTranslation;  
    connect((LinearTranslation*)translation, SIGNAL(translationChanged()), this, SIGNAL(translationChanged()));
  layout->addWidget(translation);
  }
  emit translationChanged();
}

TiXmlElement* TranslationChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"LinearTranslation")
        comboBox->setCurrentIndex(0);
      translation->initializeUsingXML(ee);
      return e;
    }
  }
  return 0;
}

TiXmlElement* TranslationChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    //if(getTranslation()==1) {
    translation->writeXMLFile(ele0);
    //}
    parent->LinkEndChild(ele0);
  }
  else
    translation->writeXMLFile(parent);

 return 0;
}

TiXmlElement* RotationAboutXAxis::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutXAxis" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* RotationAboutYAxis::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutYAxis" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* RotationAboutZAxis::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutZAxis" );
  parent->LinkEndChild(ele2);
  return ele2;
}

RotationAboutFixedAxis::RotationAboutFixedAxis() {
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(3),MBSIMNS"axisOfRotation",noUnitUnits(),1));
  vec = new ExtXMLWidget("Axis of rotation",new ExtPhysicalVarWidget(input));  
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  layout->addWidget(vec);
}

TiXmlElement* RotationAboutFixedAxis::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutFixedAxis" );
  vec->writeXMLFile(ele2);
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* RotationAboutFixedAxis::initializeUsingXML(TiXmlElement *element) {
  return vec->initializeUsingXML(element);
}

TiXmlElement* RotationAboutAxesXY::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutAxesXY" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* CardanAngles::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"CardanAngles" );
  parent->LinkEndChild(ele2);
  return ele2;
}

RotationChoiceWidget::RotationChoiceWidget(const string &xmlName_) : rotation(0), xmlName(xmlName_) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Rotation about x-axis"));
  comboBox->addItem(tr("Rotation about y-axis"));
  comboBox->addItem(tr("Rotation about z-axis"));
  comboBox->addItem(tr("Rotation about fixed axis"));
  comboBox->addItem(tr("Cardan angles"));
  comboBox->addItem(tr("Rotation about x- and y-axis"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineRotation(int)));
  defineRotation(0);
}

void RotationChoiceWidget::defineRotation(int index) {
//  if(index==0) {
//    layout->removeWidget(rotation);
//    delete rotation;
//    rotation = 0;
//  } 
  if(index==0) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutXAxis;  
    layout->addWidget(rotation);
  }
  else if(index==1) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutYAxis;  
    layout->addWidget(rotation);
  }
  else if(index==2) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutZAxis;  
    layout->addWidget(rotation);
  }
  else if(index==3) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutFixedAxis;  
    layout->addWidget(rotation);
  }
  else if(index==4) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new CardanAngles;  
    layout->addWidget(rotation);
  }
  else if(index==5) {
    layout->removeWidget(rotation);
    delete rotation;
    rotation = new RotationAboutAxesXY;  
    layout->addWidget(rotation);
  }
  emit rotationChanged();
}

TiXmlElement* RotationChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement *ee = e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"RotationAboutXAxis")
        comboBox->setCurrentIndex(0);
      else if(ee->ValueStr() == MBSIMNS"RotationAboutYAxis")
        comboBox->setCurrentIndex(1);
      else if(ee->ValueStr() == MBSIMNS"RotationAboutZAxis")
        comboBox->setCurrentIndex(2);
      else if(ee->ValueStr() == MBSIMNS"RotationAboutFixedAxis")
        comboBox->setCurrentIndex(3);
      else if(ee->ValueStr() == MBSIMNS"CardanAngles")
        comboBox->setCurrentIndex(4);
      else if(ee->ValueStr() == MBSIMNS"RotationAboutAxesXY")
        comboBox->setCurrentIndex(5);
      rotation->initializeUsingXML(ee);
      return e;
    }
  }
  return 0;
}

TiXmlElement* RotationChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"rotation" );
    rotation->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }
  else
    rotation->writeXMLFile(parent);

 return 0;
}
