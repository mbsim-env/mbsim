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
#include "kinematics_properties.h"
#include "frame.h"
#include "variable_properties.h"
#include "function_properties.h"
#include "extended_properties.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "kinematics_widgets.h"
#include "extended_widgets.h"
#include "octaveutils.h"
#include <mbxmlutilstinyxml/tinyxml.h>
#include <mbxmlutilstinyxml/tinynamespace.h>

using namespace std;
using namespace MBXMLUtils;

TiXmlElement* TranslationInXDirectionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"TranslationInXDirection" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* TranslationInYDirectionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"TranslationInYDirection" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* TranslationInZDirectionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"TranslationInZDirection" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* TranslationInXYDirectionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"TranslationInXYDirection" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* TranslationInXZDirectionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"TranslationInXZDirection" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* TranslationInYZDirectionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"TranslationInYZDirection" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* TranslationInXYZDirectionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"TranslationInXYZDirection" );
  parent->LinkEndChild(ele2);
  return ele2;
}

LinearTranslationProperty::LinearTranslationProperty() {
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIMNS"translationVectors"));
  mat.setProperty(new ExtPhysicalVarProperty(input));
}

int LinearTranslationProperty::getqTSize() const {
  string str = evalOctaveExpression(static_cast<const ExtPhysicalVarProperty*>(mat.getProperty())->getCurrentPhysicalVariableProperty()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

TiXmlElement* LinearTranslationProperty::initializeUsingXML(TiXmlElement *element) {
  return mat.initializeUsingXML(element);
}

TiXmlElement* LinearTranslationProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"LinearTranslation" );
  mat.writeXMLFile(ele2);
  parent->LinkEndChild(ele2);
  return ele2;
}

void LinearTranslationProperty::fromWidget(QWidget *widget) {
  mat.fromWidget(static_cast<LinearTranslationWidget*>(widget)->mat);
}

void LinearTranslationProperty::toWidget(QWidget *widget) {
  mat.toWidget(static_cast<LinearTranslationWidget*>(widget)->mat);
}

TimeDependentTranslationProperty::TimeDependentTranslationProperty() {
  function.setProperty(new Function1ChoiceProperty(MBSIMNS"translationFunction"));
}

TiXmlElement* TimeDependentTranslationProperty::initializeUsingXML(TiXmlElement *element) {
  return function.initializeUsingXML(element);
}

TiXmlElement* TimeDependentTranslationProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"TimeDependentTranslation" );
  function.writeXMLFile(ele2);
  parent->LinkEndChild(ele2);
  return ele2;
}

void TimeDependentTranslationProperty::fromWidget(QWidget *widget) {
  function.fromWidget(static_cast<TimeDependentTranslationWidget*>(widget)->function);
}

void TimeDependentTranslationProperty::toWidget(QWidget *widget) {
  function.toWidget(static_cast<TimeDependentTranslationWidget*>(widget)->function);
}

StateDependentTranslationProperty::StateDependentTranslationProperty() {
  //function.setProperty(new Function1ChoiceProperty(MBSIMNS"translationFunction",false,"VV"));
  vector<Property*> property;
  property.push_back(new SymbolicFunction1Property("VV"));
  function.setProperty(new GeneralChoiceProperty(MBSIMNS"translationFunction",property));
}

TiXmlElement* StateDependentTranslationProperty::initializeUsingXML(TiXmlElement *element) {
  return function.initializeUsingXML(element);
}

TiXmlElement* StateDependentTranslationProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"StateDependentTranslation" );
  function.writeXMLFile(ele2);
  parent->LinkEndChild(ele2);
  return ele2;
}

void StateDependentTranslationProperty::fromWidget(QWidget *widget) {
  function.fromWidget(static_cast<StateDependentTranslationWidget*>(widget)->function);
}

void StateDependentTranslationProperty::toWidget(QWidget *widget) {
  function.toWidget(static_cast<StateDependentTranslationWidget*>(widget)->function);
}

GeneralTranslationProperty::GeneralTranslationProperty() {
  //function.setProperty(new Function2ChoiceProperty(MBSIMNS"translationFunction","VVS"));
  vector<Property*> property;
  property.push_back(new SymbolicFunction2Property("VVS"));
  function.setProperty(new GeneralChoiceProperty(MBSIMNS"translationFunction",property));
}

TiXmlElement* GeneralTranslationProperty::initializeUsingXML(TiXmlElement *element) {
  return function.initializeUsingXML(element);
}

TiXmlElement* GeneralTranslationProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"GeneralTranslation" );
  function.writeXMLFile(ele2);
  parent->LinkEndChild(ele2);
  return ele2;
}

void GeneralTranslationProperty::fromWidget(QWidget *widget) {
  function.fromWidget(static_cast<GeneralTranslationWidget*>(widget)->function);
}

void GeneralTranslationProperty::toWidget(QWidget *widget) {
  function.toWidget(static_cast<GeneralTranslationWidget*>(widget)->function);
}

TranslationChoiceProperty::TranslationChoiceProperty(int index, const std::string &xmlName_) : xmlName(xmlName_), index(0) {
  translation.push_back(new TranslationInXDirectionProperty);
  translation.push_back(new TranslationInYDirectionProperty);  
  translation.push_back(new TranslationInZDirectionProperty);  
  translation.push_back(new TranslationInXYDirectionProperty);  
  translation.push_back(new TranslationInXZDirectionProperty);  
  translation.push_back(new TranslationInYZDirectionProperty);  
  translation.push_back(new TranslationInXYZDirectionProperty);  
  translation.push_back(new LinearTranslationProperty);  
  translation.push_back(new TimeDependentTranslationProperty);  
  translation.push_back(new StateDependentTranslationProperty);  
  translation.push_back(new GeneralTranslationProperty);  
}

TranslationChoiceProperty::~TranslationChoiceProperty() {
  for(unsigned int i=0; i<translation.size(); i++)
    delete translation[i];
}
  
TiXmlElement* TranslationChoiceProperty::initializeUsingXML(TiXmlElement *element) {
   TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement *ee = e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"TranslationInXDirection")
        index = 0;
      else if(ee->ValueStr() == MBSIMNS"TranslationInYDirection")
        index = 1;
      else if(ee->ValueStr() == MBSIMNS"TranslationInZDirection")
        index = 2;
      else if(ee->ValueStr() == MBSIMNS"TranslationInXYDirection")
        index = 3;
      else if(ee->ValueStr() == MBSIMNS"TranslationInXZDirection")
        index = 4;
      else if(ee->ValueStr() == MBSIMNS"TranslationInYZDirection")
        index = 5;
      else if(ee->ValueStr() == MBSIMNS"TranslationInXYZDirection")
        index = 6;
      else if(ee->ValueStr() == MBSIMNS"LinearTranslation")
        index = 7;
      else if(ee->ValueStr() == MBSIMNS"TimeDependentTranslation")
        index = 8;
      else if(ee->ValueStr() == MBSIMNS"StateDependentTranslation")
        index = 9;
      else if(ee->ValueStr() == MBSIMNS"GeneralTranslation")
        index = 10;
      translation[index]->initializeUsingXML(ee);
      return e;
    }
  }
  return 0;
}

TiXmlElement* TranslationChoiceProperty::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    translation[index]->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }
  else
    translation[index]->writeXMLFile(parent);

 return 0;
}

void TranslationChoiceProperty::fromWidget(QWidget *widget) {
  index = static_cast<TranslationChoiceWidget*>(widget)->comboBox->currentIndex();
  translation[index]->fromWidget(static_cast<TranslationChoiceWidget*>(widget)->getTranslation());
}

void TranslationChoiceProperty::toWidget(QWidget *widget) {
  static_cast<TranslationChoiceWidget*>(widget)->comboBox->blockSignals(true);
  static_cast<TranslationChoiceWidget*>(widget)->comboBox->setCurrentIndex(index);
  static_cast<TranslationChoiceWidget*>(widget)->comboBox->blockSignals(false);
  static_cast<TranslationChoiceWidget*>(widget)->blockSignals(true);
  static_cast<TranslationChoiceWidget*>(widget)->defineTranslation(index);
  static_cast<TranslationChoiceWidget*>(widget)->blockSignals(false);
  translation[index]->toWidget(static_cast<TranslationChoiceWidget*>(widget)->getTranslation());
}

TiXmlElement* RotationAboutXAxisProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutXAxis" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* RotationAboutYAxisProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutYAxis" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* RotationAboutZAxisProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutZAxis" );
  parent->LinkEndChild(ele2);
  return ele2;
}

RotationAboutFixedAxisProperty::RotationAboutFixedAxisProperty() {
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new VecProperty(3),"-",MBSIMNS"axisOfRotation"));
  vec.setProperty(new ExtPhysicalVarProperty(input));  
}

TiXmlElement* RotationAboutFixedAxisProperty::initializeUsingXML(TiXmlElement *element) {
  return vec.initializeUsingXML(element);
}

TiXmlElement* RotationAboutFixedAxisProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutFixedAxis" );
  vec.writeXMLFile(ele2);
  parent->LinkEndChild(ele2);
  return ele2;
}

void RotationAboutFixedAxisProperty::fromWidget(QWidget *widget) {
  vec.fromWidget(static_cast<RotationAboutFixedAxisWidget*>(widget)->vec);
}

void RotationAboutFixedAxisProperty::toWidget(QWidget *widget) {
  vec.toWidget(static_cast<RotationAboutFixedAxisWidget*>(widget)->vec);
}

TiXmlElement* RotationAboutAxesXYProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutAxesXY" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* RotationAboutAxesXZProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutAxesXZ" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* RotationAboutAxesYZProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutAxesYZ" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* RotationAboutAxesXYZProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"RotationAboutAxesXYZ" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* CardanAnglesProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"CardanAngles" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TiXmlElement* EulerAnglesProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"EulerAngles" );
  parent->LinkEndChild(ele2);
  return ele2;
}

TimeDependentRotationAboutFixedAxisProperty::TimeDependentRotationAboutFixedAxisProperty() {
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new VecProperty(3),"-",MBSIMNS"axisOfRotation"));
  vec.setProperty(new ExtPhysicalVarProperty(input));  

  function.setProperty(new Function1ChoiceProperty(MBSIMNS"rotationalFunction",false,"SS"));
}

TiXmlElement* TimeDependentRotationAboutFixedAxisProperty::initializeUsingXML(TiXmlElement *element) {
  vec.initializeUsingXML(element);
  return function.initializeUsingXML(element);
}

TiXmlElement* TimeDependentRotationAboutFixedAxisProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"TimeDependentRotationAboutFixedAxis" );
  vec.writeXMLFile(ele2);
  function.writeXMLFile(ele2);
  parent->LinkEndChild(ele2);
  return ele2;
}

void TimeDependentRotationAboutFixedAxisProperty::fromWidget(QWidget *widget) {
  vec.fromWidget(static_cast<TimeDependentRotationAboutFixedAxisWidget*>(widget)->vec);
  function.fromWidget(static_cast<TimeDependentRotationAboutFixedAxisWidget*>(widget)->function);
}

void TimeDependentRotationAboutFixedAxisProperty::toWidget(QWidget *widget) {
  vec.toWidget(static_cast<TimeDependentRotationAboutFixedAxisWidget*>(widget)->vec);
  function.toWidget(static_cast<TimeDependentRotationAboutFixedAxisWidget*>(widget)->function);
}

StateDependentRotationAboutFixedAxisProperty::StateDependentRotationAboutFixedAxisProperty() {
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new VecProperty(3),"-",MBSIMNS"axisOfRotation"));
  vec.setProperty(new ExtPhysicalVarProperty(input));  

  function.setProperty(new Function1ChoiceProperty(MBSIMNS"rotationalFunction",false,"SV"));
}

TiXmlElement* StateDependentRotationAboutFixedAxisProperty::initializeUsingXML(TiXmlElement *element) {
  vec.initializeUsingXML(element);
  return function.initializeUsingXML(element);
}

TiXmlElement* StateDependentRotationAboutFixedAxisProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"StateDependentRotationAboutFixedAxis" );
  vec.writeXMLFile(ele2);
  function.writeXMLFile(ele2);
  parent->LinkEndChild(ele2);
  return ele2;
}

void StateDependentRotationAboutFixedAxisProperty::fromWidget(QWidget *widget) {
  vec.fromWidget(static_cast<StateDependentRotationAboutFixedAxisWidget*>(widget)->vec);
  function.fromWidget(static_cast<StateDependentRotationAboutFixedAxisWidget*>(widget)->function);
}

void StateDependentRotationAboutFixedAxisProperty::toWidget(QWidget *widget) {
  vec.toWidget(static_cast<StateDependentRotationAboutFixedAxisWidget*>(widget)->vec);
  function.toWidget(static_cast<StateDependentRotationAboutFixedAxisWidget*>(widget)->function);
}

RotationChoiceProperty::RotationChoiceProperty(int index, const std::string &xmlName_): xmlName(xmlName_), index(0) {
  rotation.push_back(new RotationAboutXAxisProperty);
  rotation.push_back(new RotationAboutYAxisProperty);  
  rotation.push_back(new RotationAboutZAxisProperty);  
  rotation.push_back(new RotationAboutFixedAxisProperty);  
  rotation.push_back(new RotationAboutAxesXYProperty);
  rotation.push_back(new RotationAboutAxesXZProperty);  
  rotation.push_back(new RotationAboutAxesYZProperty);  
  rotation.push_back(new CardanAnglesProperty);
  rotation.push_back(new EulerAnglesProperty);
  rotation.push_back(new RotationAboutAxesXYZProperty);  
  rotation.push_back(new TimeDependentRotationAboutFixedAxisProperty);  
  rotation.push_back(new StateDependentRotationAboutFixedAxisProperty);  
}

RotationChoiceProperty::~RotationChoiceProperty() {
  for(unsigned int i=0; i<rotation.size(); i++)
    delete rotation[i];
}

TiXmlElement* RotationChoiceProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement *ee = e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"RotationAboutXAxis")
        index = 0;
      else if(ee->ValueStr() == MBSIMNS"RotationAboutYAxis")
        index = 1;
      else if(ee->ValueStr() == MBSIMNS"RotationAboutZAxis")
        index = 2;
      else if(ee->ValueStr() == MBSIMNS"RotationAboutFixedAxis")
        index = 3;
      else if(ee->ValueStr() == MBSIMNS"RotationAboutAxesXY")
        index = 4;
      else if(ee->ValueStr() == MBSIMNS"RotationAboutAxesXZ")
        index = 5;
      else if(ee->ValueStr() == MBSIMNS"RotationAboutAxesYZ")
        index = 6;
      else if(ee->ValueStr() == MBSIMNS"CardanAngles")
        index = 7;
      else if(ee->ValueStr() == MBSIMNS"EulerAngles")
        index = 8;
      else if(ee->ValueStr() == MBSIMNS"RotationAboutAxesXYZ")
        index = 9;
      else if(ee->ValueStr() == MBSIMNS"TimeDependentRotationAboutFixedAxis")
        index = 10;
      else if(ee->ValueStr() == MBSIMNS"StateDependentRotationAboutFixedAxis")
        index = 11;
      rotation[index]->initializeUsingXML(ee);
      return e;
    }
  }
  return 0;
}

TiXmlElement* RotationChoiceProperty::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"rotation" );
    rotation[index]->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }
  else
    rotation[index]->writeXMLFile(parent);

 return 0;
}

void RotationChoiceProperty::fromWidget(QWidget *widget) {
  index = static_cast<RotationChoiceWidget*>(widget)->comboBox->currentIndex();
  rotation[index]->fromWidget(static_cast<RotationChoiceWidget*>(widget)->getRotation());
}

void RotationChoiceProperty::toWidget(QWidget *widget) {
  static_cast<RotationChoiceWidget*>(widget)->comboBox->blockSignals(true);
  static_cast<RotationChoiceWidget*>(widget)->comboBox->setCurrentIndex(index);
  static_cast<RotationChoiceWidget*>(widget)->comboBox->blockSignals(false);
  static_cast<RotationChoiceWidget*>(widget)->blockSignals(true);
  static_cast<RotationChoiceWidget*>(widget)->defineRotation(index);
  static_cast<RotationChoiceWidget*>(widget)->blockSignals(false);
  rotation[index]->toWidget(static_cast<RotationChoiceWidget*>(widget)->getRotation());
}
