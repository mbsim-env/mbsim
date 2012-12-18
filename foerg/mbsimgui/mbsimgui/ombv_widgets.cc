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
#include "ombv_widgets.h"
#include "string_widgets.h"
#include "extended_widgets.h"
#include "rigidbody.h"
#include "frame.h"
#include <QtGui>

using namespace std;

void OMBVObjectWidget::writeXMLFileID(TiXmlNode *parent) {
  if(!ID.empty()) {
    TiXmlUnknown *id=new TiXmlUnknown;
    id->SetValue("?OPENMBV_ID "+ID+"?");
    parent->LinkEndChild(id);
  }
}

OMBVFrameWidget::OMBVFrameWidget(const string &name, const string &xmlName_) : OMBVObjectWidget(name), xmlName(xmlName_) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), MBSIMNS"size", lengthUnits(), 4));
  size = new ExtXMLWidget("Size",new ExtPhysicalVarWidget(input));
  layout->addWidget(size);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), MBSIMNS"offset", noUnitUnits(), 1));
  offset = new ExtXMLWidget("Offset",new ExtPhysicalVarWidget(input));
  layout->addWidget(offset);
}

bool OMBVFrameWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    size->initializeUsingXML(element);
    offset->initializeUsingXML(element);
    return true;
  }
  return false;
}

TiXmlElement* OMBVFrameWidget::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *e=new TiXmlElement(xmlName);
    writeXMLFileID(e);
    parent->LinkEndChild(e);
    size->writeXMLFile(e);
    offset->writeXMLFile(e);
    return e;
  }
  else {
    writeXMLFileID(parent);
    size->writeXMLFile(parent);
    offset->writeXMLFile(parent);
    return 0;
  }
}

OMBVArrowWidget::OMBVArrowWidget(const string &name) : OMBVObjectWidget(name) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0.1"), OPENMBVNS"diameter", lengthUnits(), 4));
  diameter = new ExtXMLWidget("Diameter",new ExtPhysicalVarWidget(input));
  layout->addWidget(diameter);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0.2"), OPENMBVNS"headDiameter", lengthUnits(), 4));
  headDiameter = new ExtXMLWidget("Head diameter",new ExtPhysicalVarWidget(input));
  layout->addWidget(headDiameter);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0.2"), OPENMBVNS"headLength", lengthUnits(), 4));
  headLength = new ExtXMLWidget("Head length",new ExtPhysicalVarWidget(input));
  layout->addWidget(headLength);

  input.clear();
  vector<string> list;
  list.push_back(string("\"")+"line"+"\"");
  list.push_back(string("\"")+"fromHead"+"\"");
  list.push_back(string("\"")+"toHead"+"\"");
  list.push_back(string("\"")+"bothHeads"+"\"");
  list.push_back(string("\"")+"formDoubleHead"+"\"");
  list.push_back(string("\"")+"toDoubleHead"+"\"");
  list.push_back(string("\"")+"bothDoubleHeads"+"\"");
  input.push_back(new PhysicalStringWidget(new ChoiceWidget(list,2), OPENMBVNS"type", QStringList(), 0));
  type = new ExtXMLWidget("Type",new ExtPhysicalVarWidget(input));
  layout->addWidget(type);

  input.clear();
  list.clear();
  list.push_back(string("\"")+"toPoint"+"\"");
  list.push_back(string("\"")+"fromPoint"+"\"");
  list.push_back(string("\"")+"midPoint"+"\"");
  input.push_back(new PhysicalStringWidget(new ChoiceWidget(list,0), OPENMBVNS"referencePoint", QStringList(), 0));
  referencePoint = new ExtXMLWidget("Reference point",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(referencePoint);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"scaleLength", noUnitUnits(), 1));
  scaleLength = new ExtXMLWidget("Scale length",new ExtPhysicalVarWidget(input));
  layout->addWidget(scaleLength);
}

bool OMBVArrowWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(OPENMBVNS+getType().toStdString());
  if(e) {
    diameter->initializeUsingXML(e);
    headDiameter->initializeUsingXML(e);
    headLength->initializeUsingXML(e);
    type->initializeUsingXML(e);
    referencePoint->initializeUsingXML(e);
    scaleLength->initializeUsingXML(e);
    return true;
  }
  return false;
}

TiXmlElement* OMBVArrowWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(OPENMBVNS+getType().toStdString());
  parent->LinkEndChild(e);
  e->SetAttribute("name", "dummy");
  writeXMLFileID(e);
  diameter->writeXMLFile(e);
  headDiameter->writeXMLFile(e);
  headLength->writeXMLFile(e);
  type->writeXMLFile(e);
  referencePoint->writeXMLFile(e);
  scaleLength->writeXMLFile(e);
  return e;
}

OMBVCoilSpringWidget::OMBVCoilSpringWidget(const string &name) : OMBVObjectWidget(name) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  vector<string> list;
  list.push_back(string("\"")+"tube"+"\"");
  list.push_back(string("\"")+"scaledTube"+"\"");
  list.push_back(string("\"")+"polyline"+"\"");
  input.push_back(new PhysicalStringWidget(new ChoiceWidget(list,0), OPENMBVNS"type", QStringList(), 0));
  type = new ExtXMLWidget("Type",new ExtPhysicalVarWidget(input));
  layout->addWidget(type);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("3"), OPENMBVNS"numberOfCoils", noUnitUnits(), 1));
  numberOfCoils= new ExtXMLWidget("Number of coils",new ExtPhysicalVarWidget(input));
  layout->addWidget(numberOfCoils);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0.1"), OPENMBVNS"springRadius", lengthUnits(), 4));
  springRadius= new ExtXMLWidget("Spring radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(springRadius);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("-1"), OPENMBVNS"crossSectionRadius", lengthUnits(), 4));
  crossSectionRadius = new ExtXMLWidget("Cross section radius",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(crossSectionRadius);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("-1"), OPENMBVNS"nominalLength", lengthUnits(), 4));
  nominalLength= new ExtXMLWidget("Nominal length",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(nominalLength);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"scaleFactor", noUnitUnits(), 1));
  scaleFactor = new ExtXMLWidget("Scale factor",new ExtPhysicalVarWidget(input));
  layout->addWidget(scaleFactor);
}

bool OMBVCoilSpringWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(OPENMBVNS+getType().toStdString());
  if(e) {
    type->initializeUsingXML(e);
    numberOfCoils->initializeUsingXML(e);
    springRadius->initializeUsingXML(e);
    crossSectionRadius->initializeUsingXML(e);
    nominalLength->initializeUsingXML(e);
    scaleFactor->initializeUsingXML(e);
    return true;
  }
  return false;
}

TiXmlElement* OMBVCoilSpringWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(OPENMBVNS+getType().toStdString());
  parent->LinkEndChild(e);
  e->SetAttribute("name", "dummy");
  writeXMLFileID(e);
  type->writeXMLFile(e);
  numberOfCoils->writeXMLFile(e);
  springRadius->writeXMLFile(e);
  crossSectionRadius->writeXMLFile(e);
  nominalLength->writeXMLFile(e);
  scaleFactor->writeXMLFile(e);
  return e;
}

OMBVBodyWidget::OMBVBodyWidget(const string &name) : OMBVObjectWidget(name) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"), OPENMBVNS"staticColor", noUnitUnits(), 1));
  color = new ExtXMLWidget("Static color",new ExtPhysicalVarWidget(input));
  layout->addWidget(color);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(3,true), OPENMBVNS"initialTranslation", lengthUnits(), 4));
  trans = new ExtXMLWidget("Initial translation",new ExtPhysicalVarWidget(input));
  layout->addWidget(trans);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(3,true), OPENMBVNS"initialRotation", angleUnits(), 0));
  rot = new ExtXMLWidget("Initial rotation",new ExtPhysicalVarWidget(input));
  layout->addWidget(rot);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"scaleFactor", noUnitUnits(), 1));
  scale = new ExtXMLWidget("Scale factor",new ExtPhysicalVarWidget(input));
  layout->addWidget(scale);
}

bool OMBVBodyWidget::initializeUsingXML(TiXmlElement *element) {
  color->initializeUsingXML(element);
  trans->initializeUsingXML(element);
  rot->initializeUsingXML(element);
  scale->initializeUsingXML(element);
}

TiXmlElement* OMBVBodyWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(OPENMBVNS+getType().toStdString());
  parent->LinkEndChild(e);
  e->SetAttribute("name", name==""?"NOTSET":name);
  writeXMLFileID(e);
  color->writeXMLFile(e);
  trans->writeXMLFile(e);
  rot->writeXMLFile(e);
  scale->writeXMLFile(e);
  return e;
}

CubeWidget::CubeWidget(const string &name) : OMBVBodyWidget(name) {

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"length", lengthUnits(), 4));
  length = new ExtXMLWidget("Length",new ExtPhysicalVarWidget(input));
  layout->addWidget(length);
}

bool CubeWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  length->initializeUsingXML(element);
}

TiXmlElement* CubeWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  length->writeXMLFile(e);
  return e;
}

CuboidWidget::CuboidWidget(const string &name) : OMBVBodyWidget(name) {

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(getScalars<string>(3,"1"),true), OPENMBVNS"length", lengthUnits(), 4));
  length = new ExtXMLWidget("Length",new ExtPhysicalVarWidget(input));
  layout->addWidget(length);
}

bool CuboidWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  length->initializeUsingXML(element);
}

TiXmlElement* CuboidWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  length->writeXMLFile(e);
  return e;
}

SphereWidget::SphereWidget(const string &name) : OMBVBodyWidget(name) {

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"radius", lengthUnits(), 4));
  radius = new ExtXMLWidget("Radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(radius);
}

bool SphereWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  radius->initializeUsingXML(element);
}

TiXmlElement* SphereWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  radius->writeXMLFile(e);
  return e;
}

FrustumWidget::FrustumWidget(const string &name) : OMBVBodyWidget(name) {

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"topRadius", lengthUnits(), 4));
  top = new ExtXMLWidget("Top radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(top);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"baseRadius", lengthUnits(), 4));
  base = new ExtXMLWidget("Base radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(base);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), OPENMBVNS"height", lengthUnits(), 4));
  height = new ExtXMLWidget("Height",new ExtPhysicalVarWidget(input));
  layout->addWidget(height);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"), OPENMBVNS"innerTopRadius", lengthUnits(), 4));
  innerTop = new ExtXMLWidget("Inner top radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(innerTop);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"), OPENMBVNS"innerBaseRadius", lengthUnits(), 4));
  innerBase = new ExtXMLWidget("Inner base radius",new ExtPhysicalVarWidget(input));
  layout->addWidget(innerBase);
}

bool FrustumWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  TiXmlElement *e;
  base->initializeUsingXML(element);
  top->initializeUsingXML(element);
  height->initializeUsingXML(element);
  innerBase->initializeUsingXML(element);
  innerTop->initializeUsingXML(element);
}

TiXmlElement* FrustumWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  base->writeXMLFile(e);
  top->writeXMLFile(e);
  height->writeXMLFile(e);
  innerBase->writeXMLFile(e);
  innerTop->writeXMLFile(e);
  return e;
}

IvBodyWidget::IvBodyWidget(const string &name) : OMBVBodyWidget(name) {

  ivFileName = new ExtXMLWidget("Iv file name",new FileWidget(OPENMBVNS"ivFileName"));
  layout->addWidget(ivFileName);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("-1"), OPENMBVNS"creaseEdges", angleUnits(), 0));
  creaseEdges = new ExtXMLWidget("Crease edges",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(creaseEdges);

  input.clear();
  input.push_back(new PhysicalStringWidget(new BoolWidget("0"), OPENMBVNS"boundaryEdges", QStringList(), 4));
  boundaryEdges = new ExtXMLWidget("Boundary edges",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(boundaryEdges);
}

bool IvBodyWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  TiXmlElement *e;
  ivFileName->initializeUsingXML(element);
  creaseEdges->initializeUsingXML(element);
  boundaryEdges->initializeUsingXML(element);
}

TiXmlElement* IvBodyWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  ivFileName->writeXMLFile(e);
  creaseEdges->writeXMLFile(e);
  boundaryEdges->writeXMLFile(e);
  return e;
}

CompoundRigidBodyWidget::CompoundRigidBodyWidget(const string &name) : OMBVBodyWidget(name) {
  QGroupBox *box = new QGroupBox("Bodies");
  QHBoxLayout *sublayout = new QHBoxLayout;
  box->setLayout(sublayout);
  layout->addWidget(box);
  bodyList = new QListWidget;
  bodyList->setContextMenuPolicy (Qt::CustomContextMenu);
  bodyList->setMinimumWidth(bodyList->sizeHint().width()/3);
  bodyList->setMaximumWidth(bodyList->sizeHint().width()/3);
  sublayout->addWidget(bodyList);
  stackedWidget = new QStackedWidget;
  connect(bodyList,SIGNAL(currentRowChanged(int)),this,SLOT(changeCurrent(int)));
  //  connect(bodyList,SIGNAL(currentRowChanged(int)),stackedWidget,SLOT(setCurrentIndex(int)));
  connect(bodyList,SIGNAL(customContextMenuRequested(const QPoint &)),this,SLOT(openContextMenu(const QPoint &)));
  sublayout->addWidget(stackedWidget);
}

void CompoundRigidBodyWidget::changeCurrent(int idx) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(idx);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
}

void CompoundRigidBodyWidget::openContextMenu(const QPoint &pos) {
  if(bodyList->itemAt(pos)) {
    QMenu menu(this);
    QAction *add = new QAction(tr("Remove"), this);
    connect(add, SIGNAL(triggered()), this, SLOT(removeBody()));
    menu.addAction(add);
    menu.exec(QCursor::pos());
  }
  else {
    QMenu menu(this);
    QAction *add = new QAction(tr("Add"), this);
    connect(add, SIGNAL(triggered()), this, SLOT(addBody()));
    menu.addAction(add);
    menu.exec(QCursor::pos());
  }
}

void CompoundRigidBodyWidget::addBody() {
  int i = body.size();
  body.push_back(new OMBVBodyChoiceWidget((QString("Body")+QString::number(i+1)).toStdString(),false));
  bodyList->addItem((QString("Body")+QString::number(i+1)));
  stackedWidget->addWidget(body[i]);
  //stackedWidget->widget(i)->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Ignored);
}

void CompoundRigidBodyWidget::removeBody() {
  int i = bodyList->currentRow();

  stackedWidget->removeWidget(body[i]);
  delete body[i];
  body.erase(body.begin()+i);
  delete bodyList->takeItem(i);
  for(int i=0; i<bodyList->count(); i++) {
    bodyList->item(i)->setText((QString("Body")+QString::number(i+1)));
    body[i]->setName(bodyList->item(i)->text().toStdString());
  }
}

bool CompoundRigidBodyWidget::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyWidget::initializeUsingXML(element);
  TiXmlElement *e=element->FirstChildElement(OPENMBVNS"scaleFactor");
  e=e->NextSiblingElement();
  while(e) {
    addBody();
    body[body.size()-1]->initializeUsingXML(e);
    e=e->NextSiblingElement();
  }
}

TiXmlElement* CompoundRigidBodyWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyWidget::writeXMLFile(parent);
  for(unsigned int i=0; i<body.size(); i++)
    body[i]->writeXMLFile(e);
  return e;
}

OMBVBodyChoiceWidget::OMBVBodyChoiceWidget(const string &name_, bool flag, const string &ID_) : ombv(0), name(name_), ID(ID_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Cube"));
  comboBox->addItem(tr("Cuboid"));
  comboBox->addItem(tr("Frustum"));
  comboBox->addItem(tr("Sphere"));
  comboBox->addItem(tr("IvBody"));
  if(flag)
    comboBox->addItem(tr("CompoundRigidBody"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(ombvSelection(int)));
  ombvSelection(0);
}

void OMBVBodyChoiceWidget::ombvSelection(int index) {
  if(index==0) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new CubeWidget(name);  
    layout->addWidget(ombv);
    ombv->update();
  }
  if(index==1) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new CuboidWidget(name);  
    layout->addWidget(ombv);
    ombv->update();
  }
  else if(index==2) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new FrustumWidget(name);  
    layout->addWidget(ombv);
    ombv->update();
  }
  else if(index==3) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new SphereWidget(name);  
    layout->addWidget(ombv);
    ombv->update();
  }
  else if(index==4) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new IvBodyWidget(name);  
    layout->addWidget(ombv);
    ombv->update();
  }
  else if(index==5) {
    layout->removeWidget(ombv);
    delete ombv;
    ombv = new CompoundRigidBodyWidget(name);  
    layout->addWidget(ombv);
    ombv->update();
  }

  ombv->setID(ID);
}

bool OMBVBodyChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  //TiXmlElement *e1 = element->FirstChildElement();
  TiXmlElement *e1 = element;
  if(e1) {
    if(e1->ValueStr() == OPENMBVNS"Cube") {
      comboBox->setCurrentIndex(0);
      ombv->initializeUsingXML(e1);
    }
    if(e1->ValueStr() == OPENMBVNS"Cuboid") {
      comboBox->setCurrentIndex(1);
      ombv->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == OPENMBVNS"Frustum") {
      comboBox->setCurrentIndex(2);
      ombv->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == OPENMBVNS"Sphere") {
      comboBox->setCurrentIndex(3);
      ombv->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == OPENMBVNS"IvBody") {
      comboBox->setCurrentIndex(4);
      ombv->initializeUsingXML(e1);
    }
    else if(e1->ValueStr() == OPENMBVNS"CompoundRigidBody") {
      comboBox->setCurrentIndex(5);
      ombv->initializeUsingXML(e1);
    }
    return true;
  }
  return false;
}

TiXmlElement* OMBVBodyChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  ombv->writeXMLFile(parent);
  return 0;
}

OMBVBodySelectionWidget::OMBVBodySelectionWidget(RigidBody *body) : ombv(0), ref(0) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  ombv = new OMBVBodyChoiceWidget("NOTSET", true, body->getID());
  ref=new LocalFrameOfReferenceWidget(MBSIMNS"frameOfReference",body);
  ExtXMLWidget *widget = new ExtXMLWidget("Frame of reference",ref);
  layout->addWidget(ombv);
  layout->addWidget(widget);
}

bool OMBVBodySelectionWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(MBSIMNS"openMBVRigidBody");
  if(e) {
    ombv->initializeUsingXML(e->FirstChildElement());
    ref->initializeUsingXML(e);
    return true;
  }
  return false;
}

TiXmlElement* OMBVBodySelectionWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"openMBVRigidBody" );
  ombv->writeXMLFile(ele0);
  if(ref->getFrame()->getName()!="C")
    ref->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);
  return 0;
}

OMBVEmptyWidget::OMBVEmptyWidget(const string &xmlName_) : OMBVObjectWidget("Empty"), xmlName(xmlName_) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
}

bool OMBVEmptyWidget::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e)
    return true;
  return false;
}

TiXmlElement* OMBVEmptyWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  writeXMLFileID(ele);
  parent->LinkEndChild(ele);
  return 0;
}

OMBVPlaneWidget::OMBVPlaneWidget(const string &xmlName_) : OMBVObjectWidget("Plane"), xmlName(xmlName_) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0.1"), MBSIMNS"size", lengthUnits(), 4));
  size = new ExtXMLWidget("Size",new ExtPhysicalVarWidget(input));
  layout->addWidget(size);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("10"), MBSIMNS"numberOfLines", QStringList(), 0));
  numberOfLines = new ExtXMLWidget("Number of lines",new ExtPhysicalVarWidget(input));
  layout->addWidget(numberOfLines);
}

bool OMBVPlaneWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    size->initializeUsingXML(e);
    numberOfLines->initializeUsingXML(e);
    return true;
  }
  return false;
}

TiXmlElement* OMBVPlaneWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(xmlName);
  writeXMLFileID(e);
  parent->LinkEndChild(e);
  size->writeXMLFile(e);
  numberOfLines->writeXMLFile(e);
  return e;
}
