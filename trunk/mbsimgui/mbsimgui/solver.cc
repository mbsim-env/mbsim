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
#include "solver.h"
#include <QtGui/QMenu>
#include "objectfactory.h"
#include <string>
#include "basic_properties.h"
#include "basic_widgets.h"
#include "string_widgets.h"
#include "extended_widgets.h"

using namespace std;

TiXmlElement* Element::copiedElement;

void Environment::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"accelerationOfGravity");
  //setAccelerationOfGravity(Element::getVec3(e));
}

TiXmlElement* Environment::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement* ele0 = new TiXmlElement( MBSIMNS"MBSimEnvironment" );

  TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"accelerationOfGravity" );
  //TiXmlText *text;// = new TiXmlText( mat2str(getAccelerationOfGravity()) );
  //ele1->LinkEndChild(text);
  ele0->LinkEndChild( ele1 );
  parent->LinkEndChild(ele0);
  return ele0;
}

Environment::Environment() {}
Environment::~Environment() {}

Environment *Environment::instance=NULL;

Solver::Solver(const QString &str, QTreeWidgetItem *parentItem, int ind) : Group(str, parentItem, ind), solverParametersProperty(0,false), inverseKineticsProperty(0,false) {

  setText(1,getType());

  Element::copiedElement = 0;

  vector<PhysicalStringProperty*> input;
  vector<string> g(3);
  g[0] = "0";
  g[1] = "-9.81";
  g[2] = "0";
  input.push_back(new PhysicalStringProperty(new VecProperty(g),"m/s^2",MBSIMNS"accelerationOfGravity"));
  environmentProperty.setProperty(new ExtPhysicalVarProperty(input));

  solverParametersProperty.setProperty(new SolverParametersProperty); 

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1"),"",MBSIMNS"inverseKinetics"));
  inverseKineticsProperty.setProperty(new ExtPhysicalVarProperty(input));
}

void Solver::initializeDialog() {
  Group::initializeDialog();

  dialog->addTab("Environment");
  dialog->addTab("Solver parameters");
  dialog->addTab("Extra");

  vector<PhysicalStringWidget*> input;
  vector<string> g(3);
  g[0] = "0";
  g[1] = "-9.81";
  g[2] = "0";
  input.push_back(new PhysicalStringWidget(new VecWidget(g),accelerationUnits(),0));
  environmentWidget = new ExtWidget("Acceleration of gravity",new ExtPhysicalVarWidget(input));
  dialog->addToTab("Environment", environmentWidget);

  solverParametersWidget = new ExtWidget("Solver parameters",new SolverParametersWidget,true); 
  dialog->addToTab("Solver parameters",solverParametersWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new BoolWidget("1"),QStringList(),1));
  inverseKineticsWidget = new ExtWidget("Inverse kinetics",new ExtPhysicalVarWidget(input),true); 
  dialog->addToTab("Extra", inverseKineticsWidget);
}

void Solver::toWidget() {
  Group::toWidget();
  environmentProperty.toWidget(environmentWidget);
  solverParametersProperty.toWidget(solverParametersWidget);
  inverseKineticsProperty.toWidget(inverseKineticsWidget);
}

void Solver::fromWidget() {
  Group::fromWidget();
  environmentProperty.fromWidget(environmentWidget);
  solverParametersProperty.fromWidget(solverParametersWidget);
  inverseKineticsProperty.fromWidget(inverseKineticsWidget);
}

void Solver::initializeUsingXML(TiXmlElement *element) {
  Group::initializeUsingXML(element);
  TiXmlElement *e;

  // search first Environment element
  e=element->FirstChildElement(MBSIMNS"environments")->FirstChildElement();
  Environment *env;
  while((env=ObjectFactory::getInstance()->getEnvironment(e))) {
    env->initializeUsingXML(e);
    environmentProperty.initializeUsingXML(e);
    e=e->NextSiblingElement();
  }

  solverParametersProperty.initializeUsingXML(element);

  inverseKineticsProperty.initializeUsingXML(element);
}

TiXmlElement* Solver::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Group::writeXMLFile(parent);
  ele0->SetAttribute("xmlns", "http://mbsim.berlios.de/MBSim");
  ele0->SetAttribute("xmlns:ombv", "http://openmbv.berlios.de/OpenMBV");

  TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"environments" );
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"MBSimEnvironment" );
  environmentProperty.writeXMLFile(ele2);
  ele1->LinkEndChild( ele2 );
  ele0->LinkEndChild( ele1 );

  solverParametersProperty.writeXMLFile(ele0);

  inverseKineticsProperty.writeXMLFile(ele0);

  return ele0;
}

Solver* Solver::readXMLFile(const QString &filename, QTreeWidgetItem* parent) {
  MBSimObjectFactory::initialize();
  TiXmlDocument doc;
  bool ret=doc.LoadFile(filename.toAscii().data());
  assert(ret==true);
  TiXml_PostLoadFile(&doc);
  TiXmlElement *e=doc.FirstChildElement();
  map<string,string> dummy;
  incorporateNamespace(doc.FirstChildElement(), dummy);
  Solver *solver=dynamic_cast<Solver*>(ObjectFactory::getInstance()->createGroup(e, parent, 1));
  solver->initializeUsingXML(doc.FirstChildElement());
  solver->initialize();
  return solver;
}

void Solver::writeXMLFile(const QString &name) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  writeXMLFile(&doc);
  unIncorporateNamespace(doc.FirstChildElement(), Utils::getMBSimNamespacePrefixMapping());  
  doc.SaveFile((name.right(10)==".mbsim.xml"?name:name+".mbsim.xml").toAscii().data());
}
