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

using namespace std;
using namespace MBXMLUtils;

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

Solver::Solver(const string &str, Element *parent) : Group(str,parent), solverParameters(0,false), inverseKinetics(0,false) {

  vector<PhysicalVariableProperty*> input;
  vector<string> g(3);
  g[0] = "0";
  g[1] = "-9.81";
  g[2] = "0";
  input.push_back(new PhysicalVariableProperty(new VecProperty(g),"m/s^2",MBSIMNS"accelerationOfGravity"));
  environment.setProperty(new ExtPhysicalVarProperty(input));

  solverParameters.setProperty(new SolverParametersProperty); 

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"),"",MBSIMNS"inverseKinetics"));
  inverseKinetics.setProperty(new ExtPhysicalVarProperty(input));
}

void Solver::initializeUsingXML(TiXmlElement *element) {
  Group::initializeUsingXML(element);
  TiXmlElement *e;

  // search first Environment element
  e=element->FirstChildElement(MBSIMNS"environments")->FirstChildElement();
  Environment *env;
  while((env=ObjectFactory::getInstance()->getEnvironment(e))) {
    env->initializeUsingXML(e);
    environment.initializeUsingXML(e);
    e=e->NextSiblingElement();
  }

  solverParameters.initializeUsingXML(element);

  inverseKinetics.initializeUsingXML(element);
}

TiXmlElement* Solver::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Group::writeXMLFile(parent);
  ele0->SetAttribute("xmlns", "http://mbsim.berlios.de/MBSim");
  ele0->SetAttribute("xmlns:ombv", "http://openmbv.berlios.de/OpenMBV");

  TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"environments" );
  TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"MBSimEnvironment" );
  environment.writeXMLFile(ele2);
  ele1->LinkEndChild( ele2 );
  ele0->LinkEndChild( ele1 );

  solverParameters.writeXMLFile(ele0);

  inverseKinetics.writeXMLFile(ele0);

  return ele0;
}

Solver* Solver::readXMLFile(const string &filename) {
  MBSimObjectFactory::initialize();
  TiXmlDocument doc;
  if(doc.LoadFile(filename)) {
    TiXml_PostLoadFile(&doc);
    TiXmlElement *e=doc.FirstChildElement();
    map<string,string> dummy;
    incorporateNamespace(doc.FirstChildElement(), dummy);
    Solver *solver=dynamic_cast<Solver*>(ObjectFactory::getInstance()->createGroup(e,0));
    solver->initializeUsingXML(e);
    solver->initialize();
    return solver;
  }
  return 0;
}

void Solver::writeXMLFile(const string &name) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  writeXMLFile(&doc);
  unIncorporateNamespace(doc.FirstChildElement(), Utils::getMBSimNamespacePrefixMapping());  
  doc.SaveFile((name.length()>10 && name.substr(name.length()-10,10)==".mbsim.xml")?name:name+".mbsim.xml");
}
