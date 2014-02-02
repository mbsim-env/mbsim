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
#include "mainwindow.h"
#include <QtGui/QMenu>
#include "objectfactory.h"
#include <string>
#include "basic_properties.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

void Environment::initializeUsingXML(DOMElement *element) {
  DOMElement *e;
  e=E(element)->getFirstElementChildNamed(MBSIM%"accelerationOfGravity");
  //setAccelerationOfGravity(Element::getVec3(e));
}

DOMElement* Environment::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement* ele0 = D(doc)->createElement( MBSIM%"MBSimEnvironment" );

  DOMElement *ele1 = D(doc)->createElement( MBSIM%"accelerationOfGravity" );
  //DOMText *text;// = new DOMText( mat2str(getAccelerationOfGravity()) );
  //ele1->insertBefore(text, NULL);
  ele0->insertBefore( ele1, NULL );
  parent->insertBefore(ele0, NULL);
  return ele0;
}

Environment::Environment() {}
Environment::~Environment() {}

Environment *Environment::instance=NULL;

Solver::Solver(const string &str, Element *parent) : Group(str,parent), solverParameters(0,false), inverseKinetics(0,false) {
  type = "DynamicSystemSolver";

  vector<PhysicalVariableProperty> input;
  vector<string> g(3);
  g[0] = "0";
  g[1] = "-9.81";
  g[2] = "0";
//  input.push_back(PhysicalVariableProperty(new VecProperty(g),"m/s^2",MBSIM%"accelerationOfGravity"));
//  environment.setProperty(new ExtPhysicalVarProperty(input));
  property.push_back(new ChoiceProperty2("accelerationOfGravity",new VecPropertyFactory(g,AccelerationUnits()),"",4));
//  environment.setProperty(new ChoiceProperty2(new VecPropertyFactory(g,MBSIM%"accelerationOfGravity",AccelerationUnits()),MBSIM%"accelerationOfGravity",0));

  solverParameters.setProperty(new SolverParametersProperty); 

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"",MBSIM%"inverseKinetics"));
  inverseKinetics.setProperty(new ExtPhysicalVarProperty(input));
}

void Solver::initializeUsingXML(DOMElement *element) {
  Group::initializeUsingXML(element);
  DOMElement *e;

  // search first Environment element
  e=E(element)->getFirstElementChildNamed(MBSIM%"environments")->getFirstElementChild();
  Environment *env;
  while((env=ObjectFactory::getInstance()->getEnvironment(e))) {
    env->initializeUsingXML(e);
    DOMElement* ele1 = E(e)->getFirstElementChildNamed( MBSIM%"accelerationOfGravity" );
    property[0]->initializeUsingXML(ele1);
    e=e->getNextElementSibling();
  }

  solverParameters.initializeUsingXML(element);

  inverseKinetics.initializeUsingXML(element);
}

DOMElement* Solver::writeXMLFile(DOMNode *parent) {
  DOMElement *ele0 = Group::writeXMLFile(parent);

  DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
  DOMElement *ele1 = D(doc)->createElement( MBSIM%"environments" );
  DOMElement *ele2 = D(doc)->createElement( MBSIM%"MBSimEnvironment" );
  DOMElement* ele3 = D(doc)->createElement( MBSIM%"accelerationOfGravity" );
  property[0]->writeXMLFile(ele3);
  ele2->insertBefore(ele3, NULL);
  ele1->insertBefore( ele2, NULL );
  ele0->insertBefore( ele1, NULL );

  solverParameters.writeXMLFile(ele0);

  inverseKinetics.writeXMLFile(ele0);

  return ele0;
}

Solver* Solver::readXMLFile(const string &filename) {
  MBSimObjectFactory::initialize();
  shared_ptr<DOMDocument> doc=MainWindow::parser->parse(filename);
  DOMElement *e=doc->getDocumentElement();
  Solver *solver=dynamic_cast<Solver*>(ObjectFactory::getInstance()->createGroup(e, 0));
  solver->initializeUsingXML(e);
  solver->initialize();
  return solver;
}

void Solver::writeXMLFile(const string &name) {
  shared_ptr<DOMDocument> doc=MainWindow::parser->createDocument();
  writeXMLFile(doc.get());
  DOMParser::serialize(doc.get(), (name.length()>10 && name.substr(name.length()-10,10)==".mbsim.xml")?name:name+".mbsim.xml");
}
