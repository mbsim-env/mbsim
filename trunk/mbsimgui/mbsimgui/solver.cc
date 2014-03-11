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

namespace MBSimGUI {

  void Environment::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"accelerationOfGravity");
    //setAccelerationOfGravity(Element::getVec3(e));
  }

  DOMElement* Environment::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement* ele0 = D(doc)->createElement( MBSIM%"MBSimEnvironment" );

    DOMElement *ele1 = D(doc)->createElement( MBSIM%"accelerationOfGravity" );
    ele0->insertBefore( ele1, NULL );
    parent->insertBefore(ele0, NULL);
    return ele0;
  }

  Environment::Environment() {}
  Environment::~Environment() {}

  Environment *Environment::instance=NULL;

  Solver::Solver(const string &str, Element *parent) : Group(str,parent), solverParameters(0,false), inverseKinetics(0,false) {

    vector<PhysicalVariableProperty> input;
    vector<string> g(3);
    g[0] = "0";
    g[1] = "-9.81";
    g[2] = "0";
    input.push_back(PhysicalVariableProperty(new VecProperty(g),"m/s^2",MBSIM%"accelerationOfGravity"));
    environment.setProperty(new ExtPhysicalVarProperty(input));
    environment.setProperty(new ChoiceProperty2(new VecPropertyFactory(g,MBSIM%"accelerationOfGravity",vector<string>(3,"m/s^2")),"",4));

    solverParameters.setProperty(new SolverParametersProperty); 

    inverseKinetics.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"inverseKinetics",vector<string>(2,"")),"",4));
  }

  void Solver::initializeUsingXML(DOMElement *element) {
    Group::initializeUsingXML(element);
    DOMElement *e;

    // search first Environment element
    e=E(element)->getFirstElementChildNamed(MBSIM%"environments")->getFirstElementChild();
    Environment *env;
    while((env=ObjectFactory::getInstance()->getEnvironment(e))) {
      env->initializeUsingXML(e);
      environment.initializeUsingXML(e);
      e=e->getNextElementSibling();
    }

    solverParameters.initializeUsingXML(element);

    inverseKinetics.initializeUsingXML(element);
  }

  DOMElement* Solver::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele0 = Group::writeXMLFile(parent);

    DOMElement *ele1 = D(doc)->createElement( MBSIM%"environments" );
    DOMElement *ele2 = D(doc)->createElement( MBSIM%"MBSimEnvironment" );
    environment.writeXMLFile(ele2);
    ele1->insertBefore( ele2, NULL );
    ele0->insertBefore( ele1, NULL );

    solverParameters.writeXMLFile(ele0);

    inverseKinetics.writeXMLFile(ele0);

    return ele0;
  }

  Solver* Solver::readXMLFile(const string &filename, Element *parent) {
    MBSimObjectFactory::initialize();
    shared_ptr<DOMDocument> doc=MainWindow::parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
    Solver *solver=static_cast<Solver*>(ObjectFactory::getInstance()->createGroup(e, parent));
    solver->initializeUsingXML(e);
    solver->initialize();
    return solver;
  }

}
