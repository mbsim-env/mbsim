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
#include "dynamic_system_solver.h"
#include "objectfactory.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

//extern shared_ptr<DOMLSParser> parser;
extern DOMLSParser *parser;

namespace MBSimGUI {

  Environment *Environment::instance=NULL;

  DynamicSystemSolver::DynamicSystemSolver(const string &str) : Group(str) {
    config = true;

//    solverParameters.setProperty(new DynamicSystemSolverParametersProperty);
//
//    inverseKinetics.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"inverseKinetics",vector<string>(2,"")),"",4));
//
//    initialProjection.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"initialProjection",vector<string>(2,"")),"",4));
//
//    useConstraintSolverForPlot.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"useConstraintSolverForPlot",vector<string>(2,"")),"",4));
  }

  void DynamicSystemSolver::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    DOMElement *ombvFrame=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameI");
    while(e) {
      DOMNode *en=e->getNextSibling();
      if(e == environments) {
        DOMElement *env = E(static_cast<DOMElement*>(e))->getFirstElementChildNamed(MBSIM%"MBSimEnvironment");
        DOMElement *grav = E(env)->getFirstElementChildNamed(MBSIM%"accelerationOfGravity");
        DOMNode *ee = env->getFirstChild();
        while(ee) {
          DOMNode *een=ee->getNextSibling();
          if(ee == grav)
            env->removeChild(ee);
          ee = een;
        }
      }
      else if((e != frames) and (e != contours) and (e != groups) and (e != objects) and (e != links) and (e != constraints) and (e != observers) and (e != ombvFrame))
        element->removeChild(e);
      e = en;
    }
  }

  DOMElement* DynamicSystemSolver::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Group::createXMLElement(parent);
    DOMDocument *doc=ele0->getOwnerDocument();

    E(ele0)->setAttribute("name", getName());
    environments = D(doc)->createElement( MBSIM%"environments" );
    DOMElement *ele2 = D(doc)->createElement( MBSIM%"MBSimEnvironment" );
    DOMElement *ele3 = D(doc)->createElement( MBSIM%"accelerationOfGravity" );
    E(ele3)->setAttribute("unit", "m/s^2");
    DOMElement *ele4 = D(doc)->createElement( PV%"xmlVector" );
    vector<string> g(3);
    g[0] = "0";
    g[1] = "-9.81";
    g[2] = "0";
    for(int i=0; i<3; i++) {
      DOMElement *ele5 = D(doc)->createElement( PV%"ele" );
      DOMText *text = doc->createTextNode(X()%g[i]);
      ele5->insertBefore(text, NULL);
      ele4->insertBefore(ele5, NULL);
    }
    ele3->insertBefore( ele4, NULL );
    ele2->insertBefore( ele3, NULL );
    environments->insertBefore( ele2, NULL );
    ele0->insertBefore( environments, NULL );
    return ele0;
  }

  DOMElement* DynamicSystemSolver::initializeUsingXML(DOMElement *element) {
    Group::initializeUsingXML(element);
    environments = E(element)->getFirstElementChildNamed(MBSIM%"environments");
    return element;
  }

  DynamicSystemSolver* DynamicSystemSolver::readXMLFile(const string &filename) {
    MBSimObjectFactory::initialize();
    shared_ptr<DOMDocument> doc(parser->parseURI(X()%filename));
    DOMElement *e=doc->getDocumentElement();
    DynamicSystemSolver *solver=static_cast<DynamicSystemSolver*>(ObjectFactory::getInstance()->createGroup(e));
    solver->initializeUsingXML(e);
    return solver;
  }

}
