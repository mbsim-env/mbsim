/* Copyright (C) 2004-2014 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include <cstdlib>
#include "integrator.h"
#include "mbsim/element.h"
#include "mbsim/utils/utils.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  Integrator::Integrator() : tStart(0.), tEnd(1.), dtPlot(1e-4), warnLevel(0), output(true), name("Integrator") {}

  void Integrator::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"startTime");
    setStartTime(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"endTime");
    setEndTime(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"plotStepSize");
    setPlotStepSize(Element::getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"initialState");
    if(e) setInitialState(Element::getVec(e));
  }

  DOMElement* Integrator::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0 = D(doc)->createElement(MBSIMINT%getType());
    parent->insertBefore(ele0, NULL);

    addElementText(ele0,MBSIMINT%"startTime",getStartTime());
    addElementText(ele0,MBSIMINT%"endTime",getEndTime());
    addElementText(ele0,MBSIMINT%"plotStepSize",getPlotStepSize());
    if(getInitialState().size())
      addElementText(ele0,MBSIMINT%"initialState",getInitialState());

    return ele0;
  }

  Integrator* Integrator::readXMLFile(const string &filename) {
    shared_ptr<DOMParser> parser=DOMParser::create(false);
    shared_ptr<DOMDocument> doc=parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
    Integrator *integrator=ObjectFactory::createAndInit<Integrator>(e);
    return integrator;
  }

  void Integrator::writeXMLFile(const string &name) {
//    shared_ptr<DOMDocument> doc=MainWindow::parser->createDocument();
//    writeXMLFile(&doc);
//    map<string, string> nsprefix=XMLNamespaceMapping::getNamespacePrefixMapping();
//    unIncorporateNamespace(doc.FirstChildElement(), nsprefix);  
//    doc.SaveFile((name.length()>13 && name.substr(name.length()-13,13)==".mbsimint.xml")?name:name+".mbsimint.xml");
  }

  // This function is called first by each implementation of Integrator::integrate.
  // We modify here some integrator date for debugging (valgrind) purposes.
  void Integrator::debugInit() {
    // set a minimal end time: integrate only up to the first plot time (+10%) after the plot at tStart
    if(getenv("MBSIM_SET_MINIMAL_TEND")!=NULL)
      setEndTime(getStartTime()+1.1*getPlotStepSize());
  }

//  void Integrator::plot(const Vec& z, double t) {
//    system->resetUpToDate();
//    if (system->getq()() != z()) system->updatezRef(z);
//
////    if (qd() != zdParent()) updatezdRef(zdParent);
//    system->setTime(t);
//    system->updateWRef(system->getWParent(1)(Index(0, system->getuSize(1) - 1), Index(0, system->getlaSize() - 1)), 1);
//    system->updateVRef(system->getVParent(1)(Index(0, system->getuSize(1) - 1), Index(0, system->getlaSize() - 1)), 1);
////    if (system->getlaSize()) {
////      if(useConstraintSolverForPlot) {
////        b << evalW().T() * slvLLFac(evalLLM(), evalh()) + evalwb();
////        solveConstraints();
////      }
////      else
////        system->computeConstraintForces();
//    system->updatezd();
//    system->plot();
//  }
    
}
