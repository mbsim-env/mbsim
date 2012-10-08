/* Copyright (C) 2004-2009 MBSim Development Team
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
#include "integrator.h"
#include "mbsim/element.h"
#include "mbsim/utils/utils.h"
#include "mbsim/objectfactory.h"
#include "mbsimtinyxml/tinyxml-src/tinynamespace.h"

using namespace std;

namespace MBSim {

  DynamicSystemSolver * Integrator::system = 0;

  Integrator::Integrator() : tStart(0.), tEnd(1.), dtPlot(1e-4), warnLevel(0), output(true), name("Integrator") {}

  void Integrator::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMINTNS"startTime");
    setStartTime(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"endTime");
    setEndTime(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"plotStepSize");
    setPlotStepSize(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMINTNS"initialState");
    if(e) setInitialState(Element::getVec(e));
  }

  TiXmlElement* Integrator::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0=new TiXmlElement(MBSIMINTNS+getType());
    parent->LinkEndChild(ele0);

    addElementText(ele0,MBSIMINTNS"startTime",getStartTime());
    addElementText(ele0,MBSIMINTNS"endTime",getEndTime());
    addElementText(ele0,MBSIMINTNS"plotStepSize",getPlotStepSize());
    if(getInitialState().size())
      addElementText(ele0,MBSIMINTNS"initialState",mat2str(getInitialState()));

    return ele0;
  }

  Integrator* Integrator::readXMLFile(const string &filename) {
    MBSimObjectFactory::initialize();
    TiXmlDocument doc;
    assert(doc.LoadFile(filename)==true);
    TiXml_PostLoadFile(&doc);
    TiXmlElement *e=doc.FirstChildElement();
    TiXml_setLineNrFromProcessingInstruction(e);
    map<string,string> dummy;
    incorporateNamespace(e, dummy);
    Integrator *integrator=ObjectFactory::getInstance()->createIntegrator(e);
    integrator->initializeUsingXML(doc.FirstChildElement());
    return integrator;
  }

  void Integrator::writeXMLFile(const string &name) {
    MBSimObjectFactory::initialize();
    TiXmlDocument doc;
    TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
    doc.LinkEndChild( decl );
    writeXMLFile(&doc);
    map<string, string> nsprefix=ObjectFactory::getInstance()->getNamespacePrefixMapping();
    unIncorporateNamespace(doc.FirstChildElement(), nsprefix);  
    doc.SaveFile(name+".mbsimint.xml");
  }

}

