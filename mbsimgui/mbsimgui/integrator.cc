/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#include <config.h>
#include "integrator.h"
#include <xercesc/dom/DOMDocument.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  DOMElement* Integrator::createXMLElement(DOMNode *parent) {
    DOMElement *element = Solver::createXMLElement(parent);
    DOMDocument *doc=element->getOwnerDocument();
    DOMElement *ele1 = D(doc)->createElement( MBSIM%"startTime" );
    E(ele1)->setAttribute("unit", "s");
    DOMText *text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, nullptr);
    element->insertBefore( ele1, nullptr );
    ele1 = D(doc)->createElement( MBSIM%"endTime" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"1");
    ele1->insertBefore(text, nullptr);
    element->insertBefore( ele1, nullptr );
    ele1 = D(doc)->createElement( MBSIM%"plotStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"1e-2");
    ele1->insertBefore(text, nullptr);
    element->insertBefore( ele1, nullptr );
    return element;
  }

}
