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
#include "integrator.h"
#include <xercesc/dom/DOMDocument.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  DOMElement* Integrator::createXMLElement(DOMNode *parent) {
    DOMElement *element = Solver::createXMLElement(parent);
    DOMDocument *doc=element->getOwnerDocument();
    DOMElement *ele1 = D(doc)->createElement( MBSIMINT%"startTime" );
    E(ele1)->setAttribute("unit", "s");
    DOMText *text = doc->createTextNode(X()%"0");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"endTime" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"1");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    ele1 = D(doc)->createElement( MBSIMINT%"plotStepSize" );
    E(ele1)->setAttribute("unit", "s");
    text = doc->createTextNode(X()%"1e-2");
    ele1->insertBefore(text, NULL);
    element->insertBefore( ele1, NULL );
    return element;
  }

}
