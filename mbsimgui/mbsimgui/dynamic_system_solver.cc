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
#include "dynamic_system_solver.h"
#include "project.h"
#include "objectfactory.h"
#include "utils.h"
#include "mainwindow.h"
#include "parameter.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <mbxmlutils/eval.h>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  DynamicSystemSolver::DynamicSystemSolver() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"dss.svg").string()));
    parameterEmbedItem->setIcon(icon);
  }

  void DynamicSystemSolver::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    while(e) {
      DOMNode *en=e->getNextSibling();
      if((e != frames) and (e != contours) and (e != groups) and (e != objects) and (e != links) and (e != constraints) and (e != observers) and (E(e)->getTagName() != MBSIM%"enableOpenMBVFrameI") and (E(e)->getTagName() != MBSIM%"plotFeatureFrameI"))
        element->removeChild(e);
      e = en;
    }
  }

  DOMElement* DynamicSystemSolver::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Group::createXMLElement(parent);
    DOMDocument *doc=ele0->getOwnerDocument();

    E(ele0)->setAttribute("name", "MBS");
    name = "MBS";
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
      ele5->insertBefore(text, nullptr);
      ele4->insertBefore(ele5, nullptr);
    }
    ele3->insertBefore( ele4, nullptr );
    ele2->insertBefore( ele3, nullptr );
    environments->insertBefore( ele2, nullptr );
    ele0->insertBefore( environments, nullptr );
    return ele0;
  }

  void DynamicSystemSolver::create() {
    Group::create();
    environments = E(element)->getFirstElementChildNamed(MBSIM%"environments");
  }

  EmbedItemData* DynamicSystemSolver::getEmbedItemParent() {
    return project;
  }

}
