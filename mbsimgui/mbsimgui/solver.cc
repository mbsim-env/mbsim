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
#include "solver.h"
#include "project.h"
#include "utils.h"
#include "mainwindow.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMLSSerializer.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  MBSIMGUI_REGOBJECTFACTORY(UnknownSolver);

  Solver::Solver() {
    icon = Utils::QIconCached(QString::fromStdString((mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"solver.svg").string()));
  }

  void Solver::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    while(e) {
      DOMNode *en=e->getNextSibling();
      element->removeChild(e);
      e = en;
    }
  }

  DOMElement* Solver::createXMLElement(DOMNode *parent) {
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    element=D(doc)->createElement(getXMLType());
    parent->insertBefore(element, nullptr);
    return element;
  }

  EmbedItemData* Solver::getEmbedItemParent() {
    return project;
  }

}
