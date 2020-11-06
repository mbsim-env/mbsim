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
    xercesc::DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<xercesc::DOMDocument*>(parent) : parent->getOwnerDocument();
    element=D(doc)->createElement(getXMLType());
    parent->insertBefore(element, nullptr);
    return element;
  }

  EmbedItemData* Solver::getEmbedItemParent() {
    return project;
  }

}
