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
#include "frame.h"
#include "utils.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMProcessingInstruction.hpp>
#include "mainwindow.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  MBSIMGUI_REGOBJECTFACTORY(FixedRelativeFrame);
  MBSIMGUI_REGOBJECTFACTORY(UnknownFixedRelativeFrame);
  MBSIMGUI_REGOBJECTFACTORY(NodeFrame);
  MBSIMGUI_REGOBJECTFACTORY(UnknownNodeFrame);
  MBSIMGUI_REGOBJECTFACTORY(InterfaceNodeFrame);
  MBSIMGUI_REGOBJECTFACTORY(FfrInterfaceNodeFrame);

  Frame::Frame() {
    icon = Utils::QIconCached(QString::fromStdString((mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"frame.svg").string()));
  }

  DOMElement* Frame::processIDAndHref(DOMElement *element) {
    element = Element::processIDAndHref(element);
    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }
    return element;
  }

  InternalFrame::InternalFrame(const QString &name_, MBXMLUtils::FQN xmlFrameName_, const MBXMLUtils::FQN &plotFeatureType_) : name(name_), xmlFrameName(std::move(xmlFrameName_)), plotFeatureType(plotFeatureType_) {
  }

  void InternalFrame::removeXMLElements() {
    DOMElement *e = E(parent->getXMLElement())->getFirstElementChildNamed(getXMLFrameName());
    if(e) {
      if(X()%e->getPreviousSibling()->getNodeName()=="#text")
        parent->getXMLElement()->removeChild(e->getPreviousSibling());
      parent->getXMLElement()->removeChild(e);
    }
    e = E(parent->getXMLElement())->getFirstElementChildNamed(getPlotFeatureType());
    while (e and E(e)->getTagName()==getPlotFeatureType()) {
      DOMElement *en = e->getNextElementSibling();
      if(X()%e->getPreviousSibling()->getNodeName()=="#text")
        parent->getXMLElement()->removeChild(e->getPreviousSibling());
      parent->getXMLElement()->removeChild(e);
      e = en;
    }
  }

  UnknownFixedRelativeFrame::UnknownFixedRelativeFrame() {
    icon = QIcon(new OverlayIconEngine((mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"frame.svg").string(),
                                       (mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"unknownelement.svg").string()));
  }

  UnknownNodeFrame::UnknownNodeFrame() {
    icon = QIcon(new OverlayIconEngine((mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"frame.svg").string(),
                                       (mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"unknownelement.svg").string()));
  }

}
