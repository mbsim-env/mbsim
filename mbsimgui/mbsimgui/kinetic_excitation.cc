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
#include "kinetic_excitation.h"
#include <xercesc/dom/DOMDocument.hpp>
#include "objectfactory.h"
#include "diagram_item.h"
#include "diagram_arrow.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  MBSIMGUI_REGOBJECTFACTORY(KineticExcitation);

  DOMElement* KineticExcitation::processIDAndHref(DOMElement *element) {
    element = Link::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

    return element;
  }

  void KineticExcitation::createDiagramArrows() {
    FloatingFrameLink::createDiagramArrows();
    auto *e=E(getXMLElement())->getFirstElementChildNamed(MBSIM%"momentFunction");
    if(e) {
      auto *ele=E(e)->getFirstElementChildNamed(MBSIMCONTROL%"SignalFunction");
      if(ele) {
        auto *ret = E(ele)->getFirstElementChildNamed(MBSIMCONTROL%"returnSignal");
        if(ret) {
          auto str = QString::fromStdString(E(ret)->getAttribute("ref"));
          auto *ref = getByPath<Element>(str.remove(0,3));
          auto *startItem = ref->getDiagramItem();
          if(startItem) {
            DiagramArrow *arrow = new DiagramArrow(startItem, diagramItem, diagramItem);
            arrow->updatePosition();
          }
        }
      }
    }
  }

}
