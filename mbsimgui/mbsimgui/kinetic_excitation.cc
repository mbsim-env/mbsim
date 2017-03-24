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
#include "kinetic_excitation.h"
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  KineticExcitation::KineticExcitation(const QString &str) : FloatingFrameLink(str) {

//    static_cast<ConnectFramesProperty*>(connections.getProperty())->setDefaultFrame("../Frame[I]");
//
//    forceDirection.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(3,1,"0"),MBSIM%"forceDirection",vector<string>(3,"-")),"",4));
//
//    forceFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"forceFunction",0));
//
//    momentDirection.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(3,1,"0"),MBSIM%"momentDirection",vector<string>(3,"-")),"",4));
//
//    momentFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"momentFunction",0));
//
//    arrow.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
//    arrow.setActive(true);
  }

  DOMElement* KineticExcitation::processFileID(DOMElement *element) {
    Link::processFileID(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    return element;
  }

}
