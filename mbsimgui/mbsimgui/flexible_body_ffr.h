/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2016 Martin Förg

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

#ifndef _FLEXIBLE_BODY_FFR__H_
#define _FLEXIBLE_BODY_FFR__H_

#include "body.h"
#include "extended_properties.h"

namespace MBSimGUI {

  class FlexibleBodyFFR : public Body {
    friend class FlexibleBodyFFRPropertyDialog;
    public:
      FlexibleBodyFFR(const std::string &str="");
//      virtual PropertyInterface* clone() const {return new FlexibleBodyFFR(*this);}
      std::string getType() const { return "FlexibleBodyFFR"; }
      MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIMFLEX; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      ElementPropertyDialog* createPropertyDialog() {return new FlexibleBodyFFRPropertyDialog(this);}
      QMenu* createContextMenu() {return new FlexibleBodyFFRContextMenu(this);}
      QMenu* createFrameContextMenu() {return new NodeFrameContextContextMenu(this);}
    protected:
      ExtProperty mass, pdm, ppdm, Pdm, rPdm, PPdm, Ke, De, beta, Knl1, Knl2, ksigma0, ksigma1, K0t, K0r, K0om, r, A, Phi, Psi, sigmahel, sigmahen, sigma0, K0F, K0M, translation, rotation, translationDependentRotation, coordinateTransformationForRotation, ombvEditor;
  };

}

#endif
