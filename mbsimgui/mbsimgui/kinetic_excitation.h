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

#ifndef _KINETIC_EXCITATION__H_
#define _KINETIC_EXCITATION__H_

#include "link.h"
#include "extended_properties.h"

namespace MBSimGUI {

  class KineticExcitation : public Link {
    friend class KineticExcitationPropertyDialog;
    public:
    KineticExcitation(const std::string &str, Element *parent);
    virtual PropertyInterface* clone() const {return new KineticExcitation(*this);}
    std::string getType() const { return "KineticExcitation"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void initialize();
    ElementPropertyDialog* createPropertyDialog() {return new KineticExcitationPropertyDialog(this);}
    protected:
    ExtProperty refFrameID, forceDirection, forceFunction, momentDirection, momentFunction, connections, forceArrow, momentArrow;
  };

}

#endif
