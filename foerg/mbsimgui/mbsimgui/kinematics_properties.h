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

#ifndef _KINEMATICS_PROPERTIES_H_
#define _KINEMATICS_PROPERTIES_H_

#include "property.h"
#include "kinematics_widgets.h"
#include "function_widget.h"
#include <string>

class Translation : public Property {

  public:
    Translation(const std::string &name="");
    virtual Property* clone() const {return new Translation(*this);}
    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    QMenu* createContextMenu() {return new TranslationChoiceContextMenu(this);}
    int getqSize() const;
    int getuSize() const;
};

#endif

