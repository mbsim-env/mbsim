/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012-2016 Martin Förg

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

#ifndef _CONNECTION__H_
#define _CONNECTION__H_

#include "link.h"
#include "extended_properties.h"

namespace MBSimGUI {

  class GeneralizedElasticConnection : public DualRigidBodyLink {
    friend class GeneralizedElasticConnectionPropertyDialog;
    public:
    GeneralizedElasticConnection(const std::string &str="");
    std::string getType() const { return "GeneralizedElasticConnection"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new GeneralizedElasticConnectionPropertyDialog(this);}
    protected:
    ExtProperty function;
  };

}

#endif
