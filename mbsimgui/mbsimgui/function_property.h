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

#ifndef _FUNCTION_PROPERTY_H_
#define _FUNCTION_PROPERTY_H_

#include "element.h"

namespace MBSimGUI {

  class Function : public Element {
    public:
      Function(const QString &name="") : Element(name) {}
      virtual ~Function() {}
      virtual int getArg1Size() const {return 0;}
      virtual int getArg2Size() const {return 0;}
      virtual QString getType() const { return "Function"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) { return element; }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent);
      void fromWidget(QWidget *widget) { }
      void toWidget(QWidget *widget) { }
  };

}

#endif

