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

#ifndef _FUNCTION_WIDGET_H_
#define _FUNCTION_WIDGET_H_

#include "widget.h"
#include "namespace.h"

namespace MBSimGUI {

  class FunctionWidget : public Widget {
    public:
      enum VarType {
        scalar=0,
        fixedVec,
        varVec
      };
      virtual int getArg1Size() const { return 1; }
      virtual int getArg2Size() const { return 1; }
      virtual void setArg1Size(int i) { }
      virtual void setArg2Size(int i) { }
      virtual MBXMLUtils::FQN getXMLType() const { return MBSIM%"Function"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override { return element; }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

}

#endif
