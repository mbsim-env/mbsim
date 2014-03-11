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

#ifndef _EMBED__H_
#define _EMBED__H_

#include <mbxmlutilshelper/dom.h>
#include <parameter.h>

namespace MBSimGUI {

  class Element;

  template <typename T>
    class Embed {
      public:
        static T* create(xercesc::DOMElement *element, Element *parent);

        static T* createAndInit(xercesc::DOMElement *ele1, Element* parent) {
          T *object;
          if(MBXMLUtils::E(ele1)->getTagName()==MBXMLUtils::PV%"Embed") {
            xercesc::DOMElement *ele2 = 0;
            Parameters param;
            if(MBXMLUtils::E(ele1)->hasAttribute("parameterHref")) {
              param = Parameters::readXMLFile(MBXMLUtils::E(ele1)->getAttribute("parameterHref"));
              ele2=ele1->getFirstElementChild();
            }
            else {
              ele2=MBXMLUtils::E(ele1)->getFirstElementChildNamed(MBXMLUtils::PV%"Parameter");
              if(ele2) {
                param.initializeUsingXML(ele2);
                ele2=ele2->getNextElementSibling();
              }
              else 
                ele2=ele1->getFirstElementChild();
            }
            if(MBXMLUtils::E(ele1)->hasAttribute("href"))
              object=T::readXMLFile(MBXMLUtils::E(ele1)->getAttribute("href"),parent);
            else
              object=create(ele2,parent);
            if(object) {
              object->initializeUsingXMLEmbed(ele1);
              if(ele2)
                object->initializeUsingXML(ele2);
              object->setParameters(param);
            }
          }
          else {
            object=create(ele1,parent);
            if(object) object->initializeUsingXML(ele1);
          }
          return object;
        }

        static xercesc::DOMElement* writeXML(T* object, xercesc::DOMNode *ele0) {
          if(object->isEmbedded())
            object->writeXMLFileEmbed(ele0);
          else
            object->writeXMLFile(ele0);
        }

    };

}

#endif
