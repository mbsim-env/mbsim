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
#include "parameter.h"
#include "utils.h"
#include <QFileInfo>
#include <QDir>
#include <boost/lexical_cast.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <unordered_map>

namespace MBSimGUI {

  class Element;
  extern QDir mbsDir;
  extern xercesc::DOMLSParser *parser;
  extern std::unordered_map<std::string,std::pair<xercesc::DOMDocument*,int> > hrefMap;

  template <typename T>
    class Embed {
      public:
        static T* create(xercesc::DOMElement *element);

        static T* createAndInit(xercesc::DOMElement *ele1) {
          T *object;
          std::vector<Parameter*> param;
          if(MBXMLUtils::E(ele1)->getTagName()==MBXMLUtils::PV%"Embed") {
            xercesc::DOMElement *ele2 = 0;
            xercesc::DOMDocument *doc;
            if(MBXMLUtils::E(ele1)->hasAttribute("parameterHref")) {
              std::cout << "parameterHref is currently not supported by MBSimGUI" << std::endl;
              throw;
              QFileInfo fileInfo(mbsDir.absoluteFilePath(QString::fromStdString(MBXMLUtils::E(ele1)->getAttribute("parameterHref"))));
              auto it = hrefMap.find(fileInfo.canonicalFilePath().toStdString());
              if(it == hrefMap.end()) {
                doc = parser->parseURI(MBXMLUtils::X()%fileInfo.canonicalFilePath().toStdString());
                hrefMap[fileInfo.canonicalFilePath().toStdString()] = std::pair<xercesc::DOMDocument*,int>(doc,1);
              }
              else {
                doc = it->second.first;
                it->second.second++;
              }
              param = Parameter::initializeParametersUsingXML(doc->getDocumentElement());
              ele2 = ele1->getFirstElementChild();
            }
            else {
              ele2 = MBXMLUtils::E(ele1)->getFirstElementChildNamed(MBXMLUtils::PV%"Parameter");
              param = Parameter::initializeParametersUsingXML(ele2);
              ele2 = ele2->getNextElementSibling();
            }
            if(MBXMLUtils::E(ele1)->hasAttribute("href")) {
              std::cout << "href is currently not supported by MBSimGUI" << std::endl;
              throw;
            }
            object=create(ele2);
            if(object) {
              object->initializeUsingXML(ele2);
              for(size_t i=0; i<param.size(); i++)
                object->addParameter(param[i]);
            }
          }
          else {
            object=create(ele1);
            if(object) object->initializeUsingXML(ele1);
          }
          return object;
        }

    };

}

#endif
