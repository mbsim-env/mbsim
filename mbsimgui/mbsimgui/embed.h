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
#include <xercesc/dom/DOMProcessingInstruction.hpp>
#include <unordered_map>

namespace MBSimGUI {

  class Element;
  extern QDir mbsDir;
  extern xercesc::DOMLSParser *parser;
  extern std::unordered_map<std::string,xercesc::DOMDocument*> hrefMap;

  template <typename T>
    class Embed {
      public:
        static T* create(xercesc::DOMElement *element);

        static T* createAndInit(xercesc::DOMElement *ele1) {
          T *object;
          std::vector<Parameter*> param;
          if(MBXMLUtils::E(ele1)->getTagName()==MBXMLUtils::PV%"Embed") {
            QString href, parameterHref;
            xercesc::DOMElement *ele2 = 0;
            xercesc::DOMDocument *doc1;
            if(MBXMLUtils::E(ele1)->hasAttribute("parameterHref")) {
              parameterHref = QString::fromStdString(MBXMLUtils::E(ele1)->getAttribute("parameterHref"));
              QFileInfo fileInfo(mbsDir.absoluteFilePath(parameterHref));
              auto it = hrefMap.find(fileInfo.canonicalFilePath().toStdString());
              if(it == hrefMap.end()) {
                doc1 = parser->parseURI(MBXMLUtils::X()%fileInfo.canonicalFilePath().toStdString());
                hrefMap[fileInfo.canonicalFilePath().toStdString()] = doc1;
              }
              else
                doc1 = it->second;
              //ele2 = static_cast<xercesc::DOMElement*>(ele1->getOwnerDocument()->importNode(doc1->getDocumentElement(),true));
              //ele1->insertBefore(ele2,ele1->getFirstElementChild());
              //MBXMLUtils::E(ele1)->removeAttribute("parameterHref");
              param = Parameter::initializeParametersUsingXML(doc1->getDocumentElement());
              ele2 = ele1->getFirstElementChild();
              xercesc::DOMProcessingInstruction *id=ele1->getOwnerDocument()->createProcessingInstruction(MBXMLUtils::X()%"parameterHref", MBXMLUtils::X()%parameterHref.toStdString());
              ele1->insertBefore(id, ele1->getFirstChild());
            }
            else {
              ele2 = MBXMLUtils::E(ele1)->getFirstElementChildNamed(MBXMLUtils::PV%"Parameter");
              param = Parameter::initializeParametersUsingXML(ele2);
              ele2 = ele2->getNextElementSibling();
            }
            if(MBXMLUtils::E(ele1)->hasAttribute("href")) {
              href = QString::fromStdString(MBXMLUtils::E(ele1)->getAttribute("href"));
              QFileInfo fileInfo(mbsDir.absoluteFilePath(href));
              xercesc::DOMDocument *doc = parser->parseURI(MBXMLUtils::X()%fileInfo.canonicalFilePath().toStdString());
              ele2 = static_cast<xercesc::DOMElement*>(ele1->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
              ele1->insertBefore(ele2,NULL);
              MBXMLUtils::E(ele1)->removeAttribute("href");
              xercesc::DOMProcessingInstruction *id=ele1->getOwnerDocument()->createProcessingInstruction(MBXMLUtils::X()%"href", MBXMLUtils::X()%href.toStdString());
              ele1->insertBefore(id, ele1->getFirstChild());
            }
            object=create(ele2);
            if(object) {
              object->initializeUsingXML(ele2);
              for(size_t i=0; i<param.size(); i++)
                object->addParameter(param[i]);
              if((not parameterHref.isEmpty()) or (not href.isEmpty())) {
                object->setParamHref(doc1);
                xercesc::DOMDocument *doc=ele2->getOwnerDocument();
                xercesc::DOMProcessingInstruction *instr = MBXMLUtils::E(doc->getDocumentElement())->getFirstProcessingInstructionChildNamed("hrefCount");
                if(not instr) {
                  instr=doc->createProcessingInstruction(MBXMLUtils::X()%"hrefCount", MBXMLUtils::X()%"1");
                  doc->getDocumentElement()->insertBefore(instr, doc->getDocumentElement()->getFirstChild());
                }
                else {
                  std::string count = MBXMLUtils::X()%instr->getData();
                  instr->setData(MBXMLUtils::X()%toStr(boost::lexical_cast<int>(count)+1));
                }
              }
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
