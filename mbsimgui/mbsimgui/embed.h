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
#include "mainwindow.h"
#include <QFileInfo>
#include <QDir>
#include <QUrl>
#include <xercesc/dom/DOMDocument.hpp>
#include <mbxmlutils/eval.h>

namespace MBSimGUI {

  class Element;
  extern xercesc::DOMLSParser *parser;
  extern MainWindow *mw;

  template <typename T>
    class Embed {
      public:
        static T* create(xercesc::DOMElement *element);

        static T* createAndInit(xercesc::DOMElement *ele1, EmbedItemData* item=nullptr) {
          T *object;
          std::vector<Parameter*> param;
          if(MBXMLUtils::E(ele1)->getTagName()==MBXMLUtils::PV%"Embed") {
            bool embededParam = false;
            xercesc::DOMElement *ele2 = nullptr;

            if(MBXMLUtils::E(ele1)->hasAttribute("parameterHref")) {
              mw->updateParameters(item,false);
              std::string evaltmp;
              try{
                evaltmp = mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(MBXMLUtils::E(ele1)->getAttribute("parameterHref"),ele1,false));
              }
              catch(MBXMLUtils::DOMEvalException e) {
                std::cout << e.getMessage() << std::endl;
              }
              catch(...) {
                std::cout << "Unknwon error" << std::endl;
              }
              xercesc::DOMDocument *doc = parser->parseURI(MBXMLUtils::X()%QDir(QFileInfo(QUrl(QString::fromStdString(MBXMLUtils::X()%ele1->getOwnerDocument()->getDocumentURI())).toLocalFile()).canonicalPath()).absoluteFilePath(QString::fromStdString(evaltmp.substr(1,evaltmp.size()-2))).toStdString());
              MBXMLUtils::DOMParser::handleCDATA(doc->getDocumentElement());
              param = Parameter::initializeParametersUsingXML(doc->getDocumentElement());
              embededParam = true;
            }
            else
              ele2 = MBXMLUtils::E(ele1)->getFirstElementChildNamed(MBXMLUtils::PV%"Parameter");
            if(ele2) {
              param = Parameter::initializeParametersUsingXML(ele2);
              ele2 = ele2->getNextElementSibling();
            }
            else
              ele2 = ele1->getFirstElementChild();
            bool embeded = false;
            if(MBXMLUtils::E(ele1)->hasAttribute("href")) {
              if(not embededParam) mw->updateParameters(item,false);
              std::string evaltmp;
              try{
                evaltmp = mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(MBXMLUtils::E(ele1)->getAttribute("href"),ele1,false));
              }
              catch(MBXMLUtils::DOMEvalException e) {
                std::cout << e.getMessage() << std::endl;
              }
              catch(...) {
                std::cout << "Unknwon error" << std::endl;
              }
              xercesc::DOMDocument *doc = parser->parseURI(MBXMLUtils::X()%QDir(QFileInfo(QUrl(QString::fromStdString(MBXMLUtils::X()%ele1->getOwnerDocument()->getDocumentURI())).toLocalFile()).canonicalPath()).absoluteFilePath(QString::fromStdString(evaltmp.substr(1,evaltmp.size()-2))).toStdString());
              MBXMLUtils::DOMParser::handleCDATA(doc->getDocumentElement());
              ele2 = doc->getDocumentElement();
              embeded = true;
            }
            object=create(ele2);
            if(object) {
              object->setEmbedXMLElement(ele1);
              if(embededParam) object->setEmbededParameters(embededParam);
              if(embeded) object->setEmbeded(embeded);
              for(auto & i : param)
                object->addParameter(i);
              object->initializeUsingXML(ele2);
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
