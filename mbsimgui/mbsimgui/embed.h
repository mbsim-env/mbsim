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

#ifndef _EMBED__H_
#define _EMBED__H_

#include <mbxmlutilshelper/dom.h>
#include "parameter.h"
#include "utils.h"
#include "mainwindow.h"
#include "fileitemdata.h"
#include <QFileInfo>
#include <QDir>
#include <QUrl>
#include <QMessageBox>
#include <xercesc/dom/DOMDocument.hpp>
#include <mbxmlutils/eval.h>

namespace MBSimGUI {

  class Element;
  extern MainWindow *mw;

  template <typename T>
    class Embed {
      public:
        static T* create(xercesc::DOMElement *element);

        static T* create(xercesc::DOMElement *ele1, EmbedItemData* parent) {
          T *object;
          if(MBXMLUtils::E(ele1)->getTagName()==MBXMLUtils::PV%"Embed") {
            xercesc::DOMElement *ele2 = nullptr;
            FileItemData *parameterFileItem = nullptr;

            auto load = [](xercesc::DOMElement *ele1, const std::string &attr) {
              std::string href;
              try{
                href = mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(MBXMLUtils::E(ele1)->getAttribute(attr),ele1,false));
                href = href.substr(1,href.size()-2);
              }
              catch(MBXMLUtils::DOMEvalException &e) {
                mw->setExitBad();
                mw->statusBar()->showMessage(e.getMessage().c_str());
                std::cerr << e.getMessage() << std::endl;
              }
              catch(...) {
                mw->setExitBad();
                mw->statusBar()->showMessage("Unknown exception");
                std::cerr << "Unknwon exception" << std::endl;
              }
              auto docFilename = MBXMLUtils::X()%ele1->getOwnerDocument()->getDocumentURI();
              auto fileInfo = QFileInfo(QDir(QFileInfo(QUrl(docFilename.c_str()).toLocalFile()).canonicalPath()).absoluteFilePath(href.c_str()));
              if(!fileInfo.exists())
                QMessageBox::warning(nullptr, "Import/Reference model file",
                                              ("The imported/referenced model file has a reference to another file which cannot be found:\n"
                                               "\n'"+href+"'\n\n"
                                               "See the errors in the 'MBSim Echo Area'.\n"
                                               "(This may happen e.g. if a model is imported which has relative path reference to other files)").c_str());
              return fileInfo;
            };

            if(MBXMLUtils::E(ele1)->hasAttribute("parameterHref")) {
              mw->updateParameters(parent,false);
              auto fileInfo = load(ele1, "parameterHref");
              if(!fileInfo.exists())
                return nullptr;
              parameterFileItem = mw->addFile(fileInfo);
            }
            else
              ele2 = MBXMLUtils::E(ele1)->getFirstElementChildNamed(MBXMLUtils::PV%"Parameter");
            if(ele2)
              ele2 = ele2->getNextElementSibling();
            else
              ele2 = ele1->getFirstElementChild();
            FileItemData *fileItem = nullptr;
            if(MBXMLUtils::E(ele1)->hasAttribute("href")) {
              if(not parameterFileItem) mw->updateParameters(parent,false);
              auto fileInfo = load(ele1, "href");
              if(!fileInfo.exists())
                return nullptr;
              fileItem = mw->addFile(fileInfo);
              ele2 = fileItem->getXMLElement();
            }
	    try {
	      object=create(ele2);
	    }
	    catch(std::exception &ex) {
	      return nullptr;
	    }
	    object->setXMLElement(ele2);
	    object->setEmbedXMLElement(ele1);
	    object->setParameterFileItem(parameterFileItem);
	    object->createParameters();
	    object->setFileItem(fileItem);
          }
          else {
	    try {
	      object=create(ele1);
	    }
	    catch(std::exception &ex) {
	      return nullptr;
	    }
            object->setXMLElement(ele1);
          }
          return object;
        }

    };

}

#endif
