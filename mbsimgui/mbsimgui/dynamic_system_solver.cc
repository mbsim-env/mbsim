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

#include <config.h>
#include "dynamic_system_solver.h"
#include "project.h"
#include "objectfactory.h"
#include "mainwindow.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMProcessingInstruction.hpp>
#include <mbxmlutils/eval.h>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern bool currentTask;
  extern MainWindow *mw;

  Environment *Environment::instance=nullptr;

  void DynamicSystemSolver::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    while(e) {
      DOMNode *en=e->getNextSibling();
      if((e != frames) and (e != contours) and (e != groups) and (e != objects) and (e != links) and (e != constraints) and (e != observers) and (E(e)->getTagName() != MBSIM%"enableOpenMBVFrameI") and (E(e)->getTagName() != MBSIM%"plotFeatureFrameI"))
        element->removeChild(e);
      e = en;
    }
  }

  DOMElement* DynamicSystemSolver::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Group::createXMLElement(parent);
    xercesc::DOMDocument *doc=ele0->getOwnerDocument();

    E(ele0)->setAttribute("name", "MBS");
    environments = D(doc)->createElement( MBSIM%"environments" );
    DOMElement *ele2 = D(doc)->createElement( MBSIM%"MBSimEnvironment" );
    DOMElement *ele3 = D(doc)->createElement( MBSIM%"accelerationOfGravity" );
    E(ele3)->setAttribute("unit", "m/s^2");
    DOMElement *ele4 = D(doc)->createElement( PV%"xmlVector" );
    vector<string> g(3);
    g[0] = "0";
    g[1] = "-9.81";
    g[2] = "0";
    for(int i=0; i<3; i++) {
      DOMElement *ele5 = D(doc)->createElement( PV%"ele" );
      DOMText *text = doc->createTextNode(X()%g[i]);
      ele5->insertBefore(text, nullptr);
      ele4->insertBefore(ele5, nullptr);
    }
    ele3->insertBefore( ele4, nullptr );
    ele2->insertBefore( ele3, nullptr );
    environments->insertBefore( ele2, nullptr );
    ele0->insertBefore( environments, nullptr );
    return ele0;
  }

  void DynamicSystemSolver::create() {
    Group::create();
    environments = E(element)->getFirstElementChildNamed(MBSIM%"environments");
  }

  DOMElement* DynamicSystemSolver::processIDAndHref(DOMElement *element) {
    element = Group::processIDAndHref(element);
    if(currentTask==1) {
      E(element)->setAttribute("name","MBS_tmp");
      DOMElement *ele1 = D(element->getOwnerDocument())->createElement( MBSIM%"plotFeatureRecursive" );
      E(ele1)->setAttribute("value","plotRecursive");
      ele1->insertBefore(element->getOwnerDocument()->createTextNode(X()%project->getVarFalse().toStdString()), nullptr);
      element->insertBefore( ele1, element->getFirstElementChild() );
    }
    DOMElement *ele1=E(E(element)->getFirstElementChildNamed(MBSIM%"environments"))->getFirstElementChildNamed(MBSIM%"MBSimEnvironment");
    if(not ele1) return element;
    ele1=E(ele1)->getFirstElementChildNamed(MBSIM%"openMBVObject");
    if(not ele1) return element;
    ele1 = ele1->getFirstElementChild();
    if(not ele1) return element;
    if(MBXMLUtils::E(ele1)->hasAttribute("href")) {
      mw->updateParameters(this,false);
      std::string evaltmp;
      try{
        evaltmp = mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(MBXMLUtils::E(ele1)->getAttribute("href"),ele1,false));
      }
      catch(MBXMLUtils::DOMEvalException &e) {
        std::cout << e.getMessage() << std::endl;
      }
      catch(...) {
        std::cout << "Unknwon error" << std::endl;
      }
      xercesc::DOMDocument *doc = mw->parser->parseURI(MBXMLUtils::X()%QDir(QFileInfo(QUrl(QString::fromStdString(MBXMLUtils::X()%element->getOwnerDocument()->getDocumentURI())).toLocalFile()).canonicalPath()).absoluteFilePath(QString::fromStdString(evaltmp.substr(1,evaltmp.size()-2))).toStdString());
      MBXMLUtils::DOMParser::handleCDATA(doc->getDocumentElement());
      DOMElement *ele2 = static_cast<xercesc::DOMElement*>(element->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
      ele1->insertBefore(ele2,NULL);
      boost::filesystem::path orgFileName=E(doc->getDocumentElement())->getOriginalFilename();
      DOMProcessingInstruction *filenamePI=ele2->getOwnerDocument()->createProcessingInstruction(X()%"OriginalFilename",
          X()%orgFileName.string());
      ele2->insertBefore(filenamePI, ele2->getFirstChild());
      MBXMLUtils::E(ele1)->removeAttribute("href");
    }
    xercesc::DOMDocument *doc=element->getOwnerDocument();
    DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
    if(E(ele1)->getTagName()==PV%"Embed")
      ele1 = ele1->getFirstElementChild();
    ele1->insertBefore(id, nullptr);
    return element;
  }

  EmbedItemData* DynamicSystemSolver::getEmbedItemParent() {
    return project;
  }

}
