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

#include <config.h>
#include "dynamic_system_solver.h"
#include "project.h"
#include "objectfactory.h"
#include "utils.h"
#include "mainwindow.h"
#include "parameter.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <mbxmlutils/eval.h>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  DynamicSystemSolver::DynamicSystemSolver() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"dss.svg").string()));
    parameters->setIcon(icon);
  }

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
    DOMDocument *doc=ele0->getOwnerDocument();

    E(ele0)->setAttribute("name", "MBS");
    name = "MBS";
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
    DOMElement *ele1=E(E(element)->getFirstElementChildNamed(MBSIM%"environments"))->getFirstElementChildNamed(MBSIM%"MBSimEnvironment");
    if(not ele1) return element;
    ele1=E(ele1)->getFirstElementChildNamed(MBSIM%"openMBVObject");
    if(not ele1) return element;
    ele1 = ele1->getFirstElementChild();
    while(ele1) {
      if(embed && MBXMLUtils::E(ele1)->hasAttribute("href")) {
        mw->updateParameters(this);
        std::string evaltmp;
        try{
          evaltmp = mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(MBXMLUtils::E(ele1)->getAttribute("href"),ele1,false));
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
        auto docFilename = MBXMLUtils::D(element->getOwnerDocument())->getDocumentFilename();
        boost::filesystem::canonical(docFilename);
        auto doc = mw->mbxmlparserNoVal->parse(QDir(docFilename.parent_path().string().c_str()).absoluteFilePath(QString::fromStdString(evaltmp.substr(1,evaltmp.size()-2))).toStdString());
        if(!doc) {
          ele1 = ele1->getNextElementSibling();
          continue;
        }
        DOMElement *ele2 = static_cast<xercesc::DOMElement*>(element->getOwnerDocument()->importNode(doc->getDocumentElement(),true));
        ele1->insertBefore(ele2,nullptr);
        boost::filesystem::path orgFileName=E(doc->getDocumentElement())->getOriginalFilename();
        E(ele2)->addEmbedData("MBXMLUtils_OriginalFilename", orgFileName.string());
        E(ele2)->setOriginalElementLineNumber(E(ele1)->getLineNumber());
        MBXMLUtils::E(ele1)->removeAttribute("href");
      }
      if(E(ele1)->getTagName()==PV%"Embed")
        E(ele1->getFirstElementChild())->addProcessingInstructionChildNamed("OPENMBV_ID", getID());
      else
        E(ele1)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());

      ele1 = ele1->getNextElementSibling();
    }
    return element;
  }

  EmbedItemData* DynamicSystemSolver::getEmbedItemParent() {
    return project;
  }

}
