/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2017 Martin Förg

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
#include "project.h"
#include "dynamic_system_solver.h"
#include "solver.h"
#include "integrator.h"
#include "embed.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMLSSerializer.hpp>
#include <xercesc/dom/DOMComment.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Project::Project() {
    trueMap["xmlflat"] = "true";
    trueMap["octave"] = "true";
    trueMap["python"] = "True";
    falseMap["xmlflat"] = "false";
    falseMap["octave"] = "false";
    falseMap["python"] = "False";
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"project.svg").string()));
  }

  Project::~Project() {
    delete dss;
    delete solver;
  }

  void Project::removeXMLElements() {
    auto *cele = E(element)->getFirstCommentChild();
    if(cele)
      element->removeChild(cele);
    DOMElement *ele = element->getFirstElementChild();
    if(E(ele)->getTagName()==PV%"evaluator")
      element->removeChild(ele);
  }

  DOMElement* Project::createXMLElement(DOMNode *parent) {
    auto *doc=static_cast<DOMDocument*>(parent);
    element=D(doc)->createElement(getXMLType());
    E(element)->setAttribute("name","Project");
    name = "Project";
    parent->insertBefore(element, nullptr);
    QSettings settings;
    if(settings.value("mainwindow/options/defaultevaluator", 1).toInt()==1) {
      evaluator = "python";
      DOMElement *ele = D(doc)->createElement(PV%"evaluator");
      DOMText *text = doc->createTextNode(X()%"python");
      ele->insertBefore(text, nullptr);
      element->insertBefore(ele, nullptr);
    }
    setDynamicSystemSolver(new DynamicSystemSolver);
    dss->createXMLElement(element);
    setSolver(new DOPRI5Integrator);
    solver->createXMLElement(element);
    return element;
  }

  void Project::create() {
    EmbedItemData::create();
    DOMElement *ele = element->getFirstElementChild();
    if(E(ele)->getTagName()==PV%"evaluator") {
      setEvaluator(X()%E(ele)->getFirstTextChild()->getData());
      ele = ele->getNextElementSibling();
    }
    setDynamicSystemSolver(Embed<DynamicSystemSolver>::create(ele,this));
    dss->create();
    ele = ele->getNextElementSibling();
    setSolver(Embed<Solver>::create(ele,this));
    solver->create();
  }

  DOMElement* Project::processIDAndHref(DOMElement *element) {
    element = EmbedItemData::processIDAndHref(element);
    DOMElement* ele0 = element->getFirstElementChild();
    if(E(ele0)->getTagName()==PV%"evaluator")
      ele0 = ele0->getNextElementSibling();
    dss->processIDAndHref(ele0);
    ele0 = ele0->getNextElementSibling();
    solver->processIDAndHref(ele0);
    return element;
  }

  void Project::setDynamicSystemSolver(DynamicSystemSolver *dss_) {
    delete dss;
    dss = dss_;
    dss->setProject(this);
  }

  void Project::setSolver(Solver *solver_) {
    delete solver;
    solver = solver_;
    solver->setProject(this);
  }

  DOMElement* Project::createEmbedXMLElement() {
    if(not getEmbedXMLElement()) {
      DOMDocument *doc=element->getOwnerDocument();
      setEmbedXMLElement(D(doc)->createElement(PV%"Embed"));
      doc->removeChild(getXMLElement());
      doc->insertBefore(getEmbedXMLElement(),nullptr);
      getEmbedXMLElement()->insertBefore(getXMLElement(),nullptr);
    }
    return getEmbedXMLElement();
  }

  void Project::maybeRemoveEmbedXMLElement() {
    if(embed and not getNumberOfParameters()) {
      DOMElement *param = E(embed)->getFirstElementChildNamed(PV%"Parameter");
      if(param) {
        DOMNode *ps = param->getPreviousSibling();
        if(ps and X()%ps->getNodeName()=="#text")
          embed->removeChild(ps);
        embed->removeChild(param);
      }
      if(not E(embed)->hasAttribute("count") and not E(embed)->hasAttribute("counterName") and
         not E(embed)->hasAttribute("href") and not E(embed)->hasAttribute("parameterHref") and
         not E(embed)->hasAttribute("onlyif")) {
        DOMDocument *doc=element->getOwnerDocument();
        doc->removeChild(embed);
        doc->insertBefore(element,nullptr);
        embed = nullptr;
      }
    }
  }
}
