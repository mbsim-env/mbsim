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
#include "embed.h"
#include "dynamic_system_solver.h"
#include "parameter.h"
#include "project.h"
#include "contour.h"
#include "link_.h"
#include "constraint.h"
#include "observer.h"
#include "solver.h"
#include "objectfactory.h"

namespace MBSimGUI {

  template <>
    DynamicSystemSolver* Embed<DynamicSystemSolver>::create(xercesc::DOMElement *element) {
      return new DynamicSystemSolver; // this object is very spezial -> do not use the ObjectFactory
    }

  template <>
    Group* Embed<Group>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Group>(element);
    }

  template <>
    Contour* Embed<Contour>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Contour>(element);
    }

  template <>
    FixedRelativeFrame* Embed<FixedRelativeFrame>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<FixedRelativeFrame>(element);
    }

  template <>
    NodeFrame* Embed<NodeFrame>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<NodeFrame>(element);
    }

  template <>
    Object* Embed<Object>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Object>(element);
    }

  template <>
    Link* Embed<Link>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Link>(element);
    }

  template <>
    Constraint* Embed<Constraint>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Constraint>(element);
    }

  template <>
    Observer* Embed<Observer>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Observer>(element);
    }

  template <>
    Solver* Embed<Solver>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Solver>(element);
    }

  template <>
    Project* Embed<Project>::create(xercesc::DOMElement *element) {
      return new Project;
    }

  QFileInfo getQFileInfoForEmbed(xercesc::DOMElement *ele1, const std::string &attr) {
    std::string href;
    try{
      href = mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(MBXMLUtils::E(ele1)->getAttribute(attr),ele1,false));
      href = href.substr(1,href.size()-2);
    }
    catch(MBXMLUtils::DOMEvalException &e) {
      mw->setExitBad();
      std::cerr << e.getMessage() << std::endl;
    }
    catch(...) {
      mw->setExitBad();
      std::cerr << "Unknwon error" << std::endl;
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

}
