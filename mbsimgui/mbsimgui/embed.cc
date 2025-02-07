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
#include <iterator>

using namespace std;
using namespace xercesc;
using namespace MBXMLUtils;

namespace MBSimGUI {

  template <>
    DynamicSystemSolver* Embed<DynamicSystemSolver>::create(DOMElement *element) {
      return new DynamicSystemSolver; // this object is very spezial -> do not use the ObjectFactory
    }

  template <>
    Group* Embed<Group>::create(DOMElement *element) {
      return ObjectFactory::getInstance().create<Group>(element);
    }

  template <>
    Contour* Embed<Contour>::create(DOMElement *element) {
      return ObjectFactory::getInstance().create<Contour>(element);
    }

  template <>
    FixedRelativeFrame* Embed<FixedRelativeFrame>::create(DOMElement *element) {
      return ObjectFactory::getInstance().create<FixedRelativeFrame>(element);
    }

  template <>
    NodeFrame* Embed<NodeFrame>::create(DOMElement *element) {
      return ObjectFactory::getInstance().create<NodeFrame>(element);
    }

  template <>
    Object* Embed<Object>::create(DOMElement *element) {
      return ObjectFactory::getInstance().create<Object>(element);
    }

  template <>
    Link* Embed<Link>::create(DOMElement *element) {
      return ObjectFactory::getInstance().create<Link>(element);
    }

  template <>
    Constraint* Embed<Constraint>::create(DOMElement *element) {
      return ObjectFactory::getInstance().create<Constraint>(element);
    }

  template <>
    Observer* Embed<Observer>::create(DOMElement *element) {
      return ObjectFactory::getInstance().create<Observer>(element);
    }

  template <>
    Solver* Embed<Solver>::create(DOMElement *element) {
      return ObjectFactory::getInstance().create<Solver>(element);
    }

  template <>
    Project* Embed<Project>::create(DOMElement *element) {
      return new Project;
    }

  optional<QFileInfo> getQFileInfoForEmbed(DOMElement *ele1, const string &attr, const vector<MainWindow::ParameterLevel> &parameterLevels) {
    string filename;
    try{
      // MBSimGUI cannot handle href/parameterHref attribures which depend on a parameter.
      // Hence, if the attribute is not a plain file we return no QFileInfo object which will create a UnknownObject later
      // (instead of a EmbedObject) for this Embed Element.
      auto code = E(ele1)->getAttribute(attr);
      auto idx = code.find('{');
      if(idx!=string::npos && (idx==0 || code[idx-1]!='\\'))
        return {};
      // If the above restriction is lifted we still have the problem in MBSimGUI that it cannot handle the case when a
      // href/parameterHref attribute resolves to multiple different filenames. This can be the case the attribute depends on
      // a counterName parameter of a previous Embed element.
      // The following code will handle this and create a UnknownObject for such Embed element (just like above)
      // For now we can just return the attribute (we have ensured above that is does not depend on any parameter
      filename = code;
      // auto possibleValues = MainWindow::evaluateForAllArrayPattern(parameterLevels, code, ele1);
      // set<string> uniqueValues;
      // transform(possibleValues.second.begin(), possibleValues.second.end(), inserter(uniqueValues, uniqueValues.begin()), [](auto &x) {
      //   return mw->eval->cast<string>(x.second);
      // });
      // if(uniqueValues.size()==1)
      //   href = *uniqueValues.begin();
      // else
      //   return {};
    }
    catch(DOMEvalException &e) {
      mw->setExitBad();
      cerr << e.getMessage() << endl;
    }
    catch(...) {
      mw->setExitBad();
      cerr << "Unknwon error" << endl;
    }
    auto docFilename = X()%ele1->getOwnerDocument()->getDocumentURI();
    auto fileInfo = QFileInfo(QDir(QFileInfo(QUrl(docFilename.c_str()).path()).canonicalPath()).absoluteFilePath(filename.c_str()));
    if(!fileInfo.exists())
      QMessageBox::warning(nullptr, "Import/Reference model file",
                                    ("The imported/referenced model file has a reference to another file which cannot be found:\n"
                                     "\n'"+filename+"'\n\n"
                                     "See the errors in the 'MBSim Echo Area'.\n"
                                     "(This may happen e.g. if a model is imported which has relative path reference to other files)").c_str());
    return fileInfo;
  };

}
