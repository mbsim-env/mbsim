/* Copyright (C) 2004-2014 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include <cassert>
#include <cfenv>
#include <mbsim/element.h>
#include <mbsim/objectfactory_impl.h>
#include <mbsim/frames/frame.h>
#include <mbsim/contours/contour.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/objects/object.h>
#include <mbsim/links/link.h>
#include <mbsim/constraints/constraint.h>
#include <mbsim/observers/observer.h>
#include <mbsim/functions/function.h>
#include <mbsim/mbsim_event.h>
#include <mbsim/utils/eps.h>
#include "mbsim/utils/xmlutils.h"
#include <hdf5serie/simpleattribute.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

#if !defined(_WIN32) && !defined(NDEBUG)
  // enable FPE of everything which load libmbsim.so -> this enables it automaticaly for all source examples
  static struct EnableFPE {
    EnableFPE() {
      assert(feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW)!=-1);
    }
  } enableFPE;
#endif

  // we use none signaling (quiet) NaN values for double in MBSim -> Throw compile error if these do not exist.
  static_assert(numeric_limits<double>::has_quiet_NaN, "This platform does not support quiet NaN for double.");

  // this is the root element which uses PlotFeature, hence we explicit instantate the enum factory here.
  template class EnumFactory<PlotFeatureEnum>;

  const PlotFeatureEnum plotRecursive;
  const PlotFeatureEnum openMBV;
  const PlotFeatureEnum debug;

  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, plotRecursive)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, openMBV)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, debug)

  Element::Element(const string &name_) : Atom(), parent(0), name(name_), ds(0), plotVectorSerie(0), plotGroup(0) {
  }

  void Element::plot() {
    if(plotFeature[ref(plotRecursive)]) {
      if(plotColumns.size()>1) {
        plotVector.insert(plotVector.begin(), getTime());
        assert(plotColumns.size()==plotVector.size());
        plotVectorSerie->append(plotVector);
        plotVector.clear();
      }
    }
  }

  void Element::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit)
      updatePlotFeatures();
    else if(stage==plotting) {

      if(plotFeature[ref(plotRecursive)]) {
        bool plotData = false;
        for (auto& x: plotFeature) {
          if((x.first.get() != plotRecursive) and (x.first.get() != openMBV) and x.second) {
            plotData = true;
            break;
          }
        }

        if(plotData || !plotAttribute.empty()) {
          if(parent && not parent->getPlotGroup()) {
            parent->createPlotGroup();
          }
          if(not getPlotGroup())
            createPlotGroup();
          if(plotColumns.size()>1) {
            // copy plotColumns to a std::vector
            vector<string> dummy; copy(plotColumns.begin(), plotColumns.end(), insert_iterator<vector<string>>(dummy, dummy.begin()));
            plotVectorSerie=plotGroup->createChildObject<H5::VectorSerie<double>>("data")(dummy.size());
            plotVectorSerie->setColumnLabel(dummy);
            plotVectorSerie->setDescription("Default dataset for class: "+boost::core::demangle(typeid(*this).name()));
          }
          plotVector.clear();
          plotVector.reserve(plotColumns.size()); // preallocation
        }

        // plot attributes
        for(auto &nameVariant : plotAttribute) {
          visit([this, &nameVariant](const auto &v) {
            using PlainType = decay_t<decltype(v)>;
            if constexpr (is_same_v<PlainType, monostate>) {
              plotGroup->createChildAttribute<H5::SimpleAttribute<int>>(nameVariant.first)();
            }
            else if constexpr (is_same_v<PlainType, int> || is_same_v<PlainType, double> || is_same_v<PlainType, string> ) {
              H5::SimpleAttribute<PlainType> *attr=plotGroup->createChildAttribute<H5::SimpleAttribute<PlainType>>(nameVariant.first)();
              attr->write(v);
            }
            else if constexpr (is_same_v<PlainType, vector<int>> || is_same_v<PlainType, vector<double>>) {
              H5::SimpleAttribute<PlainType> *attr=plotGroup->createChildAttribute<H5::SimpleAttribute<PlainType>>(nameVariant.first)(v.size());
              attr->write(v);
            }
            else if constexpr (is_same_v<PlainType, vector<vector<double>>>) {
              H5::SimpleAttribute<PlainType> *attr=plotGroup->createChildAttribute<H5::SimpleAttribute<PlainType>>(nameVariant.first)(v.size(), v.size() > 0 ? v[0].size() : 0);
              attr->write(v);
            }
          }, nameVariant.second);
        }
      }
    }
  }

  void Element::createPlotGroup() {
    plotGroup=parent->getPlotGroup()->createChildObject<H5::Group>(name)();
    plotGroup->createChildAttribute<H5::SimpleAttribute<string>>("Description")()->write("Object of class: "+boost::core::demangle(typeid(*this).name()));
    plotColumns.insert(plotColumns.begin(), "time");
  }

  void Element::updatePlotFeatures() {
    for (auto& x: parent->plotFeatureForChildren) {
      auto it=plotFeature.find(ref(x.first));
      if(it==plotFeature.end())
        plotFeature[ref(x.first)]=x.second;
    }
    for (auto& x: parent->plotFeatureForChildren) {
      auto it=plotFeatureForChildren.find(ref(x.first));
      if(it==plotFeatureForChildren.end())
        plotFeatureForChildren[ref(x.first)]=x.second;
    }
  }

  namespace {
    string containerName(const Element *e) {
      if(dynamic_cast<const Frame*>       (e)) return "Frame";
      if(dynamic_cast<const Contour*>     (e)) return "Contour";
      if(dynamic_cast<const Object*>      (e)) return "Object";
      if(dynamic_cast<const Link*>        (e)) return "Link";
      if(dynamic_cast<const Constraint*>  (e)) return "Constraint";
      if(dynamic_cast<const Group*>       (e)) return "Group";
      if(dynamic_cast<const Observer*>    (e)) return "Observer";
      if(dynamic_cast<const FunctionBase*>(e)) return "Function";
      // note we can not throw a MBSimError here since containerName and getPath is used in MBSimError itself
      // -> endless recursive call (however this error is a internal one)
      throw runtime_error("Internal error: Unknown object type.");
    }
  }

  string Element::getPath(const Element *relTo, string sep) const {
    try {
      // compose a absolute path
      if(!relTo) {
        // after init stage reorganizeHierarchy just return the store path since the hierarchy is changed now
        if(!path.empty())
          return path;

        // before the init stage reorganizeHierarchy compose the path dynamically

        // a parent exists -> return parent path + this elements sub path
        if(parent) {
          string parentPath=parent->getPath(NULL, sep);
          return parentPath+(parentPath==sep?"":sep)+containerName(this)+"["+getName()+"]";
        }
        // no parent exits and its a DynamicSystemSolver (we can generate a absolute path)
        if(dynamic_cast<const DynamicSystemSolver*>(this))
          return sep; // return the root separator
        // no parent exits and its not a DynamicSystemSolver (we can not generate a absolute path ->
        // append address to local name to have a unique local name!)
        stringstream str;
        str<<containerName(this)<<"["<<getName()<<"(with_ID_"<<this<<")]";
        return str.str();
      }
      // compose a relative path
      else {
        // get absolute path of this object and relTo (get it relative to the top level (remove the leading /))
        string thisPath=getPath(NULL, sep).substr(1);
        string relToPath=relTo->getPath(NULL, sep).substr(1)+sep;
        // check for "real" absolute path (see above)
        if(thisPath.substr(0, sep.length())!=sep || relToPath.substr(0, sep.length())!=sep)
          throwError("Can not generate a relative path: at least one element is not part of a DynamicSystemSolver");
        // remove sub path which are equal in both
        while(1) {
          size_t thisIdx=thisPath.find(sep);
          size_t relToIdx=relToPath.find(sep);
          if(thisPath.substr(0, thisIdx)==relToPath.substr(0, relToIdx)) {
            thisPath=thisPath.substr(thisIdx+1);
            relToPath=relToPath.substr(relToIdx+1);
          }
          else
            break;
        }
        // replace all sub path in relToPath with ".."
        string dotPath;
        size_t relToIdx=0;
        while((relToIdx=relToPath.find(sep, relToIdx+1))!=string::npos)
          dotPath+=".."+sep;
        // return the relative path
        return dotPath+thisPath;
      }
    }
    catch(MBSimError &ex) {
      // we convert every possible MBSimError exception here to a runtime_error since
      // we can not throw a MBSimError here since containerName and getPath is used in MBSimError itself
      // -> endless recursive call (however the code above should not throw any exception)
      throw runtime_error(ex.what());
    }
  }

  void Element::setPlotFeature(const PlotFeatureEnum &pf, bool value) {
    plotFeature[ref(pf)] = value;
  }

  void Element::setPlotFeatureForChildren(const PlotFeatureEnum &pf, bool value) {
    plotFeatureForChildren[ref(pf)] = value;
  }

  void Element::initializeUsingXML(DOMElement *element) {
    // set the XML location of this element which can be used, later, by exceptions.
    domEvalError=DOMEvalException("", element);

    if(E(element)->hasAttribute("name")) // their are element with no name e.g. Function's
      setName(E(element)->getAttribute("name"));

    // plot feature
    DOMElement *e;
    e=element->getFirstElementChild();
    while(e && (E(e)->getTagName()==MBSIM%"plotFeature" ||
                E(e)->getTagName()==MBSIM%"plotFeatureForChildren" ||
                E(e)->getTagName()==MBSIM%"plotFeatureRecursive")) {
      auto pf=getPlotFeatureFromXML(e);
      if(E(e)->getTagName()==MBSIM%"plotFeature") plotFeature[pf.first] = pf.second;
      else if(E(e)->getTagName()==MBSIM%"plotFeatureForChildren") plotFeatureForChildren[pf.first] = pf.second;
      else if(E(e)->getTagName()==MBSIM%"plotFeatureRecursive") {
        plotFeature[pf.first] = pf.second;
        plotFeatureForChildren[pf.first] = pf.second;
      }

      e=e->getNextElementSibling();
    }

    // plot attributes
    while(e && (E(e)->getTagName()==MBSIM%"plotAttribute" ||
                E(e)->getTagName()==MBSIM%"plotAttributeInt" ||
                E(e)->getTagName()==MBSIM%"plotAttributeFloat" ||
                E(e)->getTagName()==MBSIM%"plotAttributeString" ||
                E(e)->getTagName()==MBSIM%"plotAttributeIntVector" ||
                E(e)->getTagName()==MBSIM%"plotAttributeFloatVector" ||
                E(e)->getTagName()==MBSIM%"plotAttributeFloatMatrix")) {
      auto name=E(e)->getAttribute("name");
      if     (E(e)->getTagName()==MBSIM%"plotAttribute")             setPlotAttribute(name);
      else if(E(e)->getTagName()==MBSIM%"plotAttributeInt")          setPlotAttribute(name, E(e)->getText<int>());
      else if(E(e)->getTagName()==MBSIM%"plotAttributeFloat")        setPlotAttribute(name, E(e)->getText<double>());
      else if(E(e)->getTagName()==MBSIM%"plotAttributeString")       { auto s=E(e)->getText<string>(); setPlotAttribute(name, s.substr(1, s.length()-2)); }
      else if(E(e)->getTagName()==MBSIM%"plotAttributeIntVector")    setPlotAttribute(name, E(e)->getText<vector<int>>());
      else if(E(e)->getTagName()==MBSIM%"plotAttributeFloatVector")  setPlotAttribute(name, E(e)->getText<vector<double>>());
      else if(E(e)->getTagName()==MBSIM%"plotAttributeFloatMatrix")  setPlotAttribute(name, E(e)->getText<vector<vector<double>>>());

      e=e->getNextElementSibling();
    }
  }

  int Element::computeLevel() {
    int lOld = 0;
    for (unsigned int i = 0; i < dependency.size(); i++) {
      int lNew = dependency[i]->computeLevel() + 1;
      if (lNew > lOld) {
        lOld = lNew;
      }
    }
    return lOld;
  }

  const double& Element::getTime() const {
    return ds->getTime();
  }

  double Element::getStepSize() const {
    return ds->getStepSize();
  }

  void Element::addToPlot(const string &name) {
    plotColumns.emplace_back(name);
  }

  void Element::addToPlot(const string &name, int size) {
    for(int i=baseIndexForPlot; i<baseIndexForPlot+size; i++)
      plotColumns.emplace_back(name+" ("+to_string(i)+")");
  }

  void Element::addToPlot(const string &name, const vector<string> &iname) {
    for(size_t i=0; i<iname.size(); i++)
      plotColumns.emplace_back(name+" ("+iname[i]+")");
  }

  Element* Element::getByPathElement(const std::string &path, bool initialCaller) const {
    try {
      if(path.substr(0, 1) == "/") { // if absolute path ...
        if(parent) // .. and a parent exists ...
          return parent->getByPathElement(path, false); // ... than call getByPath of the parent (walk to the top)
        else // .. and no parent exits ...
          return getByPathElement(path.substr(1), false); // ... we are at top and call getByPath again with the leading "/" removed (call relative to top)
      }
      else if (path.substr(0, 3) == "../") { // if relative path to parent ...
        if(!parent)
          throwError("Reference to parent '"+path+"' requested but already at the root node.");
        return parent->getByPathElement(path.substr(3), false); // ... call getByPath of the parent with the leading "../" removed
      }
      else { // if relative path to a child ...
        // extract the first path and all other paths (rest)
        size_t idx=path.find('/');
        std::string first=path.substr(0, idx);
        std::string rest;
        if(idx!=std::string::npos)
          rest=path.substr(idx+1);
        // get the object of the first child path by calling the virtual function getChildByContainerAndName
        size_t pos0=first.find('[');
        if(pos0==std::string::npos)
          throwError("Syntax error in subreference '"+first+"': no [ found.");
        std::string container=first.substr(0, pos0);
        if(first[first.size()-1]!=']')
          throwError("Syntax error in subreference '"+first+"': not ending with ].");
        std::string name=first.substr(pos0+1, first.size()-pos0-2);
        Element *e=getChildByContainerAndName(container, name);
        // if their are other child paths call getByPath of e for this
        if(!rest.empty())
          return e->getByPathElement(rest, false);
        // this is the last relative path -> check type and return
        return e;
      }
    }
    catch(MBSimError &ex) {
      if(initialCaller)
        throwError("Evaluation of refernece '"+path+"' failed: Message from "+
          ex.getPath()+": "+ex.getErrorMessage());
      else
        throw ex;
    }
  }

}
