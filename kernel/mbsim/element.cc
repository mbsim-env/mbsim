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

  // this is the root element which uses PlotFeature, hence we define static parts of the enum factory here.
  template<>
  map<FQN, reference_wrapper<const PlotFeatureEnum>> EnumFactory<PlotFeatureEnum>::reg={};

  const PlotFeatureEnum plotRecursive;
  const PlotFeatureEnum openMBV;
  const PlotFeatureEnum debug;
  const PlotFeatureEnum separateFilePerGroup;
  const PlotFeatureEnum energy;

  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, plotRecursive)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, openMBV)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, debug)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, separateFilePerGroup)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, energy)

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

  void Element::init(InitStage stage) {
    if(stage==preInit)
      updatePlotFeatures();
    else if(stage==plotting) {

      if(plotFeature[ref(plotRecursive)]) {
        unsigned int numEnabled=0;
        for (auto& x: plotFeature) {
          if((x.first.get() != plotRecursive) and (x.first.get() != openMBV) and x.second) {
            numEnabled++;
            break;
          }
        }

        if(numEnabled>0) {
          if(not parent->getPlotGroup()) {
            parent->createPlotGroup();
          }
          createPlotGroup();
          if(plotColumns.size()>1) {
            // copy plotColumns to a std::vector
            vector<string> dummy; copy(plotColumns.begin(), plotColumns.end(), insert_iterator<vector<string> >(dummy, dummy.begin()));
            plotVectorSerie=plotGroup->createChildObject<H5::VectorSerie<double> >("data")(dummy.size());
            plotVectorSerie->setColumnLabel(dummy);
            plotVectorSerie->setDescription("Default dataset for class: "+getType());
          }
          plotVector.clear();
          plotVector.reserve(plotColumns.size()); // preallocation
        }
      }
    }
  }

  void Element::createPlotGroup() {
    plotGroup=parent->getPlotGroup()->createChildObject<H5::Group>(name)();
    plotGroup->createChildAttribute<H5::SimpleAttribute<string> >("Description")()->write("Object of class: "+getType());

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
        str<<containerName(this)<<"["<<getName()<<"<with_ID_"<<this<<">]";
        return str.str();
      }
      // compose a relative path
      else {
        // get absolute path of this object and relTo (get it relative to the top level (remove the leading /))
        string thisPath=getPath(NULL, sep).substr(1);
        string relToPath=relTo->getPath(NULL, sep).substr(1)+sep;
        // check for "real" absolute path (see above)
        if(thisPath.substr(0, sep.length())!=sep || relToPath.substr(0, sep.length())!=sep)
          THROW_MBSIMERROR("Can not generate a relative path: at least one element is not part of a DynamicSystemSolver");
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
    // set the XML location stack of this element which can be used, later, by exceptions.
    DOMEvalException::generateLocationStack(element, locationStack);

    if(E(element)->hasAttribute("name")) // their are element with no name e.g. Function's
      setName(E(element)->getAttribute("name"));
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
  }

  // some convenience function for XML
  double Element::getDouble(const DOMElement *e) {
    Mat m=Mat((X()%E(e)->getFirstTextChild()->getData()).c_str());
    if(m.rows()==1 && m.cols()==1)
      return m(0,0);
    else {
      ostringstream str;
      str<<"Wrong type: Obtained matrix of size "<<m.rows()<<"x"<<m.cols()<<" ("<<X()%E(e)->getFirstTextChild()->getData()<<") "<<
           "where a scalar was requested.";
      throw DOMEvalException(str.str(), e);
    }
  }

  int Element::getInt(const DOMElement *e) {
    Mat m=Mat((X()%E(e)->getFirstTextChild()->getData()).c_str());
    if(m.rows()==1 && m.cols()==1)
      return lround(m(0,0));
    else {
      ostringstream str;
      str<<"Wrong type: Obtained matrix of size "<<m.rows()<<"x"<<m.cols()<<" ("<<X()%E(e)->getFirstTextChild()->getData()<<") "<<
           "where a scalar integer was requested for element "<<X()%e->getTagName();
      throw DOMEvalException(str.str(), e);
    }
    return 0;
  }

  bool Element::getBool(const DOMElement *e) {
    string text = X()%E(e)->getFirstTextChild()->getData();
    if(text=="true" || text=="1")
      return true;
    else if(text=="false" || text=="0")
      return false;
    else {
      ostringstream str;
      str<<"Wrong type: Obtained "<<X()%E(e)->getFirstTextChild()->getData()<<" where a boolean was requested for element "<<X()%e->getTagName();
      throw DOMEvalException(str.str(), e);
    }
    return 0;
  }

  Vec3 Element::getVec3(const DOMElement *e) {
    Vec x = getVec(e,3);
    return Vec3(x);
  }

  Vec Element::getVec(const DOMElement *e, int rows) {
    Mat m=Mat((X()%E(e)->getFirstTextChild()->getData()).c_str());
    if((rows==0 || m.rows()==rows) && m.cols()==1)
      return m.col(0);
    else if(m.rows()) {
      ostringstream str;
      str<<"Wrong type: Obtained matrix of size "<<m.rows()<<"x"<<m.cols()<<" ("<<X()%E(e)->getFirstTextChild()->getData()<<") "<<
           "where a vector of size "<<((rows==0)?-1:rows)<<" was requested for element "<<X()%e->getTagName();
      throw DOMEvalException(str.str(), e);
    }
    return Vec();
  }

  Mat3xV Element::getMat3xV(const DOMElement *e, int cols) {
    Mat A = getMat(e,3,cols);
    return Mat3xV(A);
  }

  Mat Element::getMat(const DOMElement *e, int rows, int cols) {
    Mat m=Mat((X()%E(e)->getFirstTextChild()->getData()).c_str());
    if((rows==0 || m.rows()==rows) && (cols==0 || m.cols()==cols))
      return m;
    else {
      ostringstream str;
      str<<"Wrong type: Obtained matrix of size "<<m.rows()<<"x"<<m.cols()<<" ("<<X()%E(e)->getFirstTextChild()->getData()<<") "<<
           "where a matrix of size "<<((rows==0)?-1:rows)<<"x"<<((cols==0)?-1:cols)<<" was requested for element "<<X()%e->getTagName();
      throw DOMEvalException(str.str(), e);
    }
    return Mat();
  }

  SqrMat3 Element::getSqrMat3(const DOMElement *e) {
    SqrMat A = getSqrMat(e,3);
    return SqrMat3(A);
  }

  SqrMat Element::getSqrMat(const DOMElement *e, int size) {
    Mat m=Mat((X()%E(e)->getFirstTextChild()->getData()).c_str());
    if((size==0 || m.rows()==size) && (size==0 || m.cols()==size) && m.rows()==m.cols())
      return SqrMat(m);
    else {
      ostringstream str;
      str<<"Wrong type: Obtained matrix of size "<<m.rows()<<"x"<<m.cols()<<" ("<<X()%E(e)->getFirstTextChild()->getData()<<") "<<
           "where a square matrix of size "<<size<<" was requested for element "<<X()%e->getTagName();
      throw DOMEvalException(str.str(), e);
    }
    return SqrMat();
  }

  fmatvec::SymMat3 Element::getSymMat3(const DOMElement *e) {
    SymMat A = getSymMat(e,3);
    return SymMat3(A);
  }

  fmatvec::SymMat Element::getSymMat(const DOMElement *e, int size) {
    Mat m=Mat((X()%E(e)->getFirstTextChild()->getData()).c_str());
    bool isSym=true;
    for(int i=0; i<min(m.rows(),m.cols()); i++) {
      for(int j=0; j<min(m.rows(),m.cols()); j++)
        if(fabs(m(i,j)-m(j,i))>epsroot()) { isSym=false; break; }
      if(isSym==false) break;
    }
    if((size==0 || m.rows()==size) && (size==0 || m.cols()==size) && m.rows()==m.cols() && isSym)
      return SymMat(m);
    else {
      ostringstream str;
      str<<"Wrong type: Obtained matrix of size "<<m.rows()<<"x"<<m.cols()<<" ("<<X()%E(e)->getFirstTextChild()->getData()<<") "<<
           "where a symmetric matrix of size "<<size<<" was requested for element "<<X()%e->getTagName();
      throw DOMEvalException(str.str(), e);
    }
    return SymMat();
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
  
}
