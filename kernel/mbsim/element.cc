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
#include <hdf5serie/simpleattribute.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  // we use none signaling (quiet) NaN values for double in MBSim -> Throw compile error if these do not exist.
  BOOST_STATIC_ASSERT_MSG(numeric_limits<double>::has_quiet_NaN, "This platform does not support quiet NaN for double.");

  Element::Element(const string &name_) : Atom(), parent(0), name(name_), ds(0), plotVectorSerie(0), plotGroup(0) {
    for(int i=0; i<LASTPLOTFEATURE; i++) {
      plotFeature[(PlotFeature)i]=unset;
      plotFeatureForChildren[(PlotFeature)i]=unset;
    }
  }

  Element::~Element() {
  }

  void Element::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(plotColumns.size()>1) {
        plotVector.insert(plotVector.begin(), t);
        assert(plotColumns.size()==plotVector.size());
        plotVectorSerie->append(plotVector);
        plotVector.clear();
      }
    }
  }

  void Element::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
    }
  }

  void Element::init(InitStage stage) {
    if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
        unsigned int numEnabled=0;
        int i=1;
        while ((i<LASTPLOTFEATURE) && (numEnabled==0)) {
          if (i!=openMBV)
            if (getPlotFeature((PlotFeature)i)==enabled)
              numEnabled++;
          i++;
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

    plotColumns.insert(plotColumns.begin(), "Time");
  }

  void Element::updatePlotFeatures() {
    for(int i=0; i<LASTPLOTFEATURE; i++) {
      if(getPlotFeature((PlotFeature)i)==unset) setPlotFeature((PlotFeature)i, parent->getPlotFeatureForChildren((PlotFeature)i));
      if(getPlotFeatureForChildren((PlotFeature)i)==unset) setPlotFeatureForChildren((PlotFeature)i, parent->getPlotFeatureForChildren((PlotFeature)i));
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
      PlotFeatureStatus status;
      if(E(e)->getAttribute("feature")[0]=='+') status=enabled;
      else if(E(e)->getAttribute("feature")[0]=='-') status=disabled;
      else {
        ostringstream str;
        str<<"Plot feature must start with '+' or '-' but is "<<E(e)->getAttribute("feature");
        throw DOMEvalException(str.str(), e, e->getAttributeNode(X()%"feature"));
      }
      PlotFeature feature=plotRecursive;
      if     (E(e)->getAttribute("feature").substr(1)=="plotRecursive") feature=plotRecursive;
      else if(E(e)->getAttribute("feature").substr(1)=="separateFilePerGroup") feature=separateFilePerGroup;
      else if(E(e)->getAttribute("feature").substr(1)=="state") feature=state;
      else if(E(e)->getAttribute("feature").substr(1)=="stateDerivative") feature=stateDerivative;
      else if(E(e)->getAttribute("feature").substr(1)=="notMinimalState") feature=notMinimalState;
      else if(E(e)->getAttribute("feature").substr(1)=="rightHandSide") feature=rightHandSide;
      else if(E(e)->getAttribute("feature").substr(1)=="globalPosition") feature=globalPosition;
      else if(E(e)->getAttribute("feature").substr(1)=="globalVelocity") feature=globalVelocity;
      else if(E(e)->getAttribute("feature").substr(1)=="globalAcceleration") feature=globalAcceleration;
      else if(E(e)->getAttribute("feature").substr(1)=="energy") feature=energy;
      else if(E(e)->getAttribute("feature").substr(1)=="openMBV") feature=openMBV;
      else if(E(e)->getAttribute("feature").substr(1)=="generalizedLinkForce") feature=generalizedLinkForce;
      else if(E(e)->getAttribute("feature").substr(1)=="linkKinematics") feature=linkKinematics;
      else if(E(e)->getAttribute("feature").substr(1)=="stopVector") feature=stopVector;
      else if(E(e)->getAttribute("feature").substr(1)=="debug") feature=debug;
      else {
        ostringstream str;
        str<<"Unknown plot feature: "<<E(e)->getAttribute("feature");
        throw DOMEvalException(str.str(), e, e->getAttributeNode(X()%"feature"));
      }
      if(E(e)->getTagName()==MBSIM%"plotFeature") setPlotFeature(feature, status);
      else if(E(e)->getTagName()==MBSIM%"plotFeatureForChildren") setPlotFeatureForChildren(feature, status);
      else if(E(e)->getTagName()==MBSIM%"plotFeatureRecursive") setPlotFeatureRecursive(feature, status);
      e=e->getNextElementSibling();
    }
  }

  DOMElement* Element::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0 = D(doc)->createElement(MBSIM%getType());
    parent->insertBefore(ele0, NULL);
    E(ele0)->setAttribute("name", getName());
    return ele0;
  }

  // some convenience function for XML
  double Element::getDouble(DOMElement *e) {
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

  int Element::getInt(DOMElement *e) {
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

  bool Element::getBool(DOMElement *e) {
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

  Vec3 Element::getVec3(DOMElement *e) {
    Vec x = getVec(e,3);
    return Vec3(x);
  }

  Vec Element::getVec(DOMElement *e, int rows) {
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

  Mat3xV Element::getMat3xV(DOMElement *e, int cols) {
    Mat A = getMat(e,3,cols);
    return Mat3xV(A);
  }

  Mat Element::getMat(DOMElement *e, int rows, int cols) {
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

  SqrMat3 Element::getSqrMat3(DOMElement *e) {
    SqrMat A = getSqrMat(e,3);
    return SqrMat3(A);
  }

  SqrMat Element::getSqrMat(DOMElement *e, int size) {
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

  fmatvec::SymMat3 Element::getSymMat3(DOMElement *e) {
    SymMat A = getSymMat(e,3);
    return SymMat3(A);
  }

  fmatvec::SymMat Element::getSymMat(DOMElement *e, int size) {
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

  double Element::getTime() {
    return ds->getTime();
  }
  
}

