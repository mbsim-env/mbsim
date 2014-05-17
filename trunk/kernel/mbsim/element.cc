/* Copyright (C) 2004-2009 MBSim Development Team
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
 *          rzander@users.berlios.de
 */

#include <config.h>
#include <mbsim/element.h>
#include <mbsim/frame.h>
#include <mbsim/contour.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/object.h>
#include <mbsim/link.h>
#include <mbsim/observer.h>
#include <mbsim/mbsim_event.h>
#include <mbsim/utils/eps.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  Element::Element(const string &name_) : Atom(), parent(0), name(name_), ds(0), plotVectorSerie(0), plotGroup(0) {
    for(int i=0; i<LASTPLOTFEATURE; i++) {
      plotFeature[(PlotFeature)i]=unset;
      plotFeatureForChildren[(PlotFeature)i]=unset;
    }
  }

  Element::~Element() {
    delete plotGroup;
    delete plotVectorSerie;
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
    if(stage==MBSim::plot) {
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
            plotVectorSerie=new H5::VectorSerie<double>;
            // copy plotColumns to a std::vector
            vector<string> dummy; copy(plotColumns.begin(), plotColumns.end(), insert_iterator<vector<string> >(dummy, dummy.begin()));
            plotVectorSerie->create(*plotGroup,"data",dummy);
            plotVectorSerie->setDescription("Default dataset for class: "+getType());
          }
          plotVector.clear();
          plotVector.reserve(plotColumns.size()); // preallocation
        }
      }
    }
  }

  void Element::createPlotGroup() {
    plotGroup=new H5::Group(parent->getPlotGroup()->createGroup(name));
    H5::SimpleAttribute<string>::setData(*plotGroup, "Description", "Object of class: "+getType());

    plotColumns.insert(plotColumns.begin(), "Time");
  }

  void Element::updatePlotFeatures() {
    for(int i=0; i<LASTPLOTFEATURE; i++) {
      if(getPlotFeature((PlotFeature)i)==unset) setPlotFeature((PlotFeature)i, parent->getPlotFeatureForChildren((PlotFeature)i));
      if(getPlotFeatureForChildren((PlotFeature)i)==unset) setPlotFeatureForChildren((PlotFeature)i, parent->getPlotFeatureForChildren((PlotFeature)i));
    }
  }

  string Element::getPath(char pathDelim) {
    return parent?parent->getPath()+pathDelim+name:name;
  }

  string Element::getXMLPath(MBSim::Element *ref, bool rel) {
    if(rel) {
      vector<Element*> e0, e1;
      Element* element = ref;
      e0.push_back(element);
      while(!dynamic_cast<DynamicSystemSolver*>(element)) {
        element = element->getParent();
        e0.push_back(element);
      }
      element = getParent();
      e1.push_back(element);
      while(!dynamic_cast<DynamicSystemSolver*>(element)) {
        element = element->getParent();
        e1.push_back(element);
      }
      int imatch=0;
      for(vector<Element*>::iterator i0 = e0.end()-1, i1 = e1.end()-1 ; (i0 != e0.begin()-1) && (i1 != e1.begin()-1) ; i0--, i1--) 
        if(*i0 == *i1) imatch++;
      string type;
      if(dynamic_cast<Frame*>(this))
        type = "Frame";
      else if(dynamic_cast<Contour*>(this))
        type = "Contour";
      else if(dynamic_cast<Group*>(this))
        type = "Group";
      else if(dynamic_cast<Object*>(this))
        type = "Object";
      else if(dynamic_cast<Link*>(this))
        type = "Link";
      else if(dynamic_cast<Observer*>(this))
        type = "Observer";
      else 
        type = getType();
      string str = type + "[" + getName() + "]";
      for(vector<Element*>::iterator i1 = e1.begin() ; i1 != e1.end()-imatch ; i1++) {
        if(dynamic_cast<Group*>(*i1))
          str = string("Group[") + (*i1)->getName() + "]/" + str;
        else if(dynamic_cast<Object*>(*i1))
          str = string("Object[") + (*i1)->getName() + "]/" + str;
        else if(dynamic_cast<Link*>(*i1))
          str = string("Link[") + (*i1)->getName() + "]/" + str;
        else if(dynamic_cast<Observer*>(*i1))
          str = string("Observer[") + (*i1)->getName() + "]/" + str;
        else
          str = "";
      }
      for(int i=0; i<int(e0.size())-imatch; i++)
        str = "../" + str;
      return str;
    } else {
      string type;
      if(dynamic_cast<Frame*>(this))
        type = "Frame";
      else if(dynamic_cast<Contour*>(this))
        type = "Contour";
      else if(dynamic_cast<Group*>(this))
        type = "Group";
      else if(dynamic_cast<Object*>(this))
        type = "Object";
      else if(dynamic_cast<Link*>(this))
        type = "Link";
      else if(dynamic_cast<Observer*>(this))
        type = "Observer";
      else 
        type = getType();
      string str = type + "[" + getName() + "]";
      Element* element = parent;
      while(!dynamic_cast<DynamicSystemSolver*>(element)) {
        if(dynamic_cast<Group*>(element))
          str = string("Group[") + element->getName() + "]/" + str;
        else if(dynamic_cast<Object*>(element))
          str = string("Object[") + element->getName() + "]/" + str;
        else if(dynamic_cast<Link*>(element))
          str = string("Link[") + element->getName() + "]/" + str;
        else if(dynamic_cast<Observer*>(element))
          str = string("Observer[") + element->getName() + "]/" + str;
        else
          str = "";
        element = element->getParent();
      }
      str = "/" + str;
      return str;
    }
  }

  void Element::initializeUsingXML(DOMElement *element) {
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
        throw DOMEvalException(str.str(), e);
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
        throw DOMEvalException(str.str(), e);
      }
      if(E(e)->getTagName()==MBSIM%"plotFeature") setPlotFeature(feature, status);
      if(E(e)->getTagName()==MBSIM%"plotFeatureForChildren") setPlotFeatureForChildren(feature, status);
      if(E(e)->getTagName()==MBSIM%"plotFeatureRecursive") setPlotFeatureRecursive(feature, status);
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

}

