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
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/object.h>
#include <mbsim/mbsim_event.h>
#include <mbsim/utils/eps.h>
#include "mbxmlutilstinyxml/tinynamespace.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Element::Element(const string &name_) : parent(0), name(name_), ds(0), plotVectorSerie(0), plotGroup(0) {
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
      if(plotVectorSerie) delete plotVectorSerie;
      delete plotGroup;
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
      string str = getType()+ "[" + getName() + "]";
      for(vector<Element*>::iterator i1 = e1.begin() ; i1 != e1.end()-imatch ; i1++) {
        if(dynamic_cast<Group*>(*i1))
          str = string("Group[") + (*i1)->getName() + "]/" + str;
        else if(dynamic_cast<Object*>(*i1))
          str = string("Object[") + (*i1)->getName() + "]/" + str;
        else
          throw;
      }
      for(int i=0; i<int(e0.size())-imatch; i++)
        str = "../" + str;
      return str;
    } else {
      string str = getType()+ "[" + getName() + "]";
      Element* element = getParent();
      while(!dynamic_cast<DynamicSystemSolver*>(element)) {
        if(dynamic_cast<Group*>(element))
          str = string("Group[") + element->getName() + "]/" + str;
        else if(dynamic_cast<Object*>(element))
          str = string("Object[") + element->getName() + "]/" + str;
        else
          throw;
        element = element->getParent();
      }
      str = "/" + str;
      return str;
    }
  }

  void Element::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement();
    while(e && (e->ValueStr()==MBSIMNS"plotFeature" ||
                e->ValueStr()==MBSIMNS"plotFeatureForChildren" ||
                e->ValueStr()==MBSIMNS"plotFeatureRecursive")) {
      PlotFeatureStatus status;
      if(e->Attribute("feature")[0]=='+') status=enabled; else status=disabled;
      PlotFeature feature=plotRecursive;
      if(string(e->Attribute("feature")).substr(1)=="plotRecursive") feature=plotRecursive;
      if(string(e->Attribute("feature")).substr(1)=="separateFilePerGroup") feature=separateFilePerGroup;
      if(string(e->Attribute("feature")).substr(1)=="state") feature=state;
      if(string(e->Attribute("feature")).substr(1)=="stateDerivative") feature=stateDerivative;
      if(string(e->Attribute("feature")).substr(1)=="notMinimalState") feature=notMinimalState;
      if(string(e->Attribute("feature")).substr(1)=="rightHandSide") feature=rightHandSide;
      if(string(e->Attribute("feature")).substr(1)=="globalPosition") feature=globalPosition;
      if(string(e->Attribute("feature")).substr(1)=="globalVelocity") feature=globalVelocity;
      if(string(e->Attribute("feature")).substr(1)=="globalAcceleration") feature=globalAcceleration;
      if(string(e->Attribute("feature")).substr(1)=="energy") feature=energy;
      if(string(e->Attribute("feature")).substr(1)=="openMBV") feature=openMBV;
      if(string(e->Attribute("feature")).substr(1)=="generalizedLinkForce") feature=generalizedLinkForce;
      if(string(e->Attribute("feature")).substr(1)=="linkKinematics") feature=linkKinematics;
      if(string(e->Attribute("feature")).substr(1)=="stopVector") feature=stopVector;
      if(string(e->Attribute("feature")).substr(1)=="debug") feature=debug;
      if(e->ValueStr()==MBSIMNS"plotFeature") setPlotFeature(feature, status);
      if(e->ValueStr()==MBSIMNS"plotFeatureForChildren") setPlotFeatureForChildren(feature, status);
      if(e->ValueStr()==MBSIMNS"plotFeatureRecursive") setPlotFeatureRecursive(feature, status);
      e=e->NextSiblingElement();
    }
  }

  TiXmlElement* Element::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
    parent->LinkEndChild(ele0);
    ele0->SetAttribute("name", getName());
    return ele0;
  }

  // some convenience function for XML
  double Element::getDouble(TiXmlElement *e) {
    Mat m=Mat(e->GetText());
    if(m.rows()==1 && m.cols()==1)
      return m(0,0);
    else {
      ostringstream str;
      str<<": Obtained matrix of size "<<m.rows()<<"x"<<m.cols()<<" ("<<e->GetText()<<") "<<
           "where a scalar was requested for element "<<e->ValueStr();
      TiXml_location(e, "", str.str());
      throw MBSimError("Wrong type"+str.str());
    }
    return NAN;
  }

  int Element::getInt(TiXmlElement *e) {
    Mat m=Mat(e->GetText());
    if(m.rows()==1 && m.cols()==1)
      return lround(m(0,0));
    else {
      ostringstream str;
      str<<": Obtained matrix of size "<<m.rows()<<"x"<<m.cols()<<" ("<<e->GetText()<<") "<<
           "where a scalar integer was requested for element "<<e->ValueStr();
      TiXml_location(e, "", str.str());
      throw MBSimError("Wrong type"+str.str());
    }
    return 0;
  }

  bool Element::getBool(TiXmlElement *e) {
    if(e->GetText()==string("true") || e->GetText()==string("1"))
      return true;
    else if(e->GetText()==string("false") || e->GetText()==string("0"))
      return false;
    else {
      ostringstream str;
      str<<": Obtained "<<e->GetText()<<" where a boolean was requested for element "<<e->ValueStr();
      TiXml_location(e, "", str.str());
      throw MBSimError("Wrong type"+str.str());
    }
    return 0;
  }

  Vec3 Element::getVec3(TiXmlElement *e) {
    Vec x = getVec(e,3);
    return Vec3(x);
  }

  Vec Element::getVec(TiXmlElement *e, int rows) {
    Mat m=Mat(e->GetText());
    if((rows==0 || m.rows()==rows) && m.cols()==1)
      return m.col(0);
    else if(m.rows()) {
      ostringstream str;
      str<<": Obtained matrix of size "<<m.rows()<<"x"<<m.cols()<<" ("<<e->GetText()<<") "<<
           "where a vector of size "<<((rows==0)?-1:rows)<<" was requested for element "<<e->ValueStr();
      TiXml_location(e, "", str.str());
      throw MBSimError("Wrong type"+str.str());
    }
    return Vec();
  }

  Mat3V Element::getMat3V(TiXmlElement *e, int cols) {
    Mat A = getMat(e,3,cols);
    return Mat3V(A);
  }

  Mat Element::getMat(TiXmlElement *e, int rows, int cols) {
    Mat m=Mat(e->GetText());
    if((rows==0 || m.rows()==rows) && (cols==0 || m.cols()==cols))
      return m;
    else {
      ostringstream str;
      str<<": Obtained matrix of size "<<m.rows()<<"x"<<m.cols()<<" ("<<e->GetText()<<") "<<
           "where a matrix of size "<<((rows==0)?-1:rows)<<"x"<<((cols==0)?-1:cols)<<" was requested for element "<<e->ValueStr();
      TiXml_location(e, "", str.str());
      throw MBSimError("Wrong type"+str.str());
    }
    return Mat();
  }

  SqrMat3 Element::getSqrMat3(TiXmlElement *e) {
    SqrMat A = getSqrMat(e,3);
    return SqrMat3(A);
  }

  SqrMat Element::getSqrMat(TiXmlElement *e, int size) {
    Mat m=Mat(e->GetText());
    if((size==0 || m.rows()==size) && (size==0 || m.cols()==size) && m.rows()==m.cols())
      return SqrMat(m);
    else {
      ostringstream str;
      str<<": Obtained matrix of size "<<m.rows()<<"x"<<m.cols()<<" ("<<e->GetText()<<") "<<
           "where a square matrix of size "<<size<<" was requested for element "<<e->ValueStr();
      TiXml_location(e, "", str.str());
      throw MBSimError("Wrong type"+str.str());
    }
    return SqrMat();
  }

  fmatvec::SymMat3 Element::getSymMat3(TiXmlElement *e) {
    SymMat A = getSymMat(e,3);
    return SymMat3(A);
  }

  fmatvec::SymMat Element::getSymMat(TiXmlElement *e, int size) {
    Mat m=Mat(e->GetText());
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
      str<<": Obtained matrix of size "<<m.rows()<<"x"<<m.cols()<<" ("<<e->GetText()<<") "<<
           "where a symmetric matrix of size "<<size<<" was requested for element "<<e->ValueStr();
      TiXml_location(e, "", str.str());
      throw MBSimError("Wrong type"+str.str());
    }
    return SymMat();
  }

}

