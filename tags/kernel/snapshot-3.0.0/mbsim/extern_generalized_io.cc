/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: mfoerg@users.berlios.de
 */

#include "config.h"
#include "mbsim/extern_generalized_io.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/object.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  ExternGeneralizedIO::ExternGeneralizedIO(const string &name) : Link(name),
    connectedObject(NULL), qInd(0), uInd(0), m(0), a(0), t0(0), saved_connectedObject("") {
  }

  void ExternGeneralizedIO::updateh(double t) {
    if(type==CONSTANT) {
      connectedObject->geth()(uInd)+=la(0);
      for(unsigned int i=0; i<applyForceAlsoTo.size(); i++)
        applyForceAlsoTo[i].ref->geth()(applyForceAlsoTo[i].index)+=applyForceAlsoTo[i].factor*la(0);
    }
    else if(type==LINEAR) {
      la(0)=m*(t-t0)+a;
      connectedObject->geth()(uInd)+=la(0);
      for(unsigned int i=0; i<applyForceAlsoTo.size(); i++)
        applyForceAlsoTo[i].ref->geth()(applyForceAlsoTo[i].index)+=applyForceAlsoTo[i].factor*la(0);
    }
   
  }

  void ExternGeneralizedIO::updateg(double) {
    if(qInd>=0)
      g(0)=connectedObject->getq()(qInd);
    else
      g(0)=x(0);
  } 

  void ExternGeneralizedIO::updategd(double) {
    gd(0)=connectedObject->getu()(uInd);
  }

  void ExternGeneralizedIO::calcxSize() {
    xSize=qInd>=0?0:1;
  }

  void ExternGeneralizedIO::updatedx(double t, double dt) {
    if(qInd<0)
      xd(0)=connectedObject->getu()(uInd)*dt;
  }

  void ExternGeneralizedIO::updatexd(double t) {
    if(qInd<0)
      xd(0)=connectedObject->getu()(uInd);
  }

  void ExternGeneralizedIO::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_connectedObject!="")
        connectedObject=getByPath<Object>(saved_connectedObject);
      for(unsigned int i=0; i<applyForceAlsoTo.size(); i++)
        if(applyForceAlsoTo[i].saved_ref!="") {
          if(applyForceAlsoTo[i].saved_ref.substr(0,3)=="../")
            applyForceAlsoTo[i].ref=parent->getByPath<Object>(applyForceAlsoTo[i].saved_ref.substr(3));
          else
            applyForceAlsoTo[i].ref=parent->getByPath<Object>(applyForceAlsoTo[i].saved_ref);
        }
      Link::init(stage);
    }
    else if(stage==resize) {
      Link::init(stage);
      g.resize(1);
      gd.resize(1);
      la.resize(1); la(0)=0;
      if(qInd<0)
        x.resize(1);
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("la(0)");
        Link::init(stage);
      }
    }
    else if(stage==MBSim::calculateLocalInitialValues) {
      Link::init(stage);
      if(qInd>=0)
        g(0)=connectedObject->getq()(qInd);
      else
        g(0)=0;
      gd(0)=connectedObject->getu()(uInd);
    }
    else
      Link::init(stage);
  }

  void ExternGeneralizedIO::plot(double t,double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(la(0));
      Link::plot(t,dt);
    }
  }

  void ExternGeneralizedIO::initializeUsingXML(TiXmlElement *element) {
    Link::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"type");
    if(e) {
      string str=e->GetText();
      if(str=="\"constant\"") type=CONSTANT;
      else if(str=="\"linear\"") type=LINEAR;
    }
    else
      type=CONSTANT;
    saved_connectedObject=element->FirstChildElement(MBSIMNS"connectedObject")->Attribute("ref");
    qInd=getInt(element->FirstChildElement(MBSIMNS"qIndex"));
    uInd=getInt(element->FirstChildElement(MBSIMNS"uIndex"));
    for(e=element->FirstChildElement(MBSIMNS"applyForceAlsoTo"); e!=0; e=e->NextSiblingElement(MBSIMNS"applyForceAlsoTo")) {
      ApplyForceAlsoTo alsoTo;
      alsoTo.saved_ref=e->Attribute(("ref"));
      alsoTo.ref=0;
      alsoTo.factor=getDouble(e->FirstChildElement());
      alsoTo.index=getInt(e->FirstChildElement()->NextSiblingElement());
      applyForceAlsoTo.push_back(alsoTo);
    }
  }

  void ExternGeneralizedIO::addApplyForceAlsoTo(Object *ref, double factor, int index) {
    ApplyForceAlsoTo alsoTo;
    alsoTo.saved_ref="";
    alsoTo.ref=ref;
    alsoTo.factor=factor;
    alsoTo.index=index;
    applyForceAlsoTo.push_back(alsoTo);
  }

}
