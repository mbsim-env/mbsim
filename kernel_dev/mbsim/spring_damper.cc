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
#include "mbsim/spring_damper.h"
#include "mbsim/utils/eps.h"
#include "mbsim/objectfactory.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/objectfactory.h"
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  SpringDamper::SpringDamper(const string &name) : LinkMechanics(name), func(NULL), refFrame(NULL), forceDir(3)
#ifdef HAVE_OPENMBVCPPINTERFACE
    , coilspringOpenMBV(NULL)
#endif
  {}

  void SpringDamper::updateh(double) {
    la(0)=(*func)(g(0),gd(0));
    if(refFrame==0) // Point to Point
      WF[0]=n*la;
    else { // Directed
      Vec WforceDir=refFrame->getOrientation()*forceDir; // force direction in world system
      WF[0]=WforceDir*(trans(WforceDir)*(n*la)); // projected force in direction of WforceDir
    }
    WF[1]=-WF[0];
    for(unsigned int i=0; i<2; i++) {
      h[i]+=trans(frame[i]->getJacobianOfTranslation())*WF[i];
      hLink[i]+=trans(frame[i]->getJacobianOfTranslation())*WF[i];
    }
  }

  void SpringDamper::updateg(double) {
    Vec WrP0P1=frame[1]->getPosition() - frame[0]->getPosition();
    dist=nrm2(WrP0P1);
    if(dist>epsroot()) {
      n=WrP0P1/dist;
      g(0)=trans(n)*WrP0P1;
    }
    else
      g(0)=0;
  } 

  void SpringDamper::updategd(double) {
    if(dist>epsroot())
      gd(0)=trans(n)*(frame[1]->getVelocity() - frame[0]->getVelocity());  
    else
      gd(0)=0;
  }

  void SpringDamper::connect(Frame *frame0, Frame* frame1) {
    LinkMechanics::connect(frame0);
    LinkMechanics::connect(frame1);
  }

  void SpringDamper::init() {
    LinkMechanics::init();

    g.resize(1);
    gd.resize(1);
    la.resize(1);
  }

  void SpringDamper::initPlot() {
    updatePlotFeatures(parent);
    plotColumns.push_back("la(0)");
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(coilspringOpenMBV) {
        coilspringOpenMBV->setName(name);
        parent->getOpenMBVGrp()->addObject(coilspringOpenMBV);
      }
#endif
      LinkMechanics::initPlot();
    }
  }

  void SpringDamper::plot(double t,double dt) {
    plotVector.push_back(la(0));
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (coilspringOpenMBV) {
        Vec WrOToPoint;
        Vec WrOFromPoint;

        WrOFromPoint = frame[0]->getPosition();
        WrOToPoint   = frame[1]->getPosition();
        vector<double> data;
        data.push_back(t); 
        data.push_back(WrOFromPoint(0));
        data.push_back(WrOFromPoint(1));
        data.push_back(WrOFromPoint(2));
        data.push_back(WrOToPoint(0));
        data.push_back(WrOToPoint(1));
        data.push_back(WrOToPoint(2));
        data.push_back(0);
        coilspringOpenMBV->append(data);
      }
#endif
      LinkMechanics::plot(t,dt);
    }
  }

  void SpringDamper::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    LinkMechanics::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMNS"forceFunction");
    Function2<double,double,double> *f=ObjectFactory::getInstance()->createFunction2_SSS(e->FirstChildElement());
    setForceFunction(f);
    f->initializeUsingXML(e->FirstChildElement());
    e=element->FirstChildElement(MBSIMNS"projectionDirection");
    if(e) {
      TiXmlElement *ee=e->FirstChildElement(MBSIMNS"frameOfReference");
      Frame *ref=getFrameByPath(ee->Attribute("ref"));
      if(!ref) { cerr<<"ERROR! Cannot find frame: "<<ee->Attribute("ref")<<endl; _exit(1); }
      ee=e->FirstChildElement(MBSIMNS"direction");
      Vec dir(ee->GetText());
      setProjectionDirection(ref, dir);
    }
    e=element->FirstChildElement(MBSIMNS"connect");
    Frame *ref1=getFrameByPath(e->Attribute("ref1"));
    if(!ref1) { cerr<<"ERROR! Cannot find frame: "<<e->Attribute("ref1")<<endl; _exit(1); }
    Frame *ref2=getFrameByPath(e->Attribute("ref2"));
    if(!ref2) { cerr<<"ERROR! Cannot find frame: "<<e->Attribute("ref2")<<endl; _exit(1); }
    connect(ref1,ref2);
    e=e->NextSiblingElement();
#ifdef HAVE_OPENMBVCPPINTERFACE
    OpenMBV::CoilSpring *coilSpring=dynamic_cast<OpenMBV::CoilSpring*>(OpenMBV::ObjectFactory::createObject(e));
    if(coilSpring) {
      setOpenMBVSpring(coilSpring);
      coilSpring->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }
    OpenMBV::Arrow *arrow=dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e));
    if(arrow) {
      arrow->initializeUsingXML(e); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
      setOpenMBVForceArrow(arrow);
      e=e->NextSiblingElement();
    }
#endif
  }

}
