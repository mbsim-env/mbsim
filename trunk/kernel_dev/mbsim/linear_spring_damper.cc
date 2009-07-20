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
#include "mbsim/linear_spring_damper.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/utils/eps.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/objectfactory.h"
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  LinearSpringDamper::LinearSpringDamper(const string &name) : LinkMechanics(name) {}

  void LinearSpringDamper::updateh(double t) {
    la(0) = (cT*(g(0)-l0) + dT*gd(0));
    if (dist>epsroot())
      WF[0] = forceDir*la;
    else
      WF[0] = Vec(3, INIT, 0);
    WF[1] = -WF[0];
    for(unsigned int i=0; i<frame.size(); i++) {
      h[i] += trans(frame[i]->getJacobianOfTranslation())*WF[i];
      hLink[i] += trans(frame[i]->getJacobianOfTranslation())*WF[i];
    }
  }    

  void LinearSpringDamper::updateg(double t) {
    Vec WrP0P1=frame[1]->getPosition() - frame[0]->getPosition();
    dist = nrm2(WrP0P1);
    if (dist>epsroot()) {
      forceDir = WrP0P1/dist;
      g(0) = trans(forceDir)*WrP0P1;
    }
    else
      g(0) = 0;
  } 

  void LinearSpringDamper::updategd(double t) {
    if (dist>epsroot())
      gd(0) = trans(forceDir)*(frame[1]->getVelocity() - frame[0]->getVelocity());  
    else
      gd(0) = 0;
  }    

  void LinearSpringDamper::init() {
    LinkMechanics::init();

    g.resize(1);
    gd.resize(1);
    la.resize(1);
  }

  void LinearSpringDamper::plot(double t,double dt) {
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

  void LinearSpringDamper::initPlot() {
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

  void LinearSpringDamper::connect(Frame *frame0, Frame* frame1) {
    LinkMechanics::connect(frame0);
    LinkMechanics::connect(frame1);
  }

  void LinearSpringDamper::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    LinkMechanics::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMNS"stiffnessCoefficient");
    setStiffnessCoefficient(atof(e->GetText()));
    e=element->FirstChildElement(MBSIMNS"dampingCoefficient");
    setDampingCoefficient(atof(e->GetText()));
    e=element->FirstChildElement(MBSIMNS"unloadedLength");
    setUnloadedLength(atof(e->GetText()));
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
    }
#endif
  }

}

