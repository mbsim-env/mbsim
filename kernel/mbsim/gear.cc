/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#include <config.h>
#include "mbsim/gear.h"
#include "mbsim/fixed_relative_frame.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/constraint.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Gear, MBSIM%"Gear")

  Gear::Gear(const string &name) : MechanicalLink(name), func(0) {
    body.push_back(0); 
    ratio.push_back(-1);

#ifdef HAVE_OPENMBVCPPINTERFACE
    FArrow.resize(1);
    MArrow.resize(1);
#endif
  }

  void Gear::calclaSize(int j) {
    laSize = 1;
  }
  void Gear::calcgSize(int j) {
    gSize = 1;
  }
  void Gear::calcgdSize(int j) {
    gdSize = 1;
  }

  void Gear::addTransmission(const Transmission &transmission) { 
    body.push_back(transmission.body); 
    ratio.push_back(transmission.ratio); 
  }

  void Gear::updatehRef(const Vec &hParent, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Index I = Index(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      h[j][i]>>hParent(I);
      I = Index(body[i]->getFrameOfReference()->gethInd(j),body[i]->getFrameOfReference()->gethInd(j)+body[i]->getFrameOfReference()->getJacobianOfTranslation(j).cols()-1);
      h[j][body.size()+i]>>hParent(I);
    }
  } 

  void Gear::updateWRef(const Mat &WParent, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(body[i]->getFrameForKinematics()->gethInd(j),body[i]->getFrameForKinematics()->gethInd(j)+body[i]->getFrameForKinematics()->getJacobianOfTranslation(j).cols()-1);

      W[j][i]>>WParent(I,J);
      I = Index(body[i]->getFrameOfReference()->gethInd(j),body[i]->getFrameOfReference()->gethInd(j)+body[i]->getFrameOfReference()->getJacobianOfTranslation(j).cols()-1);
      W[j][body.size()+i]>>WParent(I,J);
    }
  } 

  void Gear::updatePositions(double t) {
    rrel.init(0);
    for(unsigned i=0; i<body.size(); i++) {
      WrP0P1 = body[i]->getFrameForKinematics()->getPosition(t)-body[i]->getFrameOfReference()->getPosition(t);
      C[i].setGlobalRelativePosition(WrP0P1);
      rrel+=body[i]->getqRel(t)*ratio[i];
    }
    updPos = false;
  }

  void Gear::updateVelocities(double t) {
    vrel.init(0);
    for(unsigned i=0; i<body.size(); i++) {
      vrel+=body[i]->getuRel(t)*ratio[i];
    }
    updVel = false;
  }

  void Gear::updateGeneralizedSetValuedForces(double t) {
    laMV = la;
    updlaMV = false;
  }

  void Gear::updateGeneralizedSingleValuedForces(double t) {
    laSV(0) = (*func)(getRelativePosition(t)(0),getRelativeVelocity(t)(0));
    updlaSV = false;
  }

  void Gear::updateh(double t, int j) {
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++)  {
        h[j][i]-=body[i]->getJRel(t,j).T()*ratio[i]*getSingleValuedForce(t);
      }
    } else {
      for(unsigned i=0; i<body.size(); i++) {
        Vec3 WF = body[i]->getFrameOfReference()->getOrientation(t)*body[i]->getPJT(t)*ratio[i]*getSingleValuedGeneralizedForce(t);
        Vec3 WM = body[i]->getFrameOfReference()->getOrientation(t)*body[i]->getPJR(t)*ratio[i]*getSingleValuedGeneralizedForce(t);
        h[j][i]-=body[i]->getFrameForKinematics()->getJacobianOfTranslation(t,j).T()*WF + body[i]->getFrameForKinematics()->getJacobianOfRotation(t,j).T()*WM;
        h[j][body.size()+i]+=C[i].getJacobianOfTranslation(t,j).T()*WF + C[i].getJacobianOfRotation(t,j).T()*WM;
      }
    }
  }

  void Gear::updateW(double t, int j) {
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++)  {
        W[j][i]-=body[i]->getJRel(t,j).T()*ratio[i];
      }
    } else {
      for(unsigned i=0; i<body.size(); i++) {
        Mat3xV WF = body[i]->getFrameOfReference()->getOrientation(t)*body[i]->getPJT(t)*ratio[i];
        Mat3xV WM = body[i]->getFrameOfReference()->getOrientation(t)*body[i]->getPJR(t)*ratio[i];
        W[j][i]-=body[i]->getFrameForKinematics()->getJacobianOfTranslation(t,j).T()*WF + body[i]->getFrameForKinematics()->getJacobianOfRotation(t,j).T()*WM;
        W[j][body.size()+i]+=C[i].getJacobianOfTranslation(t,j).T()*WF + C[i].getJacobianOfRotation(t,j).T()*WM;
      }
    }
  }

  void Gear::updateg(double) {
    g = rrel;
  }

  void Gear::updategd(double) {
    gd = vrel;
  }

  bool Gear::isSetValued() const {
    return func?false:true;
  }

  void Gear::updatewb(double t) {
    for(unsigned i=0; i<body.size(); i++)
      wb += body[i]->getjRel(t)*ratio[i];
  }

 void Gear::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if (saved_DependentBody!="")
        setDependentBody(getByPath<RigidBody>(saved_DependentBody));
      if (saved_IndependentBody.size()>0) {
        for (unsigned int i=0; i<saved_IndependentBody.size(); i++)
          body.push_back(getByPath<RigidBody>(saved_IndependentBody[i]));
      }
      MechanicalLink::init(stage);
    }
    else if(stage==unknownStage) {

      for(unsigned int i=0; i<body.size(); i++) {
        h[0].push_back(Vec(body[i]->getFrameForKinematics()->getJacobianOfTranslation(0).cols()));
        h[1].push_back(Vec(6));
        W[0].push_back(Mat(body[i]->getFrameForKinematics()->getJacobianOfTranslation(0).cols(),laSize));
        W[1].push_back(Mat(6,laSize));
        C.push_back(FixedRelativeFrame("F"));
        C[i].setParent(this);
        C[i].setUpdateGlobalRelativePositionByParent();
      }
      for(unsigned int i=0; i<body.size(); i++) {
        h[0].push_back(Vec(body[i]->getFrameOfReference()->getJacobianOfTranslation(0).cols()));
        h[1].push_back(Vec(6));
        W[0].push_back(Mat(body[i]->getFrameOfReference()->getJacobianOfTranslation(0).cols(),laSize));
        W[1].push_back(Mat(6,laSize));
      }
    }
    else if(stage==preInit) {
      MechanicalLink::init(stage);
      for(unsigned int i=0; i<body.size(); i++)
      if(func) addDependency(func->getDependency());
    }
    else if(stage==resize) {
      MechanicalLink::init(stage);
      iF = Index(0, 0);
      iM = Index(0, -1);
      rrel.resize(1);
      vrel.resize(1);
      if(isSetValued()) {
        g.resize(1);
        gd.resize(1);
        la.resize(1);
        laMV.resize(1);
      }
      else
        laSV.resize(1);
    }
    else if(stage==plotting) {
      updatePlotFeatures();
      for(unsigned int i=0; i<body.size(); i++) {
        plotColumns.push_back("M");
      }
      if(getPlotFeature(plotRecursive)==enabled) {
        MechanicalLink::init(stage);
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(FArrow[0]) {
            FArrow[0]->setName("Force0");
            openMBVForceGrp->addObject(FArrow[0]);
            for(unsigned int i=1; i<body.size(); i++) {
              stringstream s;
              s << i;
              FArrow.push_back(OpenMBV::ObjectFactory::create(FArrow[0]));
              FArrow[i]->setName("Force"+s.str());
              openMBVForceGrp->addObject(FArrow[i]);
            }
          }
          if(MArrow[0]) {
            MArrow[0]->setName("Moment0");
            openMBVForceGrp->addObject(MArrow[0]);
            for(unsigned int i=1; i<body.size(); i++) {
              stringstream s;
              s << i;
              MArrow.push_back(OpenMBV::ObjectFactory::create(MArrow[0]));
              MArrow[i]->setName("Moment"+s.str());
              openMBVForceGrp->addObject(MArrow[i]);
            }
          }
        }
#endif
      }
    }
    else {
      MechanicalLink::init(stage);
    }
    if(func) func->init(stage);
  }

  void Gear::plot(double t,double dt) {
    for(unsigned int i=0; i<body.size(); i++) {
      plotVector.push_back(ratio[i]*getGeneralizedForce(t)(0));
    }
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(FArrow[0]) {
          for(unsigned i=0; i<body.size(); i++) {
            vector<double> data;
            data.push_back(t);
            Vec3 WF = -body[i]->getFrameOfReference()->getOrientation(t)*body[i]->getPJT(t)*ratio[i]*getGeneralizedForce(t);
            Vec3 WrOS=body[i]->getFrameC()->getPosition(t);
            data.push_back(WrOS(0));
            data.push_back(WrOS(1));
            data.push_back(WrOS(2));
            data.push_back(WF(0));
            data.push_back(WF(1));
            data.push_back(WF(2));
            data.push_back(1.0);
            FArrow[i]->append(data);
          }
        }
        if(MArrow[0]) {
          for(unsigned i=0; i<body.size(); i++) {
            vector<double> data;
            data.push_back(t);
            Vec3 WM = -body[i]->getFrameOfReference()->getOrientation(t)*body[i]->getPJR(t)*ratio[i]*getGeneralizedForce(t);
            Vec3 WrOS=body[i]->getFrameC()->getPosition(t);
            data.push_back(WrOS(0));
            data.push_back(WrOS(1));
            data.push_back(WrOS(2));
            data.push_back(WM(0));
            data.push_back(WM(1));
            data.push_back(WM(2));
            data.push_back(1.0);
            MArrow[i]->append(data);
          }
        }
      }
#endif
      MechanicalLink::plot(t,dt);
    }
  }

  void Gear::initializeUsingXML(DOMElement* element) {
    MechanicalLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceFunction");
    if(e) {
      Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
      setGeneralizedForceFunction(f);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"dependentRigidBody");
    saved_DependentBody=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"transmissions");
    DOMElement *ee=e->getFirstElementChild();
    while(ee && E(ee)->getTagName()==MBSIM%"Transmission") {
      saved_IndependentBody.push_back(E(E(ee)->getFirstElementChildNamed(MBSIM%"rigidBody"))->getAttribute("ref"));
      ratio.push_back(getDouble(E(ee)->getFirstElementChildNamed(MBSIM%"ratio")));
      ee=ee->getNextElementSibling();
    }
  }

}


