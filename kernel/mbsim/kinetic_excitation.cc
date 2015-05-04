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
 */

#include <config.h>
#include "mbsim/kinetic_excitation.h"
#include "mbsim/objectfactory.h"
#include <fmatvec/function.h>
#include <mbsim/dynamic_system_solver.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(KineticExcitation, MBSIM%"KineticExcitation")

  KineticExcitation::KineticExcitation(const string &name) : MechanicalLink(name), refFrame(NULL), refFrameID(1), F(NULL), M(NULL), C("F") {
    C.setParent(this);
  }

  KineticExcitation::~KineticExcitation() {
    delete F;
    delete M;
  }

  void KineticExcitation::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_ref!="")
        connect(getByPath<Frame>(saved_ref));
      else if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      if(not(frame.size()))
        THROW_MBSIMERROR("no connection given!");
      if(frame.size()==1) {
        Frame *buf = frame[0];
        connect(buf);
        frame[0]=ds->getFrame("I");
      }
      MechanicalLink::init(stage);
    }
    else if(stage==preInit) {
      MechanicalLink::init(stage);
      if(F) addDependency(F->getDependency());
      if(M) addDependency(M->getDependency());
    }
    else if(stage==resize) {
      iF = Index(0, forceDir.cols() - 1);
      iM = Index(forceDir.cols(), forceDir.cols() + momentDir.cols() - 1);
      laSV.resize(forceDir.cols() + momentDir.cols());
    }
    else if(stage==unknownStage) {
      MechanicalLink::init(stage);
      if(F) assert((*F)(0).size()==forceDir.cols());
      if(M) assert((*M)(0).size()==momentDir.cols());
      refFrame=refFrameID?frame[1]:frame[0];
      C.setFrameOfReference(frame[0]);
    }
    else if(stage==plotting) {
      updatePlotFeatures();
      MechanicalLink::init(stage);
    }
    else
      MechanicalLink::init(stage);
    if(F) F->init(stage);
    if(M) M->init(stage);
  }

  void KineticExcitation::connect(Frame *frame0, Frame* frame1) {
    MechanicalLink::connect(frame0);
    MechanicalLink::connect(frame1);
  }

  void KineticExcitation::updatePositions(double t) {
    WrP0P1 = frame[1]->getPosition(t) - frame[0]->getPosition(t);
    C.setPosition(frame[1]->getPosition());
    C.setOrientation(frame[0]->getOrientation());
    updPos = false;
  }

  void KineticExcitation::updateForceDirections(double t) {
    DF = refFrame->getOrientation(t)*forceDir;
    DM = refFrame->getOrientation(t)*momentDir;
    updFD = false;
  }

  void KineticExcitation::updateGeneralizedSingleValuedForces(double t) {
    if(F) laSV.set(iF,(*F)(t));
    if(M) laSV.set(iM,(*M)(t));
    updlaSV = false;
  }

  void KineticExcitation::updateh(double t, int j) {
    h[j][0]-=C.getJacobianOfTranslation(t,j).T()*getSingleValuedForce(t) + C.getJacobianOfRotation(t,j).T()*getSingleValuedMoment(t);
    h[j][1]+=frame[1]->getJacobianOfTranslation(t,j).T()*getSingleValuedForce(t) + frame[1]->getJacobianOfRotation(t,j).T()*getSingleValuedMoment(t);
  }

  void KineticExcitation::calclaSize(int j) {
    MechanicalLink::calclaSize(j);
    laSize=forceDir.cols()+momentDir.cols();
  }

  void KineticExcitation::setForceDirection(const Mat3xV &fd) {

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.set(i, forceDir.col(i)/nrm2(fd.col(i)));
  }

  void KineticExcitation::setMomentDirection(const Mat3xV &md) {

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.set(i, momentDir.col(i)/nrm2(md.col(i)));
  }

  void KineticExcitation::setForceFunction(Function<VecV(double)> *func) {
    F=func;
    F->setParent(this);
    F->setName("Force");
  }

  void KineticExcitation::setMomentFunction(Function<VecV(double)> *func) {
    M=func;
    M->setParent(this);
    M->setName("Moment");
  }

  void KineticExcitation::initializeUsingXML(DOMElement *element) {
    MechanicalLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReferenceID");
    if(e) refFrameID=getInt(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    if(e) setForceDirection(getMat(e,3,0));
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    if(e) setForceFunction(ObjectFactory::createAndInit<Function<VecV(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"momentDirection");
    if(e) setMomentDirection(getMat(e,3,0));
    e=E(element)->getFirstElementChildNamed(MBSIM%"momentFunction");
    if(e) setMomentFunction(ObjectFactory::createAndInit<Function<VecV(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"connect");
    if(E(e)->hasAttribute("ref"))
      saved_ref=E(e)->getAttribute("ref");
    else {
      saved_ref1=E(e)->getAttribute("ref1");
      saved_ref2=E(e)->getAttribute("ref2");
    }
#ifdef HAVE_OPENMBVCPPINTERFACE
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      setOpenMBVForce(ombv.createOpenMBV(e));
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      setOpenMBVMoment(ombv.createOpenMBV(e));
    }
#endif
  }

  DOMElement* KineticExcitation::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = MechanicalLink::writeXMLFile(parent);
//    if(refFrame) {
//      DOMElement *ele1 = new DOMElement( MBSIM%"frameOfReference" );
//      ele1->SetAttribute("ref", refFrame->getXMLPath(this,true)); // relative path
//      ele0->LinkEndChild(ele1);
//    }
//    if(forceDir.cols()) {
//      addElementText(ele0,MBSIM%"forceDirection",forceDir);
//      DOMElement *ele1 = new DOMElement(MBSIM%"forceFunction");
//      F->writeXMLFile(ele1);
//      ele0->LinkEndChild(ele1);
//    }
//    if(momentDir.cols()) {
//      addElementText(ele0,MBSIM%"momentDirection",momentDir);
//      DOMElement *ele1 = new DOMElement(MBSIM%"momentFunction");
//      M->writeXMLFile(ele1);
//      ele0->LinkEndChild(ele1);
//    }
//    DOMElement *ele1 = new DOMElement(MBSIM%"connect");
//    ele1->SetAttribute("ref1", frame[0]->getXMLPath(this,true)); // relative path
//    ele1->SetAttribute("ref2", frame[1]->getXMLPath(this,true)); // relative path
//    ele0->LinkEndChild(ele1);
    return ele0;
  }

}

