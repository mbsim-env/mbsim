/* Copyright (C) 2004-2016 MBSim Development Team
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
#include "mbsim/observers/mechanical_link_observer.h"
#include "mbsim/links/mechanical_link.h"
#include "mbsim/frames/frame.h"
#include "mbsim/dynamic_system_solver.h"
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, MechanicalLinkObserver)

  MechanicalLinkObserver::MechanicalLinkObserver(const std::string &name) : Observer(name), link(nullptr) {
    evalOMBVForceColorRepresentation[0] = &MechanicalLinkObserver::evalNone;
    evalOMBVForceColorRepresentation[1] = &MechanicalLinkObserver::evalAbsoluteForce;
    evalOMBVMomentColorRepresentation[0] = &MechanicalLinkObserver::evalNone;
    evalOMBVMomentColorRepresentation[1] = &MechanicalLinkObserver::evalAbsoluteMoment;
  }

  void MechanicalLinkObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not saved_link.empty())
        setMechanicalLink(getByPath<MechanicalLink>(saved_link));
      if(not link)
        throwError("Mechanical link is not given!");
      if(not saved_outputFrame.empty())
        setOutputFrame(getByPath<Frame>(saved_outputFrame));
      if(not outputFrame)
        setOutputFrame(ds->getFrameI());
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
      if(plotFeature[plotRecursive]) {
        if(plotFeature[force]) {
          for(int i=0; i<link->getNumberOfForces(); i++)
            addToPlot("force "+to_string(i),{"x","y","z"});
        }
        if(plotFeature[moment]) {
          for(int i=0; i<link->getNumberOfForces(); i++)
            addToPlot("moment "+to_string(i),{"x","y","z"});
        }
      }
      Observer::init(stage, config);
      if(plotFeature[openMBV]) {
        if(ombvForce) {
          openMBVForce.resize(ombvForce->getSideOfInteraction()==2?link->getNumberOfForces():link->getNumberOfForces()/2);
          for(size_t i=0; i<openMBVForce.size(); i++) {
            openMBVForce[i]=ombvForce->createOpenMBV();
            //openMBVForce[i]->setName(string("Force")+(i<size_t(link->getNumberOfForces()/2)?"R":"A")+to_string(i%size_t(link->getNumberOfForces()/2)));
            openMBVForce[i]->setName(string("Force")+(openMBVForce.size()>1?to_string(i):string("")));
            getOpenMBVGrp()->addObject(openMBVForce[i]);
          }
        }
        if(ombvMoment) {
          openMBVMoment.resize(ombvMoment->getSideOfInteraction()==2?link->getNumberOfForces():link->getNumberOfForces()/2);
          for(size_t i=0; i<openMBVMoment.size(); i++) {
            openMBVMoment[i]=ombvMoment->createOpenMBV();
            //openMBVMoment[i]->setName(string("Moment")+(i<size_t(link->getNumberOfForces()/2)?"R":"A")+to_string(i%size_t(link->getNumberOfForces()/2)));
            openMBVMoment[i]->setName(string("Moment")+(openMBVMoment.size()>1?to_string(i):string("")));
            getOpenMBVGrp()->addObject(openMBVMoment[i]);
          }
        }
      }
    }
    else
      Observer::init(stage, config);
  }

  void MechanicalLinkObserver::plot() {
    if(plotFeature[plotRecursive]) {
      auto TWOut = outputFrame->evalOrientation();
      if(plotFeature[force]) {
        for(int i=0; i<link->getNumberOfForces(); i++)
          Element::plot(TWOut.T()*link->evalForce(i));
      }
      if(plotFeature[moment]) {
        for(int i=0; i<link->getNumberOfForces(); i++)
          Element::plot(TWOut.T()*link->evalMoment(i));
      }
    }
    if(plotFeature[openMBV]) {
      if(ombvForce) {
        int off = ombvForce->getSideOfInteraction()==0?link->getNumberOfForces()/2:0;
        for(size_t i=0; i<openMBVForce.size(); i++) {
          array<OpenMBV::Float,8> data;
          data[0] = getTime();
          Vec3 toPoint=link->getPointOfApplication(off+i)->evalPosition();
          data[1] = toPoint(0);
          data[2] = toPoint(1);
          data[3] = toPoint(2);
          Vec3 WF = link->evalForce(off+i);
          data[4] = WF(0);
          data[5] = WF(1);
          data[6] = WF(2);
          data[7] = (this->*evalOMBVForceColorRepresentation[ombvForce->getColorRepresentation()])();
          openMBVForce[i]->append(data);
        }
      }
      if(ombvMoment) {
        int off = ombvMoment->getSideOfInteraction()==0?link->getNumberOfForces()/2:0;
        for(size_t i=0; i<openMBVMoment.size(); i++) {
          array<OpenMBV::Float,8> data;
          data[0] = getTime();
          Vec3 toPoint=link->getPointOfApplication(off+i)->evalPosition();
          data[1] = toPoint(0);
          data[2] = toPoint(1);
          data[3] = toPoint(2);
          Vec3 WM = link->evalMoment(off+i);
          data[4] = WM(0);
          data[5] = WM(1);
          data[6] = WM(2);
          data[7] = (this->*evalOMBVMomentColorRepresentation[ombvMoment->getColorRepresentation()])();
          openMBVMoment[i]->append(data);
        }
      }
    }
    Observer::plot();
  }

  void MechanicalLinkObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"mechanicalLink");
    saved_link=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"outputFrame");
    if(e) saved_outputFrame=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(e) {
      ombvForce = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvForce->initializeUsingXML(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(e) {
      ombvMoment = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toDoubleHead,OpenMBVArrow::toPoint));
      ombvMoment->initializeUsingXML(e);
    }
  }

  double MechanicalLinkObserver::evalAbsoluteForce() {
    return nrm2(link->evalForce());
  }

  double MechanicalLinkObserver::evalAbsoluteMoment() {
    return nrm2(link->evalMoment());
  }

}
