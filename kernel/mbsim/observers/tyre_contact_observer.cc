/* Copyright (C) 2004-2022 MBSim Development Team
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
#include "tyre_contact_observer.h"
#include "mbsim/links/tyre_contact.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, TyreContactObserver)

  TyreContactObserver::TyreContactObserver(const std::string &name) : MechanicalLinkObserver(name), openMBVContactFrame(2) {
  }

  void TyreContactObserver::init(InitStage stage, const InitConfigSet &config) {
   if(stage==plotting) {
     if(plotFeature[plotRecursive]) {
       if(plotFeature[position]) {
	 for(int i=0; i<2; i++)
	   addToPlot("position "+to_string(convertIndex(i)),{"x","y","z"});
       }
       if(plotFeature[angle]) {
	 for(int i=0; i<2; i++)
	   addToPlot("angle "+to_string(convertIndex(i)),{"x","y","z"});
       }
       if(plotFeature[velocity]) {
	 for(int i=0; i<2; i++)
	   addToPlot("velocity "+to_string(convertIndex(i)),{"x","y","z"});
       }
       if(plotFeature[angularVelocity]) {
	 for(int i=0; i<2; i++)
	   addToPlot("angular velocity "+to_string(convertIndex(i)),{"x","y","z"});
       }
       if(plotFeature[acceleration]) {
	 for(int i=0; i<2; i++)
	   addToPlot("acceleration "+to_string(convertIndex(i)),{"x","y","z"});
       }
       if(plotFeature[angularAcceleration]) {
	 for(int i=0; i<2; i++)
	   addToPlot("angular acceleration "+to_string(convertIndex(i)),{"x","y","z"});
       }
     }
     MechanicalLinkObserver::init(stage, config);
     if (plotFeature[openMBV]) {
       if(openMBVContactFrame[0]) {
	 for (unsigned int i = 0; i < 2; i++) { // frames
	   openMBVContactFrame[i]->setName(i == 0 ? "Frame_A" : "Frame_B");
	   getOpenMBVGrp()->addObject(openMBVContactFrame[i]);
	 }
       }
       // arrows
       if(ombvNormalForce) {
	 normalForceArrow.resize(ombvNormalForce->getSideOfInteraction()==2?2:1);
	 for(size_t i=0; i<normalForceArrow.size(); i++) {
	   normalForceArrow[i]=ombvNormalForce->createOpenMBV();
	   normalForceArrow[i]->setName(string("NormalForce_")+(normalForceArrow.size()>1?to_string(i):string("B")));
	   getOpenMBVGrp()->addObject(normalForceArrow[i]);
	 }
       }
       if(ombvLongitudinalForce) {
	 longitudinalForceArrow.resize(ombvLongitudinalForce->getSideOfInteraction()==2?2:1);
	 for(size_t i=0; i<longitudinalForceArrow.size(); i++) {
	   longitudinalForceArrow[i]=ombvLongitudinalForce->createOpenMBV();
	   longitudinalForceArrow[i]->setName(string("LongitudinalForce_")+(longitudinalForceArrow.size()>1?to_string(i):string("B")));
	   getOpenMBVGrp()->addObject(longitudinalForceArrow[i]);
	 }
       }
       if(ombvLateralForce) {
	 lateralForceArrow.resize(ombvLateralForce->getSideOfInteraction()==2?2:1);
	 for(size_t i=0; i<lateralForceArrow.size(); i++) {
	   lateralForceArrow[i]=ombvLateralForce->createOpenMBV();
	   lateralForceArrow[i]->setName(string("LateralForce_")+(lateralForceArrow.size()>1?to_string(i):string("B")));
	   getOpenMBVGrp()->addObject(lateralForceArrow[i]);
	 }
       }
       if(ombvAligningMoment) {
	 aligningMomentArrow.resize(ombvAligningMoment->getSideOfInteraction()==2?2:1);
	 for(size_t i=0; i<aligningMomentArrow.size(); i++) {
	   aligningMomentArrow[i]=ombvAligningMoment->createOpenMBV();
	   aligningMomentArrow[i]->setName(string("AligningMoment_")+(aligningMomentArrow.size()>1?to_string(i):string("B")));
	   getOpenMBVGrp()->addObject(aligningMomentArrow[i]);
	 }
       }
     }
   }
   else
     MechanicalLinkObserver::init(stage, config);
  }

  void TyreContactObserver::plot() {
    if(plotFeature[plotRecursive]) {
      if(plotFeature[position]) {
	for(int i=0; i<2; i++)
	  Element::plot(static_cast<TyreContact*>(link)->getContourFrame(i)->evalPosition());
      }
      if(plotFeature[angle]) {
	for(int i=0; i<2; i++)
	  Element::plot(AIK2Cardan(static_cast<TyreContact*>(link)->getContourFrame(i)->evalOrientation()));
      }
      if(plotFeature[velocity]) {
	for(int i=0; i<2; i++)
	  Element::plot(static_cast<TyreContact*>(link)->getContourFrame(i)->evalVelocity());
      }
      if(plotFeature[angularVelocity]) {
	for(int i=0; i<2; i++)
	  Element::plot(static_cast<TyreContact*>(link)->getContourFrame(i)->evalAngularVelocity());
      }
      if(plotFeature[acceleration]) {
	for(int i=0; i<2; i++)
	  Element::plot(static_cast<TyreContact*>(link)->getContourFrame(i)->evalAcceleration());
      }
      if(plotFeature[angularAcceleration]) {
	for(int i=0; i<2; i++)
	  Element::plot(static_cast<TyreContact*>(link)->getContourFrame(i)->evalAngularAcceleration());
      }
    }
    if(plotFeature[openMBV]) {
      if(openMBVContactFrame[0]) {
	for(unsigned int i = 0; i < 2; i++) {
	  vector<double> data;
	  data.push_back(getTime());
	  Vec3 pos = static_cast<TyreContact*>(link)->getContourFrame(i)->evalPosition();
	  data.push_back(pos(0));
	  data.push_back(pos(1));
	  data.push_back(pos(2));
	  Vec3 cardan = AIK2Cardan(static_cast<TyreContact*>(link)->getContourFrame(i)->evalOrientation());
	  data.push_back(cardan(0));
	  data.push_back(cardan(1));
	  data.push_back(cardan(2));
	  data.push_back(1);
	  openMBVContactFrame[i]->append(data);
	}
      }
      // arrows
      if(ombvNormalForce) {
	int off = ombvNormalForce->getSideOfInteraction()==0?1:0;
	for(size_t i=0; i<normalForceArrow.size(); i++) {
	  vector<double> data;
	  data.push_back(getTime());
	  Vec3 toPoint = static_cast<TyreContact*>(link)->getContourFrame(off+i)->evalPosition();
	  data.push_back(toPoint(0));
	  data.push_back(toPoint(1));
	  data.push_back(toPoint(2));
	  Vec3 F = ((off+i)==1?1.:-1.)*static_cast<TyreContact*>(link)->evalGlobalForceDirection().col(2)*static_cast<TyreContact*>(link)->evalGeneralizedForce()(2);
	  data.push_back(F(0));
	  data.push_back(F(1));
	  data.push_back(F(2));
	  data.push_back(1);
	  normalForceArrow[i]->append(data);
	}
      }
      if(ombvLongitudinalForce) {
	int off = ombvLongitudinalForce->getSideOfInteraction()==0?1:0;
	for(size_t i=0; i<longitudinalForceArrow.size(); i++) {
	  vector<double> data;
	  data.push_back(getTime());
	  Vec3 toPoint = static_cast<TyreContact*>(link)->getContourFrame(off+i)->evalPosition();
	  data.push_back(toPoint(0));
	  data.push_back(toPoint(1));
	  data.push_back(toPoint(2));
	  Vec3 F = ((off+i)==1?1.:-1.)*static_cast<TyreContact*>(link)->evalGlobalForceDirection().col(0)*static_cast<TyreContact*>(link)->evalGeneralizedForce()(0);
	  data.push_back(F(0));
	  data.push_back(F(1));
	  data.push_back(F(2));
	  data.push_back(1);
	  longitudinalForceArrow[i]->append(data);
	}
      }
      if(ombvLateralForce) {
	int off = ombvLateralForce->getSideOfInteraction()==0?1:0;
	for(size_t i=0; i<lateralForceArrow.size(); i++) {
	  vector<double> data;
	  data.push_back(getTime());
	  Vec3 toPoint = static_cast<TyreContact*>(link)->getContourFrame(off+i)->evalPosition();
	  data.push_back(toPoint(0));
	  data.push_back(toPoint(1));
	  data.push_back(toPoint(2));
	  Vec3 F = ((off+i)==1?1.:-1.)*static_cast<TyreContact*>(link)->evalGlobalForceDirection().col(1)*static_cast<TyreContact*>(link)->evalGeneralizedForce()(1);
	  data.push_back(F(0));
	  data.push_back(F(1));
	  data.push_back(F(2));
	  data.push_back(1);
	  lateralForceArrow[i]->append(data);
	}
      }
      if(ombvAligningMoment) {
	int off = ombvAligningMoment->getSideOfInteraction()==0?1:0;
	for(size_t i=0; i<aligningMomentArrow.size(); i++) {
	  vector<double> data;
	  data.push_back(getTime());
	  Vec3 toPoint = static_cast<TyreContact*>(link)->getContourFrame(off+i)->evalPosition();
	  data.push_back(toPoint(0));
	  data.push_back(toPoint(1));
	  data.push_back(toPoint(2));
	  Vec3 M = ((off+i)==1?1.:-1.)*static_cast<TyreContact*>(link)->evalGlobalForceDirection().col(2)*static_cast<TyreContact*>(link)->evalGeneralizedForce()(3);
	  data.push_back(M(0));
	  data.push_back(M(1));
	  data.push_back(M(2));
	  data.push_back(1);
	  aligningMomentArrow[i]->append(data);
	}
      }
    }
    MechanicalLinkObserver::plot();
  }

  void TyreContactObserver::initializeUsingXML(DOMElement *element) {
    MechanicalLinkObserver::initializeUsingXML(element);
    DOMElement *e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVContactPoints");
    if (e) {
      OpenMBVFrame ombv;
      ombv.initializeUsingXML(e);
      openMBVContactFrame[0]=ombv.createOpenMBV();
      openMBVContactFrame[1]=OpenMBV::ObjectFactory::create(openMBVContactFrame[0]);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVNormalForce");
    if (e) {
      ombvNormalForce = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvNormalForce->initializeUsingXML(e);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVLongitudinalForce");
    if (e) {
      ombvLongitudinalForce = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvLongitudinalForce->initializeUsingXML(e);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVLateralForce");
    if (e) {
      ombvLateralForce = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvLateralForce->initializeUsingXML(e);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAligningMoment");
    if (e) {
      ombvAligningMoment = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toDoubleHead,OpenMBVArrow::toPoint));
      ombvAligningMoment->initializeUsingXML(e);
    }
  }

}
