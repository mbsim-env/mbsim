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
#include "mbsim/observers/single_contact_observer.h"
#include "mbsim/links/single_contact.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/contour.h"
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, SingleContactObserver)

  SingleContactObserver::SingleContactObserver(const std::string &name) : MechanicalLinkObserver(name), openMBVContactFrame(2) {
    evalOMBVNormalForceColorRepresentation[0] = &SingleContactObserver::evalNone;
    evalOMBVNormalForceColorRepresentation[1] = &SingleContactObserver::evalAbsoluteNormalForce;
    evalOMBVTangentialForceColorRepresentation[0] = &SingleContactObserver::evalNone;
    evalOMBVTangentialForceColorRepresentation[1] = &SingleContactObserver::evalAbsoluteTangentialForce;
    evalOMBVTangentialForceColorRepresentation[2] = &SingleContactObserver::evalStickSlip;
  }

  void SingleContactObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==plotting) {
      if(plotFeature[plotRecursive]) {
        if(plotFeature[position]) {
          for(int i=0; i<2; i++)
            addToPlot("position "+to_string(i),{"x","y","z"});
        }
        if(plotFeature[angle]) {
          for(int i=0; i<2; i++)
            addToPlot("angle "+to_string(i),{"x","y","z"});
        }
        if(plotFeature[velocity]) {
          for(int i=0; i<2; i++)
            addToPlot("velocity "+to_string(i),{"x","y","z"});
        }
        if(plotFeature[angularVelocity]) {
          for(int i=0; i<2; i++)
            addToPlot("angular velocity "+to_string(i),{"x","y","z"});
        }
        if(plotFeature[acceleration]) {
          for(int i=0; i<2; i++)
            addToPlot("acceleration "+to_string(i),{"x","y","z"});
        }
        if(plotFeature[angularAcceleration]) {
          for(int i=0; i<2; i++)
            addToPlot("angular acceleration "+to_string(i),{"x","y","z"});
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
        if(ombvContact) {
          contactArrow.resize(ombvContact->getSideOfInteraction()==2?2:1);
          for(size_t i=0; i<contactArrow.size(); i++) {
            contactArrow[i]=ombvContact->createOpenMBV();
            contactArrow[i]->setName(string("NormalForce_")+(contactArrow.size()>1?to_string(i):string("B")));
            getOpenMBVGrp()->addObject(contactArrow[i]);
          }
        }
        if(ombvFriction && static_cast<SingleContact*>(link)->getFrictionDirections() > 0) { // friction force
          frictionArrow.resize(ombvFriction->getSideOfInteraction()==2?2:1);
          for(size_t i=0; i<frictionArrow.size(); i++) {
            frictionArrow[i]=ombvFriction->createOpenMBV();
            frictionArrow[i]->setName(string("TangentialForce_")+(frictionArrow.size()>1?to_string(i):string("B")));
            getOpenMBVGrp()->addObject(frictionArrow[i]);
          }
        }
      }
    }
    else
      MechanicalLinkObserver::init(stage, config);
  }

  void SingleContactObserver::plot() {
    if(plotFeature[plotRecursive]) {
      auto TWOut = outputFrame /*from parent Contact or parent maxwell*/->evalOrientation();
      if(plotFeature[position]) {
        for(int i=0; i<2; i++)
          Element::plot(TWOut.T()*static_cast<SingleContact*>(link)->getContourFrame(i)->evalPosition());
      }
      if(plotFeature[angle]) {
        for(int i=0; i<2; i++)
          Element::plot(AIK2Cardan(static_cast<SingleContact*>(link)->getContourFrame(i)->evalOrientation()));
      }
      if(plotFeature[velocity]) {
        for(int i=0; i<2; i++)
          Element::plot(TWOut.T()*static_cast<SingleContact*>(link)->getContourFrame(i)->evalVelocity());
      }
      if(plotFeature[angularVelocity]) {
        for(int i=0; i<2; i++)
          Element::plot(TWOut.T()*static_cast<SingleContact*>(link)->getContourFrame(i)->evalAngularVelocity());
      }
      if(plotFeature[acceleration]) {
        for(int i=0; i<2; i++)
          Element::plot(TWOut.T()*static_cast<SingleContact*>(link)->getContourFrame(i)->evalAcceleration());
      }
      if(plotFeature[angularAcceleration]) {
        for(int i=0; i<2; i++)
          Element::plot(TWOut.T()*static_cast<SingleContact*>(link)->getContourFrame(i)->evalAngularAcceleration());
      }
    }
    if(plotFeature[openMBV]) {
      if(openMBVContactFrame[0]) {
        for(unsigned int i = 0; i < 2; i++) {
          array<double,8> data;
          data[0] = getTime();
          Vec3 pos = static_cast<SingleContact*>(link)->getContourFrame(i)->evalPosition();
          data[1] = pos(0);
          data[2] = pos(1);
          data[3] = pos(2);
          Vec3 cardan = AIK2Cardan(static_cast<SingleContact*>(link)->getContourFrame(i)->evalOrientation());
          data[4] = cardan(0);
          data[5] = cardan(1);
          data[6] = cardan(2);
          data[7] = 0;
          openMBVContactFrame[i]->append(data);
        }
      }
      // arrows
      if(ombvContact) {
        int off = ombvContact->getSideOfInteraction()==0?1:0;
        for(size_t i=0; i<contactArrow.size(); i++) {
          array<double,8> data;
          data[0] = getTime();
          Vec3 toPoint = static_cast<SingleContact*>(link)->getContourFrame(off+i)->evalPosition();
          data[1] = toPoint(0);
          data[2] = toPoint(1);
          data[3] = toPoint(2);
          Vec3 F = ((off+i)==1?1.:-1.)*static_cast<SingleContact*>(link)->evalGlobalForceDirection().col(0)*static_cast<SingleContact*>(link)->evalGeneralizedNormalForce();
          data[4] = F(0);
          data[5] = F(1);
          data[6] = F(2);
          data[7] = (this->*evalOMBVNormalForceColorRepresentation[ombvContact->getColorRepresentation()])();
          contactArrow[i]->append(data);
        }
      }
      if(ombvFriction) {
        int off = ombvFriction->getSideOfInteraction()==0?1:0;
        for(size_t i=0; i<frictionArrow.size(); i++) {
          array<double,8> data;
          data[0] = getTime();
          Vec3 toPoint = static_cast<SingleContact*>(link)->getContourFrame(off+i)->evalPosition();
          data[1] = toPoint(0);
          data[2] = toPoint(1);
          data[3] = toPoint(2);
          Vec3 F = ((off+i)==1?1.:-1.)*static_cast<SingleContact*>(link)->evalGlobalForceDirection()(RangeV(0,2),RangeV(1, static_cast<SingleContact*>(link)->getFrictionDirections()))*static_cast<SingleContact*>(link)->evalGeneralizedTangentialForce();
          data[4] = F(0);
          data[5] = F(1);
          data[6] = F(2);
          // TODO fdf->isSticking(evalGeneralizedRelativeVelocity()(RangeV(1,getFrictionDirections())), gdTol)
          //        data.push_back(static_cast<SingleContact*>(link)->isSticking() ? 1 : 0.5); // draw in green if slipping and draw in red if sticking
          data[7] = (this->*evalOMBVTangentialForceColorRepresentation[ombvFriction->getColorRepresentation()])();
          frictionArrow[i]->append(data);
        }
      }
    }
    MechanicalLinkObserver::plot();
  }

  double SingleContactObserver::evalAbsoluteNormalForce() {
    return nrm2(static_cast<SingleContact*>(link)->evalGlobalForceDirection().col(0)*static_cast<SingleContact*>(link)->evalGeneralizedNormalForce());
  }

  double SingleContactObserver::evalAbsoluteTangentialForce() {
    return nrm2(static_cast<SingleContact*>(link)->evalGlobalForceDirection()(RangeV(0,2),RangeV(1, static_cast<SingleContact*>(link)->getFrictionDirections()))*static_cast<SingleContact*>(link)->evalGeneralizedTangentialForce());
  }

  double SingleContactObserver::evalStickSlip() {
    return static_cast<SingleContact*>(link)->isSticking() ? 1 : 0.5; // draw in green if slipping and draw in red if sticking
  }
}
