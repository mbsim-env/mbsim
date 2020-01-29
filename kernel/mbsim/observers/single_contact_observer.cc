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
          for(int i=0; i<2; i++) {
            plotColumns.emplace_back("position"+to_string(i)+" (x)");
            plotColumns.emplace_back("position"+to_string(i)+" (y)");
            plotColumns.emplace_back("position"+to_string(i)+" (z)");
          }
        }
        if(plotFeature[velocity]) {
          for(int i=0; i<2; i++) {
            plotColumns.emplace_back("velocity"+to_string(i)+" (x)");
            plotColumns.emplace_back("velocity"+to_string(i)+" (y)");
            plotColumns.emplace_back("velocity"+to_string(i)+" (z)");
          }
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
      if(plotFeature[position]) {
        for(int i=0; i<2; i++) {
          Vec3 pos = static_cast<SingleContact*>(link)->getContourFrame(i)->evalPosition();
          for(int j=0; j<pos.size(); j++)
            plotVector.push_back(pos(j));
        }
      }
      if(plotFeature[velocity]) {
        for(int i=0; i<2; i++) {
          Vec3 vel = static_cast<SingleContact*>(link)->getContourFrame(i)->evalVelocity();
          for(int j=0; j<vel.size(); j++)
            plotVector.push_back(vel(j));
        }
      }
    }
    if(plotFeature[openMBV]) {
      if(openMBVContactFrame[0]) {
        for(unsigned int i = 0; i < 2; i++) {
          vector<double> data;
          data.push_back(getTime());
          Vec3 pos = static_cast<SingleContact*>(link)->getContourFrame(i)->evalPosition();
          data.push_back(pos(0));
          data.push_back(pos(1));
          data.push_back(pos(2));
          Vec3 cardan = AIK2Cardan(static_cast<SingleContact*>(link)->getContourFrame(i)->evalOrientation());
          data.push_back(cardan(0));
          data.push_back(cardan(1));
          data.push_back(cardan(2));
          data.push_back(0);
          openMBVContactFrame[i]->append(data);
        }
      }
      // arrows
      if(ombvContact) {
        int off = ombvContact->getSideOfInteraction()==0?1:0;
        for(size_t i=0; i<contactArrow.size(); i++) {
          vector<double> data;
          data.push_back(getTime());
          Vec3 toPoint = static_cast<SingleContact*>(link)->getContourFrame(off+i)->evalPosition();
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          Vec3 F = ((off+i)==1?1.:-1.)*static_cast<SingleContact*>(link)->evalGlobalForceDirection().col(0)*static_cast<SingleContact*>(link)->evalGeneralizedNormalForce();
          data.push_back(F(0));
          data.push_back(F(1));
          data.push_back(F(2));
          data.push_back((this->*evalOMBVNormalForceColorRepresentation[ombvContact->getColorRepresentation()])());
          contactArrow[i]->append(data);
        }
      }
      if(ombvFriction) {
        int off = ombvFriction->getSideOfInteraction()==0?1:0;
        for(size_t i=0; i<frictionArrow.size(); i++) {
          vector<double> data;
          data.push_back(getTime());
          Vec3 toPoint = static_cast<SingleContact*>(link)->getContourFrame(off+i)->evalPosition();
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          Vec3 F = ((off+i)==1?1.:-1.)*static_cast<SingleContact*>(link)->evalGlobalForceDirection()(RangeV(0,2),RangeV(1, static_cast<SingleContact*>(link)->getFrictionDirections()))*static_cast<SingleContact*>(link)->evalGeneralizedTangentialForce();
          data.push_back(F(0));
          data.push_back(F(1));
          data.push_back(F(2));
          // TODO fdf->isSticking(evalGeneralizedRelativeVelocity()(RangeV(1,getFrictionDirections())), gdTol)
          //        data.push_back(static_cast<SingleContact*>(link)->isSticking() ? 1 : 0.5); // draw in green if slipping and draw in red if sticking
          data.push_back((this->*evalOMBVTangentialForceColorRepresentation[ombvFriction->getColorRepresentation()])());
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
