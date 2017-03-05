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
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, SingleContactObserver)

  SingleContactObserver::SingleContactObserver(const std::string &name) : MechanicalLinkObserver(name), openMBVContactFrame(2) {
  }

  void SingleContactObserver::init(InitStage stage) {
    if(stage==plotting) {
      MechanicalLinkObserver::init(stage);
      updatePlotFeatures();
      if (getPlotFeature(plotRecursive) == enabled) {
        if(openMBVContactFrame[0]) {
          for (unsigned int i = 0; i < 2; i++) { // frames
            openMBVContactFrame[i]->setName(i == 0 ? "Frame_A" : "Frame_B");
            getOpenMBVGrp()->addObject(openMBVContactFrame[i]);
          }
        }
        // arrows
        if(contactArrow) {
          contactArrow->setName("NormalForce_B");
          getOpenMBVGrp()->addObject(contactArrow);
        }
        if(frictionArrow && static_cast<SingleContact*>(link)->getFrictionDirections() > 0) { // friction force
          frictionArrow->setName("FrictionForce_B");
          getOpenMBVGrp()->addObject(frictionArrow);
        }
      }
    }
    else
      MechanicalLinkObserver::init(stage);
  }

  void SingleContactObserver::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVContactFrame[0]) {
          for(unsigned int i = 0; i < 2; i++) {
            vector<double> data;
            data.push_back(getTime());
            Vec3 toPoint = static_cast<SingleContact*>(link)->getContourFrame(i)->evalPosition();
            data.push_back(toPoint(0));
            data.push_back(toPoint(1));
            data.push_back(toPoint(2));
            Vec3 cardan = AIK2Cardan(static_cast<SingleContact*>(link)->getContourFrame(i)->evalOrientation());
            data.push_back(cardan(0));
            data.push_back(cardan(1));
            data.push_back(cardan(2));
            data.push_back(0);
            openMBVContactFrame[i]->append(data);
          }
        }
        // arrows
        vector<double> data;
        if (contactArrow) {
          data.push_back(getTime());
          Vec3 toPoint = static_cast<SingleContact*>(link)->getContourFrame(1)->evalPosition();
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          Vec3 F = static_cast<SingleContact*>(link)->evalGlobalForceDirection().col(0)*static_cast<SingleContact*>(link)->evalGeneralizedNormalForce();
          data.push_back(F(0));
          data.push_back(F(1));
          data.push_back(F(2));
          data.push_back(nrm2(F));
          contactArrow->append(data);
        }
        if (frictionArrow && static_cast<SingleContact*>(link)->getFrictionDirections() > 0) { // friction force
          data.clear();
          data.push_back(getTime());
          Vec3 toPoint = static_cast<SingleContact*>(link)->getContourFrame(1)->evalPosition();
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          Vec3 F = static_cast<SingleContact*>(link)->evalGlobalForceDirection()(RangeV(0,2),RangeV(1, static_cast<SingleContact*>(link)->getFrictionDirections()))*static_cast<SingleContact*>(link)->evalGeneralizedTangentialForce();
          data.push_back(F(0));
          data.push_back(F(1));
          data.push_back(F(2));
          // TODO fdf->isSticking(evalGeneralizedRelativeVelocity()(RangeV(1,getFrictionDirections())), gdTol)
          data.push_back(static_cast<SingleContact*>(link)->isSticking() ? 1 : 0.5); // draw in green if slipping and draw in red if sticking
          frictionArrow->append(data);
        }
      }
      MechanicalLinkObserver::plot();
    }
  }

  void SingleContactObserver::initializeUsingXML(DOMElement *element) {
    MechanicalLinkObserver::initializeUsingXML(element);
    DOMElement *e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVContactPoints");
    if (e) {
      OpenMBVFrame ombv;
      openMBVContactFrame[0] = ombv.createOpenMBV(e); 
      openMBVContactFrame[1]=OpenMBV::ObjectFactory::create(openMBVContactFrame[0]);
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVNormalForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]", 0, OpenMBV::Arrow::toHead, OpenMBV::Arrow::toPoint, 1, 1);
      contactArrow = ombv.createOpenMBV(e);
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVTangentialForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]", 0, OpenMBV::Arrow::toHead, OpenMBV::Arrow::toPoint, 1, 1);
      frictionArrow = ombv.createOpenMBV(e);
    }
  }

}
