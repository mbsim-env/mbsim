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
#include "mbsim/observers/cylinder_coordinates_observer.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/utils/eps.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/group.h>
#endif

using namespace std;
using namespace xercesc;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, CylinderCoordinatesObserver)

  CylinderCoordinatesObserver::CylinderCoordinatesObserver(const std::string &name) : CoordinatesObserver(name) {
    ez(2) = 1;
  }

  void CylinderCoordinatesObserver::init(InitStage stage) {
    if(stage==plotting) {
      updatePlotFeatures();

      if(openMBVPosition) {
        plotColumns.push_back("RadialPosition");
        plotColumns.push_back("ZPosition");
      }
      if(openMBVVelocity) {
        plotColumns.push_back("RadialVelocity");
        plotColumns.push_back("CircularVelocity");
        plotColumns.push_back("ZVelocity");
      }
      if(openMBVAcceleration) {
        plotColumns.push_back("RadialAcceleration");
        plotColumns.push_back("CircularAcceleration");
        plotColumns.push_back("ZAcceleration");
      }
      CoordinatesObserver::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVPosition) {
            openMBVRadialPosition = OpenMBV::ObjectFactory::create(openMBVPosition);
            openMBVZPosition = OpenMBV::ObjectFactory::create(openMBVPosition);
            openMBVRadialPosition->setName("RadialPosition");
            openMBVPosGrp->addObject(openMBVRadialPosition);
            openMBVZPosition->setName("ZPosition");
            openMBVPosGrp->addObject(openMBVZPosition);
          }
          if(openMBVVelocity) {
            openMBVRadialVelocity = OpenMBV::ObjectFactory::create(openMBVVelocity);
            openMBVCircularVelocity = OpenMBV::ObjectFactory::create(openMBVVelocity);
            openMBVZVelocity = OpenMBV::ObjectFactory::create(openMBVVelocity);
            openMBVRadialVelocity->setName("RadialVelocity");
            openMBVVelGrp->addObject(openMBVRadialVelocity);
            openMBVCircularVelocity->setName("CircularVelocity");
            openMBVVelGrp->addObject(openMBVCircularVelocity);
            openMBVZVelocity->setName("ZVelocity");
            openMBVVelGrp->addObject(openMBVZVelocity);
          }
          if(openMBVAcceleration) {
            openMBVRadialAcceleration = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVCircularAcceleration = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVZAcceleration = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVRadialAcceleration->setName("RadialAcceleration");
            openMBVAccGrp->addObject(openMBVRadialAcceleration);
            openMBVCircularAcceleration->setName("CircularAcceleration");
            openMBVAccGrp->addObject(openMBVCircularAcceleration);
            openMBVZAcceleration->setName("ZAcceleration");
            openMBVAccGrp->addObject(openMBVZAcceleration);
          }
        }
#endif
      }
    }
    else
      CoordinatesObserver::init(stage);
  }

  void CylinderCoordinatesObserver::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        Vec3 r = frame->evalPosition();
        Vec3 v = frame->evalVelocity();
        Vec3 a = frame->evalAcceleration();
        Vec3 ep = crossProduct(ez,r);
        double nrmep = nrm2(ep);
        if(nrmep<epsroot()) {
          if(fabs(ez(0))<epsroot() && fabs(ez(1))<epsroot()) {
            ep(0) = 1.;
            ep(1) = 0.;
            ep(2) = 0.;
          }
          else {
            ep(0) = -ez(1);
            ep(1) = ez(0);
            ep(2) = 0.0;
          }
        }
        else
          ep = ep/nrmep;
        Vec3 er = crossProduct(ep,ez);

        if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          Vec3 rr =  (r.T()*er)*er;
          data.push_back(getTime());
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(rr(0));
          data.push_back(rr(1));
          data.push_back(rr(2));
          data.push_back(0.5);
          openMBVRadialPosition->append(data);
          data.clear();
          Vec3 rz =  (r.T()*ez)*ez;
          data.push_back(getTime());
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(rz(0));
          data.push_back(rz(1));
          data.push_back(rz(2));
          data.push_back(0.5);
          openMBVZPosition->append(data);
          plotVector.push_back(r.T()*er);
          plotVector.push_back(r(2));
        }

        if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
          vector<double> data;
          Vec3 vr =  (v.T()*er)*er;
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vr(0));
          data.push_back(vr(1));
          data.push_back(vr(2));
          data.push_back(0.5);
          openMBVRadialVelocity->append(data);
          data.clear();
          Vec3 vp =  (v.T()*ep)*ep;
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vp(0));
          data.push_back(vp(1));
          data.push_back(vp(2));
          data.push_back(0.5);
          openMBVCircularVelocity->append(data);
          data.clear();
          Vec3 vz =  (v.T()*ez)*ez;
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vz(0));
          data.push_back(vz(1));
          data.push_back(vz(2));
          data.push_back(0.5);
          openMBVZVelocity->append(data);
          plotVector.push_back(v.T()*er);
          plotVector.push_back(v.T()*ep);
          plotVector.push_back(v(2));
        }

        if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec3 ar =  (a.T()*er)*er;
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(ar(0));
          data.push_back(ar(1));
          data.push_back(ar(2));
          data.push_back(0.5);
          openMBVRadialAcceleration->append(data);
          data.clear();
          Vec3 ap =  (a.T()*ep)*ep;
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(ap(0));
          data.push_back(ap(1));
          data.push_back(ap(2));
          data.push_back(0.5);
          openMBVCircularAcceleration->append(data);
          data.clear();
          Vec3 az =  (a.T()*ez)*ez;
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(az(0));
          data.push_back(az(1));
          data.push_back(az(2));
          data.push_back(0.5);
          openMBVZAcceleration->append(data);
          plotVector.push_back(a.T()*er);
          plotVector.push_back(a.T()*ep);
          plotVector.push_back(a(2));
        }

        if(openMBVFrame && !openMBVFrame->isHDF5Link()) {
          vector<double> data;
          SqrMat3 AWP;
          AWP.set(0, er);
          AWP.set(1, ep);
          AWP.set(2, ez);
          data.push_back(getTime());
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          Vec3 cardan=AIK2Cardan(AWP);
          data.push_back(cardan(0));
          data.push_back(cardan(1));
          data.push_back(cardan(2));
          data.push_back(0);
          openMBVFrame->append(data);
        }
      }
#endif

      CoordinatesObserver::plot();
    }
  }

}
