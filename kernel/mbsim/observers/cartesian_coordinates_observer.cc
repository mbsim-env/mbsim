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
#include "mbsim/observers/cartesian_coordinates_observer.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/rotarymatrices.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/group.h>
#endif

using namespace std;
using namespace xercesc;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(CartesianCoordinatesObserver, MBSIM%"CartesianCoordinatesObserver")

  CartesianCoordinatesObserver::CartesianCoordinatesObserver(const std::string &name) : CoordinatesObserver(name), A(EYE) {
  }

  void CartesianCoordinatesObserver::init(InitStage stage) {
    if(stage==plotting) {
      updatePlotFeatures();

      if(openMBVPosition) {
        plotColumns.push_back("XPosition");
        plotColumns.push_back("YPosition");
        plotColumns.push_back("ZPosition");
      }
      if(openMBVVelocity) {
        plotColumns.push_back("XVelocity");
        plotColumns.push_back("YVelocity");
        plotColumns.push_back("ZVelocity");
      }
      if(openMBVAcceleration) {
        plotColumns.push_back("XAcceleration");
        plotColumns.push_back("YAcceleration");
        plotColumns.push_back("ZAcceleration");
      }
      CoordinatesObserver::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
	ex = A.col(0);
	ey = A.col(1);
	ez = A.col(2);
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVPosition) {
            openMBVXPosition = OpenMBV::ObjectFactory::create(openMBVPosition);
            openMBVYPosition = OpenMBV::ObjectFactory::create(openMBVPosition);
            openMBVZPosition = OpenMBV::ObjectFactory::create(openMBVPosition);
            openMBVXPosition->setName("XPosition");
            openMBVPosGrp->addObject(openMBVXPosition);
            openMBVYPosition->setName("YPosition");
            openMBVPosGrp->addObject(openMBVYPosition);
            openMBVZPosition->setName("ZPosition");
            openMBVPosGrp->addObject(openMBVZPosition);
          }
          if(openMBVVelocity) {
            openMBVXVelocity = OpenMBV::ObjectFactory::create(openMBVVelocity);
            openMBVYVelocity = OpenMBV::ObjectFactory::create(openMBVVelocity);
            openMBVZVelocity = OpenMBV::ObjectFactory::create(openMBVVelocity);
            openMBVXVelocity->setName("XVelocity");
            openMBVVelGrp->addObject(openMBVXVelocity);
            openMBVYVelocity->setName("YVelocity");
            openMBVVelGrp->addObject(openMBVYVelocity);
            openMBVZVelocity->setName("ZVelocity");
            openMBVVelGrp->addObject(openMBVZVelocity);
          }
          if(openMBVAcceleration) {
            openMBVXAcceleration = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVYAcceleration = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVZAcceleration = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVXAcceleration->setName("XAcceleration");
            openMBVAccGrp->addObject(openMBVXAcceleration);
            openMBVYAcceleration->setName("YAcceleration");
            openMBVAccGrp->addObject(openMBVYAcceleration);
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

  void CartesianCoordinatesObserver::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        Vec3 r = frame->evalPosition();
        Vec3 v = frame->evalVelocity();
        Vec3 a = frame->evalAcceleration();

        if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          Vec3 rx =  (r.T()*ex)*ex;
          data.push_back(getTime());
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(rx(0));
          data.push_back(rx(1));
          data.push_back(rx(2));
          data.push_back(0.5);
          openMBVXPosition->append(data);
          data.clear();
          Vec3 ry =  (r.T()*ey)*ey;
          data.push_back(getTime());
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(ry(0));
          data.push_back(ry(1));
          data.push_back(ry(2));
          data.push_back(0.5);
          openMBVYPosition->append(data);
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
          plotVector.push_back(r(0));
          plotVector.push_back(r(1));
          plotVector.push_back(r(2));
        }

        if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
          vector<double> data;
          Vec3 vx =  (v.T()*ex)*ex;
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vx(0));
          data.push_back(vx(1));
          data.push_back(vx(2));
          data.push_back(0.5);
          openMBVXVelocity->append(data);
          data.clear();
          Vec3 vy =  (v.T()*ey)*ey;
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vy(0));
          data.push_back(vy(1));
          data.push_back(vy(2));
          data.push_back(0.5);
          openMBVYVelocity->append(data);
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
          plotVector.push_back(v(0));
          plotVector.push_back(v(1));
          plotVector.push_back(v(2));
        }

        if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec3 ax =  (a.T()*ex)*ex;
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(ax(0));
          data.push_back(ax(1));
          data.push_back(ax(2));
          data.push_back(0.5);
          openMBVXAcceleration->append(data);
          data.clear();
          Vec3 ay =  (a.T()*ey)*ey;
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(ay(0));
          data.push_back(ay(1));
          data.push_back(ay(2));
          data.push_back(0.5);
          openMBVYAcceleration->append(data);
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
          plotVector.push_back(a(0));
          plotVector.push_back(a(1));
          plotVector.push_back(a(2));
        }

        if(openMBVFrame && !openMBVFrame->isHDF5Link()) {
          vector<double> data;
          SqrMat3 AWP;
          AWP.set(0, ex);
          AWP.set(1, ey);
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
