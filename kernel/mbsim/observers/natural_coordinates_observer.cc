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
#include "mbsim/observers/natural_coordinates_observer.h"
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

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, NaturalCoordinatesObserver)

  NaturalCoordinatesObserver::NaturalCoordinatesObserver(const std::string &name) : CoordinatesObserver(name) {
  }

  void NaturalCoordinatesObserver::init(InitStage stage) {
    if(stage==plotting) {
      updatePlotFeatures();

      if(openMBVAcceleration) {
        plotColumns.push_back("TangentialAcceleration");
        plotColumns.push_back("NormalAcceleration");
      }
      CoordinatesObserver::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVAcceleration) {
            openMBVTangentialAcceleration = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVNormalAcceleration = OpenMBV::ObjectFactory::create(openMBVAcceleration);
            openMBVTangentialAcceleration->setName("TangentialAcceleration");
            openMBVAccGrp->addObject(openMBVTangentialAcceleration);
            openMBVNormalAcceleration->setName("NormalAcceleration");
            openMBVAccGrp->addObject(openMBVNormalAcceleration);
          }
        }
#endif
      }
    }
    else
      CoordinatesObserver::init(stage);
  }

  void NaturalCoordinatesObserver::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        Vec3 r = frame->evalPosition();
        Vec3 v = frame->evalVelocity();
        Vec3 a = frame->evalAcceleration();
        double nrmv = nrm2(v);
        Vec3 et;
        if(nrmv<epsroot())
          et(0) = 1;
        else
          et = v/nrmv;
        Vec3 eb = crossProduct(et,a);
        double nrmeb = nrm2(eb);
        if(nrmeb<epsroot()) {
          if(fabs(et(0))<epsroot() && fabs(et(1))<epsroot()) {
            eb(0) = 1.;
            eb(1) = 0.;
            eb(2) = 0.;
          }
          else {
            eb(0) = -et(1);
            eb(1) = et(0);
            eb(2) = 0.0;
          }
        }
        else
          eb = eb/nrmeb;
        Vec3 en = -crossProduct(et,eb);

        if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec3 at =  (a.T()*et)*et;
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(at(0));
          data.push_back(at(1));
          data.push_back(at(2));
          data.push_back(0.5);
          openMBVTangentialAcceleration->append(data);
          data.clear();
          Vec3 an =  (a.T()*en)*en;
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(an(0));
          data.push_back(an(1));
          data.push_back(an(2));
          data.push_back(0.5);
          openMBVNormalAcceleration->append(data);
          plotVector.push_back(a.T()*et);
          plotVector.push_back(a.T()*en);
        }

        if(openMBVFrame && !openMBVFrame->isHDF5Link()) {
          vector<double> data;
          SqrMat3 AWP;
          AWP.set(0, et);
          AWP.set(1, en);
          AWP.set(2, eb);
          data.push_back(getTime());
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
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
