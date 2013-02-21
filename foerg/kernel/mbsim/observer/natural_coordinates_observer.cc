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
#include "mbsim/observer/natural_coordinates_observer.h"
#include "mbsim/frame.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/utils/eps.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frame.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  NaturalCoordinatesObserver::NaturalCoordinatesObserver(const std::string &name) : Observer(name), frame(0) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVPosition=0;
    openMBVVelocity=0;
    openMBVTangentialVelocity=0;
    openMBVNormalVelocity=0;
    openMBVBinormalVelocity=0;
    openMBVAcceleration=0;
    openMBVTangentialAcceleration=0;
    openMBVNormalAcceleration=0;
    openMBVBinormalAcceleration=0;
    openMBVFrame=0;
#endif
  }

  void NaturalCoordinatesObserver::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();

      Observer::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVPosition) {
            OpenMBV::Group *openMBVGrp=new OpenMBV::Group();
            openMBVGrp->setName("Position_Group");
            openMBVGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVGrp);
            openMBVPosition->setName("Position");
            openMBVGrp->addObject(openMBVPosition);
          }
          if(openMBVVelocity) {
            OpenMBV::Group *openMBVGrp=new OpenMBV::Group();
            openMBVGrp->setName("Velocity_Group");
            openMBVGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVGrp);
            openMBVVelocity->setName("Velocity");
            openMBVGrp->addObject(openMBVVelocity);
            openMBVTangentialVelocity->setName("TangentialVelocity");
            openMBVGrp->addObject(openMBVTangentialVelocity);
            openMBVNormalVelocity->setName("NormalVelocity");
            openMBVGrp->addObject(openMBVNormalVelocity);
            openMBVBinormalVelocity->setName("BinormalVelocity");
            openMBVGrp->addObject(openMBVBinormalVelocity);
          }
          if(openMBVAcceleration) {
            OpenMBV::Group *openMBVGrp=new OpenMBV::Group();
            openMBVGrp->setName("Acceleration_Group");
            openMBVGrp->setExpand(false);
            getOpenMBVGrp()->addObject(openMBVGrp);
            openMBVAcceleration->setName("Acceleration");
            openMBVGrp->addObject(openMBVAcceleration);
            openMBVTangentialAcceleration->setName("TangentialAcceleration");
            openMBVGrp->addObject(openMBVTangentialAcceleration);
            openMBVNormalAcceleration->setName("NormalAcceleration");
            openMBVGrp->addObject(openMBVNormalAcceleration);
            openMBVBinormalAcceleration->setName("BinormalAcceleration");
            openMBVGrp->addObject(openMBVBinormalAcceleration);
          }
          if(openMBVFrame) {
            openMBVFrame->setName("Frame");
            getOpenMBVGrp()->addObject(openMBVFrame);
          }
        }
#endif
      }
      Element::init(stage);
    }
    else
      Element::init(stage);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void NaturalCoordinatesObserver::enableOpenMBVPosition(double diameter, double headDiameter, double headLength, double color) {
    openMBVPosition=new OpenMBV::Arrow;
    openMBVPosition->setReferencePoint(OpenMBV::Arrow::fromPoint);
    openMBVPosition->setDiameter(diameter);
    openMBVPosition->setHeadDiameter(headDiameter);
    openMBVPosition->setHeadLength(headLength);
    openMBVPosition->setStaticColor(color);
  }

  void NaturalCoordinatesObserver::enableOpenMBVVelocity(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVVelocity=new OpenMBV::Arrow;
    openMBVVelocity->setScaleLength(scale);
    openMBVVelocity->setReferencePoint(refPoint);
    openMBVVelocity->setDiameter(diameter);
    openMBVVelocity->setHeadDiameter(headDiameter);
    openMBVVelocity->setHeadLength(headLength);
    openMBVVelocity->setStaticColor(color);
    openMBVTangentialVelocity=new OpenMBV::Arrow;
    openMBVTangentialVelocity->setScaleLength(scale);
    openMBVTangentialVelocity->setReferencePoint(refPoint);
    openMBVTangentialVelocity->setDiameter(diameter);
    openMBVTangentialVelocity->setHeadDiameter(headDiameter);
    openMBVTangentialVelocity->setHeadLength(headLength);
    openMBVTangentialVelocity->setStaticColor(color);
    openMBVNormalVelocity=new OpenMBV::Arrow;
    openMBVNormalVelocity->setScaleLength(scale);
    openMBVNormalVelocity->setReferencePoint(refPoint);
    openMBVNormalVelocity->setDiameter(diameter);
    openMBVNormalVelocity->setHeadDiameter(headDiameter);
    openMBVNormalVelocity->setHeadLength(headLength);
    openMBVNormalVelocity->setStaticColor(color);
    openMBVBinormalVelocity=new OpenMBV::Arrow;
    openMBVBinormalVelocity->setScaleLength(scale);
    openMBVBinormalVelocity->setReferencePoint(refPoint);
    openMBVBinormalVelocity->setDiameter(diameter);
    openMBVBinormalVelocity->setHeadDiameter(headDiameter);
    openMBVBinormalVelocity->setHeadLength(headLength);
    openMBVBinormalVelocity->setStaticColor(color);
  }

  void NaturalCoordinatesObserver::enableOpenMBVAcceleration(double scale, OpenMBV::Arrow::ReferencePoint refPoint, double diameter, double headDiameter, double headLength, double color) {
    openMBVAcceleration=new OpenMBV::Arrow;
    openMBVAcceleration->setScaleLength(scale);
    openMBVAcceleration->setReferencePoint(refPoint);
    openMBVAcceleration->setDiameter(diameter);
    openMBVAcceleration->setHeadDiameter(headDiameter);
    openMBVAcceleration->setHeadLength(headLength);
    openMBVAcceleration->setStaticColor(color);
    openMBVTangentialAcceleration=new OpenMBV::Arrow;
    openMBVTangentialAcceleration->setScaleLength(scale);
    openMBVTangentialAcceleration->setReferencePoint(refPoint);
    openMBVTangentialAcceleration->setDiameter(diameter);
    openMBVTangentialAcceleration->setHeadDiameter(headDiameter);
    openMBVTangentialAcceleration->setHeadLength(headLength);
    openMBVTangentialAcceleration->setStaticColor(color);
    openMBVNormalAcceleration=new OpenMBV::Arrow;
    openMBVNormalAcceleration->setScaleLength(scale);
    openMBVNormalAcceleration->setReferencePoint(refPoint);
    openMBVNormalAcceleration->setDiameter(diameter);
    openMBVNormalAcceleration->setHeadDiameter(headDiameter);
    openMBVNormalAcceleration->setHeadLength(headLength);
    openMBVNormalAcceleration->setStaticColor(color);
    openMBVBinormalAcceleration=new OpenMBV::Arrow;
    openMBVBinormalAcceleration->setScaleLength(scale);
    openMBVBinormalAcceleration->setReferencePoint(refPoint);
    openMBVBinormalAcceleration->setDiameter(diameter);
    openMBVBinormalAcceleration->setHeadDiameter(headDiameter);
    openMBVBinormalAcceleration->setHeadLength(headLength);
    openMBVBinormalAcceleration->setStaticColor(color);
  }

  void NaturalCoordinatesObserver::enableOpenMBVFrame(double size, double offset) {
    openMBVFrame=new OpenMBV::Frame;
    openMBVFrame->setSize(size);
    openMBVFrame->setOffset(offset);
  }
#endif

  void NaturalCoordinatesObserver::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        Vec3 r = frame->getPosition();
        Vec3 v = frame->getVelocity();
        Vec3 a = frame->getAcceleration();
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

        if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(0.5);
          openMBVPosition->append(data);
        }

        if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(v(0));
          data.push_back(v(1));
          data.push_back(v(2));
          data.push_back(0.5);
          openMBVVelocity->append(data);
          data.clear();
          Vec3 vt =  (v.T()*et)*et;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vt(0));
          data.push_back(vt(1));
          data.push_back(vt(2));
          data.push_back(0.5);
          openMBVTangentialVelocity->append(data);
          data.clear();
          Vec3 vn =  (v.T()*en)*en;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vn(0));
          data.push_back(vn(1));
          data.push_back(vn(2));
          data.push_back(0.5);
          openMBVNormalVelocity->append(data);
          data.clear();
          Vec3 vb =  (v.T()*eb)*eb;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(vb(0));
          data.push_back(vb(1));
          data.push_back(vb(2));
          data.push_back(0.5);
          openMBVBinormalVelocity->append(data);
        }

        if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(a(0));
          data.push_back(a(1));
          data.push_back(a(2));
          data.push_back(0.5);
          openMBVAcceleration->append(data);
          data.clear();
          Vec3 at =  (a.T()*et)*et;
          data.push_back(t);
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
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(an(0));
          data.push_back(an(1));
          data.push_back(an(2));
          data.push_back(0.5);
          openMBVNormalAcceleration->append(data);
          data.clear();
          Vec3 ab =  (a.T()*eb)*eb;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(ab(0));
          data.push_back(ab(1));
          data.push_back(ab(2));
          data.push_back(0.5);
          openMBVBinormalAcceleration->append(data);
        }

        if(openMBVFrame && !openMBVFrame->isHDF5Link()) {
          vector<double> data;
          SqrMat3 AWP;
          AWP.set(0, et);
          AWP.set(1, en);
          AWP.set(2, eb);
          data.push_back(t);
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

      Element::plot(t,dt);
    }
  }

}
