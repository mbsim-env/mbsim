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
#include "mbsim/plot_elements/plot_natural_coordinates.h"
#include "mbsim/rigid_body.h"
#include "mbsim/frame.h"
#include "mbsim/environment.h"
#include "mbsim/utils/rotarymatrices.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/frame.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  PlotNaturalCoordinates::PlotNaturalCoordinates(const std::string &name) : Element(name), frame(0), roff(3), voff(3), aoff(3), rscale(1), vscale(1), ascale(1) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVPosition=0;
    openMBVVelocity=0;
    openMBVTangentialAcceleration=0;
    openMBVNormalAcceleration=0;
    openMBVFrame=0;
#endif
  }

  void PlotNaturalCoordinates::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          openMBVGrp=new OpenMBV::Group();
          openMBVGrp->setName(name+"_Group");
          openMBVGrp->setExpand(false);
          parent->getOpenMBVGrp()->addObject(openMBVGrp);
          if(openMBVPosition) {
            openMBVPosition->setName(name+"_Position");
            getOpenMBVGrp()->addObject(openMBVPosition);
          }
          if(openMBVVelocity) {
            openMBVVelocity->setName(name+"_Velocity");
            getOpenMBVGrp()->addObject(openMBVVelocity);
          }
          if(openMBVTangentialAcceleration) {
            openMBVTangentialAcceleration->setName(name+"_TangentialAcceleration");
            getOpenMBVGrp()->addObject(openMBVTangentialAcceleration);
            openMBVNormalAcceleration->setName(name+"_NormalAcceleration");
            getOpenMBVGrp()->addObject(openMBVNormalAcceleration);
            openMBVFrame->setName(name+"_Dreibein");
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
  void PlotNaturalCoordinates::enableOpenMBVPosition(double scaleLength, double diameter, double headDiameter, double headLength, double color) {
    if(scaleLength>=0) {
      openMBVPosition=new OpenMBV::Arrow;
      openMBVPosition->setScaleLength(scaleLength);
      openMBVPosition->setDiameter(diameter);
      openMBVPosition->setHeadDiameter(headDiameter);
      openMBVPosition->setHeadLength(headLength);
      openMBVPosition->setStaticColor(color);
    }
    else {
      openMBVPosition=0;
    }
  }

  void PlotNaturalCoordinates::enableOpenMBVVelocity(double scale, double diameter, double headDiameter, double headLength, double color) {
    openMBVVelocity=new OpenMBV::Arrow;
    openMBVVelocity->setDiameter(diameter);
    openMBVVelocity->setHeadDiameter(headDiameter);
    openMBVVelocity->setHeadLength(headLength);
    openMBVVelocity->setStaticColor(color);
    vscale = scale;
  }

  void PlotNaturalCoordinates::enableOpenMBVAcceleration(double scale, double diameter, double headDiameter, double headLength, double color) {
    openMBVTangentialAcceleration=new OpenMBV::Arrow;
    openMBVTangentialAcceleration->setDiameter(diameter);
    openMBVTangentialAcceleration->setHeadDiameter(headDiameter);
    openMBVTangentialAcceleration->setHeadLength(headLength);
    openMBVTangentialAcceleration->setStaticColor(color);
    openMBVNormalAcceleration=new OpenMBV::Arrow;
    openMBVNormalAcceleration->setDiameter(diameter);
    openMBVNormalAcceleration->setHeadDiameter(headDiameter);
    openMBVNormalAcceleration->setHeadLength(headLength);
    openMBVNormalAcceleration->setStaticColor(color);
    openMBVFrame=new OpenMBV::Frame;
    openMBVFrame->setSize(0.1);
    openMBVFrame->setOffset(1);
    ascale = scale;
  }
#endif

  void PlotNaturalCoordinates::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(0.5);
          openMBVPosition->append(data);
        }
        if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          Vec vframe = frame->getVelocity()*vscale;
          Vec off = voff + vframe;
          data.push_back(frame->getPosition()(0)+off(0));
          data.push_back(frame->getPosition()(1)+off(1));
          data.push_back(frame->getPosition()(2)+off(2));
          data.push_back(vframe(0));
          data.push_back(vframe(1));
          data.push_back(vframe(2));
          data.push_back(0.5);
          openMBVVelocity->append(data);
        }
        if(openMBVTangentialAcceleration && !openMBVTangentialAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec v = frame->getVelocity();
          double nrmv = nrm2(v);
          Vec et(3);
          if(nrmv<epsroot())
            et(0) = 1;
          else
            et = v/nrmv;
          Vec a = frame->getAcceleration()*ascale;
          Vec eb = crossProduct(et,a);
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
          Vec en = -crossProduct(et,eb);
          Vec at =  (a.T()*et)*et;
          Vec an =  (a.T()*en)*en;
          Vec ab =  (a.T()*eb)*eb;
          data.push_back(t);
          Vec off = at;
          data.push_back(frame->getPosition()(0)+off(0));
          data.push_back(frame->getPosition()(1)+off(1));
          data.push_back(frame->getPosition()(2)+off(2));
          data.push_back(at(0));
          data.push_back(at(1));
          data.push_back(at(2));
          data.push_back(0.5);
          openMBVTangentialAcceleration->append(data);
          vector<double> data2;
          data2.push_back(t);
          off = an;
          data2.push_back(frame->getPosition()(0)+off(0));
          data2.push_back(frame->getPosition()(1)+off(1));
          data2.push_back(frame->getPosition()(2)+off(2));
          data2.push_back(an(0));
          data2.push_back(an(1));
          data2.push_back(an(2));
          data2.push_back(0.5);
          openMBVNormalAcceleration->append(data2);

          SqrMat AWP(3);
          AWP.col(0) = et;
          AWP.col(1) = en;
          AWP.col(2) = eb;
          vector<double> data3;
          data3.push_back(t);
          data3.push_back(frame->getPosition()(0));
          data3.push_back(frame->getPosition()(1));
          data3.push_back(frame->getPosition()(2));
          Vec cardan=AIK2Cardan(AWP);
          data3.push_back(cardan(0));
          data3.push_back(cardan(1));
          data3.push_back(cardan(2));
          data3.push_back(0);
          openMBVFrame->append(data3);

        }
      }
#endif

      Element::plot(t,dt);
    }
  }

}
