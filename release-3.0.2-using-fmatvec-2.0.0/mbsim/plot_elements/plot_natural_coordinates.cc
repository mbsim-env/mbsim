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
            openMBVPosition->setName("Position");
            getOpenMBVGrp()->addObject(openMBVPosition);
          }
          if(openMBVVelocity) {
            openMBVVelocity->setName("Velocity");
            getOpenMBVGrp()->addObject(openMBVVelocity);
          }
          if(openMBVTangentialVelocity) {
            openMBVTangentialVelocity->setName("TangentialVelocity");
            getOpenMBVGrp()->addObject(openMBVTangentialVelocity);
          }
          if(openMBVNormalVelocity) {
            openMBVNormalVelocity->setName("NormalVelocity");
            getOpenMBVGrp()->addObject(openMBVNormalVelocity);
          }
          if(openMBVBinormalVelocity) {
            openMBVBinormalVelocity->setName("BinormalVelocity");
            getOpenMBVGrp()->addObject(openMBVBinormalVelocity);
          }
          if(openMBVAcceleration) {
            openMBVAcceleration->setName("Acceleration");
            getOpenMBVGrp()->addObject(openMBVAcceleration);
          }
          if(openMBVTangentialAcceleration) {
            openMBVTangentialAcceleration->setName("TangentialAcceleration");
            getOpenMBVGrp()->addObject(openMBVTangentialAcceleration);
          }
          if(openMBVNormalAcceleration) {
            openMBVNormalAcceleration->setName("NormalAcceleration");
            getOpenMBVGrp()->addObject(openMBVNormalAcceleration);
          }
          if(openMBVBinormalAcceleration) {
            openMBVBinormalAcceleration->setName("BinormalAcceleration");
            getOpenMBVGrp()->addObject(openMBVBinormalAcceleration);
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
  void PlotNaturalCoordinates::enableOpenMBVPosition(double diameter, double headDiameter, double headLength, double color) {
    openMBVPosition=new OpenMBV::Arrow;
    openMBVPosition->setDiameter(diameter);
    openMBVPosition->setHeadDiameter(headDiameter);
    openMBVPosition->setHeadLength(headLength);
    openMBVPosition->setStaticColor(color);
  }

  void PlotNaturalCoordinates::enableOpenMBVVelocity(double scale, double diameter, double headDiameter, double headLength, double color) {
    openMBVVelocity=new OpenMBV::Arrow;
    openMBVVelocity->setDiameter(diameter);
    openMBVVelocity->setHeadDiameter(headDiameter);
    openMBVVelocity->setHeadLength(headLength);
    openMBVVelocity->setStaticColor(color);
    openMBVTangentialVelocity=new OpenMBV::Arrow;
    openMBVTangentialVelocity->setDiameter(diameter);
    openMBVTangentialVelocity->setHeadDiameter(headDiameter);
    openMBVTangentialVelocity->setHeadLength(headLength);
    openMBVTangentialVelocity->setStaticColor(color);
    openMBVNormalVelocity=new OpenMBV::Arrow;
    openMBVNormalVelocity->setDiameter(diameter);
    openMBVNormalVelocity->setHeadDiameter(headDiameter);
    openMBVNormalVelocity->setHeadLength(headLength);
    openMBVNormalVelocity->setStaticColor(color);
    openMBVBinormalVelocity=new OpenMBV::Arrow;
    openMBVBinormalVelocity->setDiameter(diameter);
    openMBVBinormalVelocity->setHeadDiameter(headDiameter);
    openMBVBinormalVelocity->setHeadLength(headLength);
    openMBVBinormalVelocity->setStaticColor(color);
    vscale = scale;
  }

  void PlotNaturalCoordinates::enableOpenMBVAcceleration(double scale, double diameter, double headDiameter, double headLength, double color) {
    openMBVAcceleration=new OpenMBV::Arrow;
    openMBVAcceleration->setDiameter(diameter);
    openMBVAcceleration->setHeadDiameter(headDiameter);
    openMBVAcceleration->setHeadLength(headLength);
    openMBVAcceleration->setStaticColor(color);
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
    openMBVBinormalAcceleration=new OpenMBV::Arrow;
    openMBVBinormalAcceleration->setDiameter(diameter);
    openMBVBinormalAcceleration->setHeadDiameter(headDiameter);
    openMBVBinormalAcceleration->setHeadLength(headLength);
    openMBVBinormalAcceleration->setStaticColor(color);
    ascale = scale;
  }

  void PlotNaturalCoordinates::enableOpenMBVFrame(double size, double offset) {
    openMBVFrame=new OpenMBV::Frame;
    openMBVFrame->setSize(size);
    openMBVFrame->setOffset(offset);
  }
#endif

  void PlotNaturalCoordinates::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        Vec r = frame->getPosition();
        Vec v = frame->getVelocity();
        Vec a = frame->getAcceleration();
        double nrmv = nrm2(v);
        Vec et(3);
        if(nrmv<epsroot())
          et(0) = 1;
        else
          et = v/nrmv;
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

        v *= vscale;
        a *= ascale;
        if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          data.push_back(0.5);
          openMBVPosition->append(data);
        }

        if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          Vec off = v;
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(v(0));
          data.push_back(v(1));
          data.push_back(v(2));
          data.push_back(0.5);
          openMBVVelocity->append(data);
        }

        if(openMBVTangentialVelocity && !openMBVTangentialVelocity->isHDF5Link()) {
          vector<double> data;
          Vec vt =  (v.T()*et)*et;
          Vec off = vt;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(vt(0));
          data.push_back(vt(1));
          data.push_back(vt(2));
          data.push_back(0.5);
          openMBVTangentialVelocity->append(data);
        }

        if(openMBVNormalVelocity && !openMBVNormalVelocity->isHDF5Link()) {
          vector<double> data;
          Vec vn =  (v.T()*en)*en;
          Vec off = vn;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(vn(0));
          data.push_back(vn(1));
          data.push_back(vn(2));
          data.push_back(0.5);
          openMBVNormalVelocity->append(data);
        }

        if(openMBVBinormalVelocity && !openMBVBinormalVelocity->isHDF5Link()) {
          vector<double> data;
          Vec vb =  (v.T()*eb)*eb;
          Vec off = vb;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(vb(0));
          data.push_back(vb(1));
          data.push_back(vb(2));
          data.push_back(0.5);
          openMBVBinormalVelocity->append(data);
        }

        if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          Vec off = a;
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(a(0));
          data.push_back(a(1));
          data.push_back(a(2));
          data.push_back(0.5);
          openMBVAcceleration->append(data);
        }

        if(openMBVTangentialAcceleration && !openMBVTangentialAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec at =  (a.T()*et)*et;
          Vec off = at;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(at(0));
          data.push_back(at(1));
          data.push_back(at(2));
          data.push_back(0.5);
          openMBVTangentialAcceleration->append(data);
        }

        if(openMBVNormalAcceleration && !openMBVNormalAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec an =  (a.T()*en)*en;
          Vec off = an;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(an(0));
          data.push_back(an(1));
          data.push_back(an(2));
          data.push_back(0.5);
          openMBVNormalAcceleration->append(data);
        }

        if(openMBVBinormalAcceleration && !openMBVBinormalAcceleration->isHDF5Link()) {
          vector<double> data;
          Vec ab =  (a.T()*eb)*eb;
          Vec off = ab;
          data.push_back(t);
          data.push_back(r(0)+off(0));
          data.push_back(r(1)+off(1));
          data.push_back(r(2)+off(2));
          data.push_back(ab(0));
          data.push_back(ab(1));
          data.push_back(ab(2));
          data.push_back(0.5);
          openMBVBinormalAcceleration->append(data);
        }

        if(openMBVFrame && !openMBVFrame->isHDF5Link()) {
          vector<double> data;
          SqrMat AWP(3);
          AWP.col(0) = et;
          AWP.col(1) = en;
          AWP.col(2) = eb;
          data.push_back(t);
          data.push_back(r(0));
          data.push_back(r(1));
          data.push_back(r(2));
          Vec cardan=AIK2Cardan(AWP);
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
