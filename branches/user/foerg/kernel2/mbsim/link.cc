/* Copyright (C) 2004-2009 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/link.h"
#include "mbsim/frame.h"
#include "mbsim/object.h"
#include "mbsim/utils/utils.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/dynamic_system_solver.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  Link::Link(const string &name) : Element(name), xSize(0), xInd(0), svSize(0), svInd(0), LinkStatusSize(0), LinkStatusInd(0), gSize(0), gInd(0), gdSize(0), gdInd(0), laSize(0), laInd(0), gTol(1e-8), gdTol(1e-10), gddTol(1e-12), laTol(1e-12), LaTol(1e-10), rFactorSize(0), rFactorInd(0), rMax(1.) {
    setPlotFeature(state, enabled);
    setPlotFeature(generalizedLinkForce, enabled);
    setPlotFeature(linkKinematics, enabled);
  }

  void Link::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(state)==enabled)
        for(int i=0; i<xSize; ++i)
          plotVector.push_back(x(i));
      if(getPlotFeature(stateDerivative)==enabled)
        for(int i=0; i<xSize; ++i)
          plotVector.push_back(xd(i)/dt);
      if(getPlotFeature(linkKinematics)==enabled) {
        for(int i=0; i<g.size(); ++i)
          plotVector.push_back(g(i));
        for(int i=0; i<gd.size(); ++i)
          plotVector.push_back(gd(i));
      }
      if(getPlotFeature(generalizedLinkForce)==enabled) {
        for(int i=0; i<la.size(); ++i)
          plotVector.push_back(la(i));
      }
      if(getPlotFeature(stopVector)==enabled)
        for(int i=0; i<sv.size(); ++i)
          plotVector.push_back(sv(i));
      if(getPlotFeature(energy)==enabled) {
        plotVector.push_back(computePotentialEnergy()); 
      }

      Element::plot(t,dt);
    }
  }

  void Link::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::closePlot();
    }
  }

  void Link::updatewbRef(const Vec& wbParent) {
    wb.resize() >> wbParent(laInd,laInd+laSize-1);
  }

  void Link::updatexRef(const Vec &xParent) {
    x >> xParent(xInd,xInd+xSize-1);
  } 

  void Link::updatexdRef(const Vec &xdParent) {
    xd >> xdParent(xInd,xInd+xSize-1);
  } 

  void Link::updatelaRef(const Vec& laParent) {
    la.resize() >> laParent(laInd,laInd+laSize-1);
  }

  void Link::deletelaRef() {
    la.resize(la.size(), NONINIT);
  }

  void Link::updategRef(const Vec& gParent) {
    g.resize() >> gParent(gInd,gInd+gSize-1);
  }

  void Link::updategdRef(const Vec& gdParent) {
    gd.resize() >> gdParent(gdInd,gdInd+gdSize-1);
  }

  void Link::updateresRef(const Vec& resParent) {
    res.resize() >> resParent(laInd,laInd+laSize-1);
  }

  void Link::updaterFactorRef(const Vec& rFactorParent) {
    rFactor.resize() >> rFactorParent(rFactorInd,rFactorInd+rFactorSize-1);
  }

  void Link::updatesvRef(const Vec &svParent) {
    sv >> svParent(svInd,svInd+svSize-1);
  }

  void Link::updatejsvRef(const VecInt &jsvParent) {
    jsv >> jsvParent(svInd,svInd+svSize-1);
  }
   
  void Link::updateLinkStatusRef(const VecInt &LinkStatusParent) {
    LinkStatus.resize() >> LinkStatusParent(LinkStatusInd,LinkStatusInd+LinkStatusSize-1);
  }

  void Link::updatebRef(const Mat &bParent) {
    Index J = Index(laInd,laInd+laSize-1);
    Index I = Index(bInd,bInd+bSize-1);
    b.resize()>>bParent(I,J);
  } 

  void Link::init(InitStage stage) {
    if(stage==unknownStage) {
      rFactorUnsure.resize(rFactorSize);
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(state)==enabled)
          for(int i=0; i<xSize; ++i)
            plotColumns.push_back("x("+numtostr(i)+")");
        if(getPlotFeature(stateDerivative)==enabled)
          for(int i=0; i<xSize; ++i)
            plotColumns.push_back("xd("+numtostr(i)+")");
        if(getPlotFeature(linkKinematics)==enabled) {
          for(int i=0; i<g.size(); ++i)
            plotColumns.push_back("g("+numtostr(i)+")");
          for(int i=0; i<gd.size(); ++i)
            plotColumns.push_back("gd("+numtostr(i)+")");
        }
        if(getPlotFeature(generalizedLinkForce)==enabled) {
          for(int i=0; i<la.size(); ++i)
            plotColumns.push_back("la("+numtostr(i)+")");
        }
        if(getPlotFeature(stopVector)==enabled)
          for(int i=0; i<svSize; ++i)
            plotColumns.push_back("sv("+numtostr(i)+")");
        if(getPlotFeature(energy)==enabled)
          plotColumns.push_back("V");

        Element::init(stage);
      }
    }
    else
      Element::init(stage);
  }

  void Link::initz() {
    x=x0;
  }

  void Link::savela(double dt) {
    la0.resize() = la/dt;
  }

  void Link::initla(double dt) {
    if(la0.size() == la.size()) // TODO check if initialising to 0 is better if contact was inactive before
      la = la0*dt;
    else
      la.init(0);
  }

  void Link::decreaserFactors() {
    for(int i=0; i<rFactor.size(); i++)
      if(rFactorUnsure(i))
        rFactor(i) *= 0.9;
  }

  Element * Link::getByPathSearch(string path) {
    if (path.substr(0, 3)=="../") // relative path
      return parent->getByPathSearch(path.substr(3));
    else // absolut path
      if(parent)
        return parent->getByPathSearch(path);
      else
        return getByPathSearch(path.substr(1));
  }

  void Link::updatecorrRef(const fmatvec::Vec &ref) {
    corr.resize() >> ref(corrInd,corrInd+corrSize-1);
  }

}

