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
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h>
#include "mbsim/link.h"
#include "mbsim/utils/function.h"
#include "mbsim/dynamic_system.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  Link::Link(const string &name) : Element(name), parent(0), xSize(0), xInd(0), svSize(0), svInd(0), gSize(0), gInd(0), gdSize(0), gdInd(0), laSize(0), laInd(0), gdTol(1e-6), gddTol(1e-6), laTol(1e-6), LaTol(1e-6), rFactorSize(0), rFactorInd(0), rMax(1.), laIndDS(0) {}

  void Link::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(state)==enabled)
        for(int i=0; i<xSize; ++i)
          plotVector.push_back(x(i));
      if(getPlotFeature(stateDerivative)==enabled)
        for(int i=0; i<xSize; ++i)
          plotVector.push_back(xd(i)/dt);
      if(getPlotFeature(contact)==enabled) {
        for(int i=0; i<g.size(); ++i)
          plotVector.push_back(g(i));
        if(isActive()) {
          for(int i=0; i<sv.size(); ++i)
            plotVector.push_back(sv(i));
          //for(int i=0; i<gdSize; ++i) TODO laSize, gdSize not constant during simulation
          //  plotVector.push_back(gd(i));
          //if(setValued)
          //  for(int i=0; i<la.size(); ++i)
          //    plotVector.push_back(la(i)/dt);
          //else
          //  for(int i=0; i<la.size(); ++i)
          //    plotVector.push_back(la(i));
        } 
        else {
          for(int i=0; i<sv.size(); ++i)
            plotVector.push_back(sv(i));
          //for(int i=0; i<gdSize; ++i) TODO laSize, gdSize not constant during simulation
          //  plotVector.push_back(gd(i));
          //for(int i=0; i<la.size(); ++i)
          //  plotVector.push_back(0);
        }
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

  void Link::updatejsvRef(const Vector<int> &jsvParent) {
    jsv >> jsvParent(svInd,svInd+svSize-1);
  }

  void Link::init() {
    rFactorUnsure.resize(rFactorSize);
  }

  void Link::initz() {
    x=x0;
  }

  void Link::initPlot() {
    updatePlotFeatures(parent);

    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(state)==enabled)
        for(int i=0; i<xSize; ++i)
          plotColumns.push_back("x("+numtostr(i)+")");
      if(getPlotFeature(stateDerivative)==enabled)
        for(int i=0; i<xSize; ++i)
          plotColumns.push_back("xd("+numtostr(i)+")");
      if(getPlotFeature(contact)==enabled) {
        for(int i=0; i<g.size(); ++i)
          plotColumns.push_back("g("+numtostr(i)+")");
        for(int i=0; i<sv.size(); ++i)
          plotColumns.push_back("sv("+numtostr(i)+")");
        // la.size()=laSize, gdSize ist nicht konstant ueber der Simulation
        //for(int i=0; i<gdSize; ++i)
        //  plotColumns.push_back("gd("+numtostr(i)+")");
        //for(int i=0; i<laSize; ++i)
        //  plotColumns.push_back("la("+numtostr(i)+")");
        plotColumns.push_back("V");
      }

      Element::initPlot(parent);
    }
  }

  void Link::savela() {
    la0.resize() = la;
  }

  void Link::initla() {
    if(la0.size() == la.size()) // TODO check if initialising to 0 is better if contact was inactive before
      la = la0;
    else
      la.init(0);
  }

  void Link::decreaserFactors() {
    for(int i=0; i<rFactor.size(); i++)
      if(rFactorUnsure(i))
        rFactor(i) *= 0.9;
  }

  FrameInterface *Link::getFrameByPath(std::string path) {
    if(path[path.length()-1]!='/') path=path+"/";
    size_t i=path.find('/');
    // absolut path
    if(i==0) {
      if(parent)
        return parent->getFrameByPath(path);
      else
        return getFrameByPath(path.substr(1));
    }
    // relative path
    string firstPart=path.substr(0, i);
    string restPart=path.substr(i+1);
    if(firstPart=="..")
      return parent->getFrameByPath(restPart);
    else
      return 0;
  }

  Contour *Link::getContourByPath(std::string path) {
    if(path[path.length()-1]!='/') path=path+"/";
    size_t i=path.find('/');
    // absolut path
    if(i==0) {
      if(parent)
        return parent->getContourByPath(path);
      else
        return getContourByPath(path.substr(1));
    }
    // relative path
    string firstPart=path.substr(0, i);
    string restPart=path.substr(i+1);
    if(firstPart=="..")
      return parent->getContourByPath(restPart);
    else
      return 0;
  }

}

