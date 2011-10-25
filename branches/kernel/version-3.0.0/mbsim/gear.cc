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
#include "mbsim/gear.h"
#include "mbsim/frame.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Gear::Gear(const string &name) : LinkMechanics(name) 
  {
    body.push_back(0); 
    ratio.push_back(-1);
  }

  void Gear::calclaSize() {
    laSize = 1;
  }
  void Gear::calcgSize() {
    gSize = 1;
  }
  void Gear::calcgdSize() {
    gdSize = 1;
  }

  void Gear::addDependency(RigidBody* body_, double ratio_) {
    body.push_back(body_); 
    ratio.push_back(ratio_);
  }

  void Gear::updateW(double t, int j) {
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++) 
        W[j][i]-=body[i]->getJRel(j).T()*ratio[i];
    } else {
      for(unsigned i=0; i<body.size(); i++) 
        W[j][i]-=body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR()*ratio[i];
    }
  }

  void Gear::updateh(double t, int j) {
    la(0) = (*func)(g(0),gd(0));
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++) 
        h[j][i]-=body[i]->getJRel(j).T()*ratio[i]*la;
    } else {
      for(unsigned i=0; i<body.size(); i++) 
        h[j][i]-=body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR()*ratio[i]*la;
    }
  }

  void Gear::updateWRef(const Mat &WParent, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(body[i]->gethInd(j)+(j==0?0:3),body[i]->gethInd(j)+(j==0?0:3)+(j==0?body[i]->getJRel(j).cols():3)-1);

      W[j][i].resize()>>WParent(I,J);
    }
  } 

  void Gear::updatehRef(const Vec &hParent, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Index I = Index(body[i]->gethInd(j)+(j==0?0:3),body[i]->gethInd(j)+(j==0?0:3)+(j==0?body[i]->getJRel(j).cols():3)-1);
      h[j][i]>>hParent(I);
    }
  } 

  void Gear::updateg(double) {
    g.init(0);
    for(unsigned i=0; i<body.size(); i++)
      g+=body[i]->getuRel()*ratio[i];
  } 

  void Gear::updategd(double) {
    gd.init(0);
    for(unsigned i=0; i<body.size(); i++)
      gd-=body[i]->getuRel()*ratio[i];
  }

  bool Gear::isSetValued() const {
    return func?false:true;
  }

  void Gear::init(InitStage stage) {
    if(stage==unknownStage) {
      LinkMechanics::init(stage);

      for(unsigned int i=0; i<body.size(); i++) {
        h[0].push_back(Vec(body[i]->getJRel(0).cols()));
        h[1].push_back(Vec(3));
        W[0].push_back(Mat(body[i]->getJRel(0).cols(),laSize));
        W[1].push_back(Mat(3,laSize));
        assert(body[i]->getRotation()!=NULL);
      }
    }
    else if(stage==resize) {
      LinkMechanics::init(stage);
      g.resize(1);
      gd.resize(1);
      la.resize(1);
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();
      plotColumns.push_back("la(0)");
      if(getPlotFeature(plotRecursive)==enabled) {
        LinkMechanics::init(stage);
      }
    }
    else {
      LinkMechanics::init(stage);
    }
  }

  void Gear::plot(double t,double dt) {
    plotVector.push_back(la(0));
    if(getPlotFeature(plotRecursive)==enabled) {
      LinkMechanics::plot(t,dt);
    }
  }

}


