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
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Gear::Gear(const string &name) : LinkMechanics(name)//, frame(0)
  {
    body.push_back(0); 
    ratio[0].push_back(-1);
    ratio[1].push_back(-1);
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

  void Gear::addDependency(RigidBody* body_, double ratio1, double ratio2) {
    body.push_back(body_); 
    ratio[0].push_back(ratio1);
    ratio[1].push_back((int)ratio2==0?ratio1:ratio2);
  }

  void Gear::updateW(double t, int j) {
    //  if(j==0) {
    //    for(unsigned i=0; i<body.size(); i++)  {
    //      W[j][i]-=body[i]->getJRel(j).T()*ratio[i];
    //    }
    //  } else {
    for(unsigned i=0; i<body.size(); i++) {
      W[j][i]-=body[i]->getFrames()[0]->getJacobianOfRotation(j).T()*(body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR()*ratio[1][i]);
      W[j][body.size()]+=frame->getJacobianOfRotation(j).T()*(body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR()*ratio[1][i]);
    }
    //  }
  }

  void Gear::updateh(double t, int j) {
    la(0) = (*func)(g(0),gd(0));
    for(unsigned i=0; i<body.size(); i++) {
      h[j][i]-=body[i]->getFrames()[0]->getJacobianOfRotation(j).T()*(body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR()*ratio[1][i]*la);
      h[j][body.size()]+=frame->getJacobianOfRotation(j).T()*(body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR()*ratio[1][i]*la);
    }
  }

  void Gear::updateWRef(const Mat &WParent, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);

      W[j][i]>>WParent(I,J);
    }
    Index J = Index(laInd,laInd+laSize-1);
    Index I = Index(frame->gethInd(j),frame->gethInd(j)+frame->getJacobianOfTranslation(j).cols()-1); // TODO Prüfen ob hSize
    W[j][body.size()]>>WParent(I,J);
  } 

  void Gear::updatehRef(const Vec &hParent, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Index I = Index(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      h[j][i]>>hParent(I);
    }
    Index I = Index(frame->gethInd(j),frame->gethInd(j)+frame->getJacobianOfTranslation(j).cols()-1); // TODO Prüfen ob hSize
    h[j][body.size()]>>hParent(I);
  } 

  void Gear::updateg(double) {
    g.init(0);
    for(unsigned i=0; i<body.size(); i++)
      g+=body[i]->getqRel()*ratio[0][i];
  } 

  void Gear::updategd(double) {
    gd.init(0);
  //  Vec buf(1);
    for(unsigned i=0; i<body.size(); i++) {
      //buf += (body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR()*ratio[1][i]).T()*(body[i]->getFrames()[0]->getAngularVelocity()-frame->getAngularVelocity());
      gd+=body[i]->getuRel()*ratio[0][i];
    }
  }

  bool Gear::isSetValued() const {
    return func?false:true;
  }

  void Gear::updatewb(double t, int j) {
    //buf += (frame->getOrientation()*body[i]->getPJR()*ratio[1][i]).T()*(body[i]->getFrames()[0]->getAngularVelocity()-frame->getAngularVelocity());

    for(unsigned i=0; i<body.size(); i++)
      wb += (body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR()*ratio[1][i]).T()*(body[i]->getFrames()[0]->getGyroscopicAccelerationOfRotation(j) - frame->getGyroscopicAccelerationOfRotation(j)); 
  }

  void Gear::init(InitStage stage) {
    if(stage==unknownStage) {
      //LinkMechanics::init(stage);

      for(unsigned int i=0; i<body.size(); i++) {
        h[0].push_back(Vec(body[i]->getJRel(0).cols()));
        h[1].push_back(Vec(6));
        W[0].push_back(Mat(body[i]->getJRel(0).cols(),laSize));
        W[1].push_back(Mat(6,laSize));
        assert(body[i]->getRotation()!=NULL);
      }
      h[0].push_back(Vec(frame->getJacobianOfTranslation(0).cols()));
      h[1].push_back(Vec(6));
      W[0].push_back(Mat(frame->getJacobianOfTranslation(0).cols(),laSize));
      W[1].push_back(Mat(6,laSize));
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
      for(unsigned int i=0; i<body.size(); i++) {
        plotColumns.push_back("M");
      }
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
    for(unsigned int i=0; i<body.size(); i++) {
      plotVector.push_back(ratio[1][i]*la(0));
    }
    if(getPlotFeature(plotRecursive)==enabled) {
      LinkMechanics::plot(t,dt);
    }
  }

}


