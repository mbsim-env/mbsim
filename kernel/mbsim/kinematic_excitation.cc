/* Copyright (C) 2004-2015 MBSim Development Team
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
#include "mbsim/kinematic_excitation.h"
#include "mbsim/fixed_relative_frame.h"
#include "mbsim/rigid_body.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  KinematicExcitation::KinematicExcitation(const string &name) : RigidBodyLink(name), func(0) {
    body.resize(1);
    ratio.resize(1);
    ratio[0] = 1;
  }

  void KinematicExcitation::calclaSize(int j) {
    laSize = body[0]->getuRelSize();
  }
  void KinematicExcitation::calcgSize(int j) {
    gSize = body[0]->getuRelSize();
  }
  void KinematicExcitation::calcgdSize(int j) {
    gdSize = body[0]->getuRelSize();
  }

  void KinematicExcitation::updateGeneralizedForce(double t) {
    if(func)
      laSV = (*func)(getGeneralizedRelativePosition(t),getGeneralizedRelativeVelocity(t));
    else
      laSV = la;
    updlaSV = false;
  }

  void KinematicExcitation::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if (saved_DependentBody!="")
        setDependentBody(getByPath<RigidBody>(saved_DependentBody));
      RigidBodyLink::init(stage);
    } else
      RigidBodyLink::init(stage);
    if(func) func->init(stage);
  }

  void GeneralizedPositionExcitation::calcxSize() {
    if(!f) xSize = body[0]->getqRelSize();
  }

  void GeneralizedPositionExcitation::init(InitStage stage) {
    KinematicExcitation::init(stage);
    f->init(stage);
  }

  void GeneralizedPositionExcitation::updateGeneralizedPositions(double t) {
    rrel=body[0]->getqRel(t)-(*f)(t);
    updrrel = false;
  } 

  void GeneralizedPositionExcitation::updateGeneralizedVelocities(double t) {
    vrel=body[0]->getuRel(t)-f->parDer(t);
    updvrel = false;
  }

  void GeneralizedPositionExcitation::updatewb(double t) {
    wb += body[0]->getjRel(t)-f->parDerParDer(t);
  }

  void GeneralizedVelocityExcitation::calcxSize() {
    xSize = body[0]->getqRelSize();
  }

  void GeneralizedVelocityExcitation::init(InitStage stage) {
    KinematicExcitation::init(stage);
    f->init(stage);
  }

  void GeneralizedVelocityExcitation::updatexd(double t) {
    if(f) xd = (*f)(x,t);
  }

  void GeneralizedVelocityExcitation::updateGeneralizedPositions(double t) {
    rrel=body[0]->getqRel(t)-x;
    updrrel = false;
  } 

  void GeneralizedVelocityExcitation::updateGeneralizedVelocities(double t) {
    vrel=body[0]->getuRel(t)-(*f)(x,t);
    updvrel = false;
  }

  void GeneralizedVelocityExcitation::updatewb(double t) {
    wb += body[0]->getjRel(t)-(f->parDer1(x,t)*xd + f->parDer2(x,t));
  }

  void GeneralizedAccelerationExcitation::calcxSize() {
    xSize = body[0]->getqRelSize()+body[0]->getuRelSize();
  }

  void GeneralizedAccelerationExcitation::init(InitStage stage) {
    KinematicExcitation::init(stage);
    f->init(stage);
  }

  void GeneralizedAccelerationExcitation::updatexd(double t) {
    xd(0,body[0]->getqRelSize()-1) = x(body[0]->getqRelSize(),body[0]->getqRelSize()+body[0]->getuRelSize()-1);
    xd(body[0]->getqRelSize(),body[0]->getqRelSize()+body[0]->getuRelSize()-1) = (*f)(x,t);
  }

  void GeneralizedAccelerationExcitation::updateGeneralizedPositions(double t) {
    rrel=body[0]->getqRel(t)-x(0,body[0]->getqRelSize()-1);
    updrrel = false;
  }

  void GeneralizedAccelerationExcitation::updateGeneralizedVelocities(double t) {
    vrel=body[0]->getuRel(t)-x(body[0]->getqRelSize(),body[0]->getqRelSize()+body[0]->getuRelSize()-1);
    updvrel = false;
  }

  void GeneralizedAccelerationExcitation::updatewb(double t) {
    wb += (*f)(x,t);
  }

}


