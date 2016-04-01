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
#include "mbsim/links/kinematic_excitation.h"
#include "mbsim/objects/rigid_body.h"

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

  void KinematicExcitation::updateGeneralizedForces() {
    if(func)
      lambda = (*func)(evalGeneralizedRelativePosition(),evalGeneralizedRelativeVelocity());
    else
      lambda = la;
    updla = false;
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

}
