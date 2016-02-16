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
#include "mbsim/links/generalized_position_excitation.h"
#include "mbsim/objects/rigid_body.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

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

}
