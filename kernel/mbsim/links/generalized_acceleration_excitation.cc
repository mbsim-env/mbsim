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
#include "mbsim/links/generalized_acceleration_excitation.h"
#include "mbsim/objects/rigid_body.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  void GeneralizedAccelerationExcitation::calcxSize() {
    xSize = body[0]->getGeneralizedPositionSize()+body[0]->getGeneralizedVelocitySize();
  }

  void GeneralizedAccelerationExcitation::init(InitStage stage) {
    GeneralizedKinematicExcitation::init(stage);
    f->init(stage);
  }

  void GeneralizedAccelerationExcitation::updatexd() {
    xd(0,body[0]->getGeneralizedPositionSize()-1) = x(body[0]->getGeneralizedPositionSize(),body[0]->getGeneralizedPositionSize()+body[0]->getGeneralizedVelocitySize()-1);
    xd(body[0]->getGeneralizedPositionSize(),body[0]->getGeneralizedPositionSize()+body[0]->getGeneralizedVelocitySize()-1) = (*f)(x,getTime());
  }

  void GeneralizedAccelerationExcitation::updateGeneralizedPositions() {
    if(body.size()>1)
      rrel=body[1]->evalGeneralizedPosition()-body[0]->evalGeneralizedPosition()-x(0,body[0]->getGeneralizedPositionSize()-1);
    else
      rrel=body[0]->evalGeneralizedPosition()-x(0,body[0]->getGeneralizedPositionSize()-1);
    updrrel = false;
  }

  void GeneralizedAccelerationExcitation::updateGeneralizedVelocities() {
    if(body.size()>1)
      vrel=body[1]->evalGeneralizedVelocity()-body[0]->evalGeneralizedVelocity()-x(body[0]->getGeneralizedPositionSize(),body[0]->getGeneralizedPositionSize()+body[0]->getGeneralizedVelocitySize()-1);
    else
      vrel=body[0]->evalGeneralizedVelocity()-x(body[0]->getGeneralizedPositionSize(),body[0]->getGeneralizedPositionSize()+body[0]->getGeneralizedVelocitySize()-1);
    updvrel = false;
  }

  void GeneralizedAccelerationExcitation::updatewb() {
    wb += (*f)(x,getTime());
  }

}
