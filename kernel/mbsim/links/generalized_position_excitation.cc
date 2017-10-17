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
    if(!f) xSize = body[0]->getGeneralizedPositionSize();
  }

  void GeneralizedPositionExcitation::init(InitStage stage, const InitConfigSet &config) {
    GeneralizedKinematicExcitation::init(stage, config);
    f->init(stage, config);
  }

  void GeneralizedPositionExcitation::updateGeneralizedPositions() {
    if(body.size()>1)
      rrel=body[1]->evalGeneralizedPosition()-body[0]->evalGeneralizedPosition()-(*f)(getTime());
    else
      rrel=body[0]->evalGeneralizedPosition()-(*f)(getTime());
    updrrel = false;
  } 

  void GeneralizedPositionExcitation::updateGeneralizedVelocities() {
    if(body.size()>1)
      vrel=body[1]->evalGeneralizedVelocity()-body[0]->evalGeneralizedVelocity()-f->parDer(getTime());
    else
      vrel=body[0]->evalGeneralizedVelocity()-f->parDer(getTime());
    updvrel = false;
  }

  void GeneralizedPositionExcitation::updatewb() {
    if(body.size()>1)
      wb += body[1]->evaljRel()-body[0]->evaljRel()-f->parDerParDer(getTime());
    else
      wb += body[0]->evaljRel()-f->parDerParDer(getTime());
  }

}
