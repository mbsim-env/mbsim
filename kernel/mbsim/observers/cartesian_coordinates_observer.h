/* Copyright (C) 2004-2016 MBSim Development Team
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

#ifndef _CARTESIAN_COORDINATES_OBSERVER_H__
#define _CARTESIAN_COORDINATES_OBSERVER_H__

#include "mbsim/observers/coordinates_observer.h"

namespace MBSim {

  class CartesianCoordinatesObserver : public CoordinatesObserver {
    private:
      fmatvec::Vec3 ex, ey, ez;
      fmatvec::SqrMat3 A;
#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::Arrow> openMBVXPosition, openMBVYPosition, openMBVZPosition, openMBVXVelocity, openMBVYVelocity, openMBVZVelocity, openMBVXAcceleration, openMBVYAcceleration, openMBVZAcceleration; 
#endif

    public:
      CartesianCoordinatesObserver(const std::string &name="");

      void init(InitStage stage);
      virtual void plot();
  };

}

#endif
