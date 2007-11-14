/* Copyright (C) 2004-2006  Roland Zander, Martin Förg
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *   rzander@users.berlios.de
 *
 */

#ifndef _MU_TIME_STEPPING_INTEGRATOR_H_ 
#define _MU_TIME_STEPPING_INTEGRATOR_H_

#include "fmatvec.h"
#include "integrator.h"

namespace MBSim {

  class ThetaTimeSteppingIntegrator : public Integrator { 

    private:

      double dt;
      double theta;
      bool driftCompensation;

    public:

      ThetaTimeSteppingIntegrator();
      ~ThetaTimeSteppingIntegrator() {}

      void setdt(double dt_) {dt = dt_;}
      void settheta(double theta_ ) {theta  = theta_;}
      void integrate(MultiBodySystem& system);
      void setDriftCompensation(bool dc) {driftCompensation = dc;}
  };

}

#endif
