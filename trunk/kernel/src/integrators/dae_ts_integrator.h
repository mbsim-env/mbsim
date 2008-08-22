/* Copyright (C) 2004-2007  Robert Huber

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
 *   rhuber@users.berlios.de
 *
 */

#ifndef DAE_TS_INTEGRATOR_H_ 
#define DAE_TS_INTEGRATOR_H_

#include<fmatvec.h>
#include "integrator.h"
//#include "integrators.h"
//#include "daskr_integrator.h"
//#include "time_stepping_ssc_integrator.h"

namespace MBSim {

  class TimeSteppingSSCIntegrator;
  class DASKRIntegrator;

  /*! Event driven DAE and TimeStepping Integration  (DAE solver: DASKR)*/
  class DAETSIntegrator : public Integrator { 
    protected :
      double s0;
      bool outputRoots;
 
    public :  
      TimeSteppingSSCIntegrator *TSIntegrator;
      DASKRIntegrator *DAEIntegrator;

      DAETSIntegrator();
      ~DAETSIntegrator();

      void settEnd(double tEnd_);
      void settStart(double tStart_);
      void setdtPlot(double dtPlot_);
      void setz0(const fmatvec::Vec &z0_);
      void setOutput(bool flag);

      void setaTol(const fmatvec::Vec &aTol_); 
      void setaTol(double aTol_);
      void setrTol(const fmatvec::Vec &rTol_);
      void setrTol(double rTol_);

      void integrate(MultiBodySystem& system);

      /** subroutines for integrate function */
      void initIntegrator(MultiBodySystem &system_);
      void IntegrationStep();
      void closeIntegrator();

  };

}

#endif 
