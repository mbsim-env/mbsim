/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 *          rzander@users.berlios.de
 */

#ifndef _MU_TIME_STEPPING_INTEGRATOR_H_ 
#define _MU_TIME_STEPPING_INTEGRATOR_H_

#include "fmatvec.h"
#include "mbsim/integrators/integrator.h"

namespace MBSim {

  /** 
   * brief theta-time-stepping integrator of first order
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-07-18 new kernel_dev (Thorsten Schindler)
   * \date 2009-07-19 Delassus matrix / split of update (Thorsten Schindler)
   * \date 2009-07-27 some fixes (Thorsten Schindler)
   * \date 2010-04-23 integrate splitted (Thorsten Schindler)
   */
  class ThetaTimeSteppingIntegrator : public Integrator { 
    public:
      /**
       * \brief constructor
       */
      ThetaTimeSteppingIntegrator();

      /**
       * \brief destructor
       */
      virtual ~ThetaTimeSteppingIntegrator() {}

      /* INHERITED INTERFACE OF INTEGRATOR */
      void integrate(DynamicSystemSolver& system);
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setStepSize(double dt_) { dt = dt_; }
      void setTheta(double theta_ ) { theta  = theta_; }
      void setDriftCompensation(bool dc) { driftCompensation = dc; }
      /***************************************************/

      /**
       * \brief update of dynamic system necessary values concerning theta time stepping integrator
       * \param dynamic system
       * \param state vector
       * \param time
       */
      void update(DynamicSystemSolver& system, const fmatvec::Vec& z, double t);

      /**
       * \brief preintegration steps
       * \param dynamic system
       */
      void preIntegrate(DynamicSystemSolver& system);
      
      /**
       * \brief integration steps
       * \param dynamic system
       * \param end time of integration
       */
      
      void subIntegrate(DynamicSystemSolver& system, double tStop);
      /**
       * \brief postintegration steps
       * \param dynamic system
       */
      void postIntegrate(DynamicSystemSolver& system);

    private:
      /**
       * \brief step size
       */
      double dt;

      /**
       * \brief convex combination parameter between explicit (0) and implicit (1) Euler scheme
       */
      double theta;

      /**
       * \brief time and plot time
       */
      double t, tPlot;

      /**
       * \brief iteration counter for constraints, plots, integration, maximum constraints, cummulation constraint
       */
      int iter,step, integrationSteps, maxIter, sumIter;

      /**
       * \brief computing time counter
       */
      double s0, time;

      /**
       * \brief plot step difference
       */
      int stepPlot;

      /**
       * \brief state, position, velocity, order coordinate of dynamical system
       */
      fmatvec::Vec z, q, u, x;

      /**
       * \brief file stream for integration information
       */
      std::ofstream integPlot;

      /**
       * \brief flag for drift compensation
       */
      bool driftCompensation;    
  };

}

#endif /* _MU_TIME_STEPPING_INTEGRATOR_H_ */

