/* Copyright (C) 2004-2012 MBSim Development Team
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
 * Contact: jan.p.clauberg@googlemail.com
 */

#ifndef _THETA_TIME_STEPPING_SSC_INTEGRATOR_H_ 
#define _THETA_TIME_STEPPING_SSC_INTEGRATOR_H_

#include "integrator.h"

namespace MBSimIntegrator {

  /** 
   * brief theta-time-stepping integrator with step size adjustment
   * \author Jan Clauberg
   */
  class ThetaTimeSteppingSSCIntegrator : public Integrator { 
    public:
      /**
       * \brief constructor
       */
      ThetaTimeSteppingSSCIntegrator();

      /**
       * \brief destructor
       */
      virtual ~ThetaTimeSteppingSSCIntegrator() {}

      /* INHERITED INTERFACE OF INTEGRATOR */
      void integrate(MBSim::DynamicSystemSolver& system);
      virtual void initializeUsingXML(xercesc::DOMElement *element);
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
      void update(MBSim::DynamicSystemSolver& system, const fmatvec::Vec& z, double t);

      /**
       * \brief preintegration steps
       * \param dynamic system
       */
      void preIntegrate(MBSim::DynamicSystemSolver& system);
      
      /**
       * \brief integration steps
       * \param dynamic system
       * \param end time of integration
       */
      
      void subIntegrate(MBSim::DynamicSystemSolver& system, double tStop);
      /**
       * \brief postintegration steps
       * \param dynamic system
       */
      void postIntegrate(MBSim::DynamicSystemSolver& system);

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

#endif /* _THETA_TIME_STEPPING_SSC_INTEGRATOR_H_ */

