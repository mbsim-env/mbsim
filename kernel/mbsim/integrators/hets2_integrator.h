/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#ifndef _HETS2_INTEGRATOR_H_ 
#define _HETS2_INTEGRATOR_H_

#include "integrator.h"

namespace MBSimIntegrator {

  /** 
   * \brief time integration scheme on velocity level for nonsmooth dynamical systems using half-explicit trapezoidal rule
   * \author Thorsten Schindler
   * \date 2014-09-12 initial commit (Thorsten Schindler)
   * \date 2014-09-17 notation of Brasey1994a (Thorsten Schindler)
   *
   * time discontinuous Galerkin method on velocity level using half-explicit trapezoidal rule
   *
   * T. Schindler, S. Rezaei, J. Kursawe, V. Acary : Half-explicit timestepping schemes 
   * on velocity level based on time-discontinuous Galerkin methods
   *
   * T. Schindler, V. Acary : Timestepping Schemes for Nonsmooth Dynamics Based
   * on Discontinuous Galerkin Methods: Definition and Outlook
   */
  class HETS2Integrator : public Integrator { 
    public:
      /**
       * \brief constructor
       */
      HETS2Integrator();
      
      /**
       * \brief destructor
       */
      virtual ~HETS2Integrator() {}

      /**
       * \brief initializes the integration
       * \param dynamical system to be integrated
       */
      void preIntegrate(MBSim::DynamicSystemSolver& system);
      
      /**
       * \brief does the integration
       * \param dynamical system to be integrated
       */
      void subIntegrate(MBSim::DynamicSystemSolver& system, double tStop);
      
      /**
       * \brief closes the integration
       * \param dynamical system to be integrated
       */
      void postIntegrate(MBSim::DynamicSystemSolver& system);

      /* INHERITED INTERFACE OF INTEGRATOR */
      virtual void integrate(MBSim::DynamicSystemSolver& system);
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setStepSize(double dt_) { dt = dt_; }
      /***************************************************/
    
    private:
      /**
       *  \brief evaluates the dynamical system given the state of the stage until the Jacobian matrices
       *  \param dynamical system to be integrated
       */
      void evaluateStage(MBSim::DynamicSystemSolver& system);

      /**
       * \brief step size
       */
      double dt;

      /**
       * \brief time and plot time
       */
      double t, tPlot;

      /**
       * \brief iteration counter for constraints, plots, integration, maximum constraints, cummulation constraint
       */
      int iter, step, integrationSteps, maxIter, sumIter;

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
  };

}

#endif /* _HETS2_INTEGRATOR_H_ */

