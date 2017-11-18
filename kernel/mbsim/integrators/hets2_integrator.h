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
   * \date 2014-09-18 first final status with contact and impact with the slider-crank mechanism (Thorsten Schindler)
   * \date 2014-09-22 naive step-size adaptation according experience with the error constants, not square root relationship (Thorsten Schindler)
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
      ~HETS2Integrator() override = default;

      /**
       * \brief initializes the integration
       * \param dynamical system to be integrated
       */
      void preIntegrate(MBSim::DynamicSystemSolver& system) override;

      /**
       * \brief does the integration
       * \param dynamical system to be integrated
       */
      void subIntegrate(MBSim::DynamicSystemSolver& system, double tStop) override;

      /**
       * \brief closes the integration
       * \param dynamical system to be integrated
       */
      void postIntegrate(MBSim::DynamicSystemSolver& system) override;

      /* INHERITED INTERFACE OF INTEGRATOR */
      void integrate(MBSim::DynamicSystemSolver& system) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setStepSize(double dt_) { dt = dt_; dtImpulsive = dt*1e-1, dtInfo = dt; }
      /***************************************************/

      void updatebc();
      void updatezd();

    private:
      /**
       *  \brief evaluates the dynamical system given the state of the stage until the Jacobian matrices
       *  \param dynamical system to be integrated
       *  return flag for occuring impact
       */
      bool evaluateStage(MBSim::DynamicSystemSolver& system);

      /**
       * \brief step size for non-impulsive periods, and impulsive periods, and last used
       */
      double dt{1e-3}, dtImpulsive{1e-4}, dtInfo{1e-3};

      /**
       * \brief time and plot time
       */
      double tPlot{0.};

      /**
       * \brief iteration counter for constraints, integration, non-impulsive integration, impulsive integration, maximum constraints, cummulation constraint
       */
      int integrationSteps{0}, integrationStepsConstraint{0}, integrationStepsImpact{0}, maxIter{0}, sumIter{0};

      /**
       * \brief computing time counter
       */
      double s0{0.}, time{0.};

      /**
       * \brief file stream for integration information
       */
      std::ofstream integPlot;

      fmatvec::Vec bc;
  };

}

#endif /* _HETS2_INTEGRATOR_H_ */

