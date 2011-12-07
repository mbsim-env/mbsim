/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: thschindler@users.berlios.de
 */

#ifndef _D1MINUSLINEAR_H_ 
#define _D1MINUSLINEAR_H_

#include<fmatvec.h>
#include "integrator.h"

namespace MBSim {

  /** 
   * \brief time-integrator for dynamical systems using D1MinusLinear ansatz
   * \author Thorsten Schindler
   * \date 2011-10-11 initial commit (Thorsten Schindler)
   *
   * Discontinuous Galerkin method with piecewise linear ansatz functions for velocity based on
   * T. Schindler, V. Acary : Timestepping Schemes for Nonsmooth Dynamics Based
   * on Discontinuous Galerkin Methods: Definition and Outlook
   */
  class TimeSteppingD1MinusLinearIntegrator : public Integrator { 
    public:
      /**
       * \brief constructor
       */
      TimeSteppingD1MinusLinearIntegrator();
      
      /**
       * \brief destructor
       */
      virtual ~TimeSteppingD1MinusLinearIntegrator() {}

      void preIntegrate(DynamicSystemSolver& system);
      void subIntegrate(DynamicSystemSolver& system, double tStop);
      void postIntegrate(DynamicSystemSolver& system);

      /* INHERITED INTERFACE OF INTEGRATOR */
      virtual void integrate(DynamicSystemSolver& system);
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setStepSize(double dt_) { dt = dt_; }
      /***************************************************/
    
    private:
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

      TimeSteppingD1MinusLinearIntegrator(const TimeSteppingD1MinusLinearIntegrator&); // copy constructor
      TimeSteppingD1MinusLinearIntegrator& operator=(const TimeSteppingD1MinusLinearIntegrator&); // assignment operator
  };

}

#endif /* _TIME_STEPPING_INTEGRATOR_H_ */

