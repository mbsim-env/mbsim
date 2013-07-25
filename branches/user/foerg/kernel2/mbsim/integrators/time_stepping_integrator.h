/* Copyright (C) 2004-2010 MBSim Development Team
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
 */

#ifndef _TIME_STEPPING_INTEGRATOR_H_ 
#define _TIME_STEPPING_INTEGRATOR_H_

#include<fmatvec.h>
#include "integrator.h"

namespace MBSim {

  /** 
   * brief half-explicit time-stepping integrator of first order
   * \author Martin Foerg
   * \date 2009-07-13 some comments (Thorsten Schindler)
   */
  class TimeSteppingIntegrator : public Integrator { 
    public:
      /**
       * \brief constructor
       */
      TimeSteppingIntegrator();
      
      /**
       * \brief destructor
       */
      virtual ~TimeSteppingIntegrator() {}

      void preIntegrate(DynamicSystemSolver& system);
      void subIntegrate(DynamicSystemSolver& system, double tStop);
      void postIntegrate(DynamicSystemSolver& system);

      /* INHERITED INTERFACE OF INTEGRATOR */
      virtual void integrate(DynamicSystemSolver& system);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setStepSize(double dt_) { dt = dt_; }
      void setDriftCompensation(bool dc) { driftCompensation = dc; }
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

#endif /* _TIME_STEPPING_INTEGRATOR_H_ */

