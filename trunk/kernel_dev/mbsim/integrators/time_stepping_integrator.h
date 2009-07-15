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
 * Contact: mfoerg@users.berlios.de
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

      /* INHERITED INTERFACE OF INTEGRATOR */
      virtual void integrate(DynamicSystemSolver& system);
      virtual void initializeUsingXML(TiXmlElement *element);
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
       * \brief flag for drift compensation
       */
      bool driftCompensation;
  };

}

#endif /* _TIME_STEPPING_INTEGRATOR_H_ */

