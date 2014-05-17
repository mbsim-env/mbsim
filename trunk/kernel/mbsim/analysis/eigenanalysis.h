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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _EIGENANALYSIS_H_
#define _EIGENANALYSIS_H_

#include "fmatvec/fmatvec.h"

namespace MBSim {

  class DynamicSystemSolver;

  /**
   * \brief Eigenanalysis for dynamic systems
   * \author Martin Foerg
   */
  class Eigenanalysis {

    public:
      /**
       * \brief Standard constructor 
       */
      Eigenanalysis() {}
      
      /**
       * \brief Destructor
       */
      ~Eigenanalysis() {}

      /**
       * \brief Perform the eigenanalysis
       * \param system The dynamic system to be analysed
       */
      void analyse(DynamicSystemSolver& system);

      /**
       * \brief Set the initial state of the dynamic system
       * \param z0_ The initial state
       */
      void setInitialState(const fmatvec::Vec &z0_) { z0 = z0_; }

    protected:

      static DynamicSystemSolver* system;

      fmatvec::Vec z0;
 };

}

#endif
