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

#ifndef _SOLVER_H_
#define _SOLVER_H_

#include <fmatvec/fmatvec.h>
#include <fmatvec/atom.h>
#include <mbxmlutilshelper/dom.h>
#include <mbsim/mbsim_event.h>
#include <string>
#include <iostream>

namespace MBSim {

  class DynamicSystemSolver;

  /**
   * \brief solver-interface for dynamic systems
   * \author Martin Foerg
   */
  class Solver : virtual public fmatvec::Atom {
    public:
      /**
       * \brief constructor 
       */
      Solver() : fmatvec::Atom() { }
      
      /**
       * \brief destructor
       */
      ~Solver() override = default;
      
      /* INTERFACE FOR DERIVED CLASSES */
      /*! 
       * \brief start solving the dynamic system
       * \param dynamic system to be solved
       */
      virtual void execute(DynamicSystemSolver& system) = 0;

      virtual void initializeUsingXML(xercesc::DOMElement *element) = 0;

      virtual const fmatvec::Vec& getInitialState() const = 0;

    protected:
      /**
       * \brief dynamic system
       */
      static DynamicSystemSolver* system;
  };

}

#endif /* _SOLVER_H_ */
