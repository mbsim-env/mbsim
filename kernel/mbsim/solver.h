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
      Solver()  { }
      
      /**
       * \brief destructor
       */
      ~Solver() override = default;

#ifndef SWIG
      [[noreturn]]
#endif
      void throwError(const std::string &msg) const {
        throw MBSimError(this, msg);
      }
      
      /* INTERFACE FOR DERIVED CLASSES */
      /*! 
       * \brief start solving the dynamic system set by setSystem.
       */
      virtual void execute() = 0;

      virtual void initializeUsingXML(xercesc::DOMElement *element) {
        // set the XML location of this element which can be used, later, by exceptions.
        domEvalError=MBXMLUtils::DOMEvalException("", element);
      }

      virtual const fmatvec::Vec& getInitialState() const = 0;

      void setSystem(DynamicSystemSolver *s) { system=s; }
      MBSim::DynamicSystemSolver* getSystem() { return system; }

#ifndef SWIG
      const MBXMLUtils::DOMEvalException& getDOMEvalError() const { return domEvalError; };
#endif

    protected:
      /**
       * \brief dynamic system
       */
      MBSim::DynamicSystemSolver* system;

#ifndef SWIG
      //! Special XML helper variable.
      MBXMLUtils::DOMEvalException domEvalError;
#endif
  };

}

#endif /* _SOLVER_H_ */
