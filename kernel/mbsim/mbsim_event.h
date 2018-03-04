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
 */

#ifndef _MBSIM_EVENT_H_
#define _MBSIM_EVENT_H_

#include <string>
#include <exception>
#include <xercesc/dom/DOMElement.hpp>
#include <mbxmlutilshelper/dom.h>

namespace MBSim {

  class Element;
  class Solver;

  /**
   * \brief basic error class for mbsim
   * \author Thorsten Schindler
   * \date 2009-03-20 inital commit (Thorsten Schindler)
   * \date 2013-05-15 changed base class to std::exception (Martin Foerg)
   */
  class MBSimError : public std::exception {
    public:
      /**
       * \brief constructor
       * \param message to be written
       * \param context the conext MBSim::Element where the error occured
       */
      MBSimError(const Element *context, std::string mbsim_error_message_) noexcept; 

      /**
       * \brief constructor
       * \param message to be written
       * \param context the conext MBSim::Solver where the error occured
       */
      MBSimError(const Solver *context, std::string mbsim_error_message_) noexcept; 

      /**
       * \brief constructor
       * ctor variant without a context
       */
      MBSimError(std::string mbsim_error_message_) noexcept; 
      
      ~MBSimError() noexcept override = default;

      const std::string& getErrorMessage() const { return mbsim_error_message; }

      const std::string& getPath() const { return path; }

      const char* what() const noexcept override;

    private:
      /**
       * \brief error message
       */
      std::string mbsim_error_message;

      std::string path;

      // domEvalError stores the DOM stack of this error, if available.
      // It is used in what() to generated the message for whatMsg.
      mutable MBXMLUtils::DOMEvalException domEvalError;

      // just a string to store the memory which is returned by the what() function
      mutable std::string whatMsg;
  };
}

#endif /* _MBSIM_EVENT_H */

