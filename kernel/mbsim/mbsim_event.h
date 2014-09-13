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
 * Contact: thschindler@users.berlios.de
 */

#ifndef _MBSIM_EVENT_H_
#define _MBSIM_EVENT_H_

#include<string>
#include<exception>
#include <xercesc/dom/DOMElement.hpp>

namespace MBSim {

  class Element;

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
       * \param context_ the conext MBSim::Element where the error occured
       */
      MBSimError(const Element *context_, const std::string &mbsim_error_message_) throw(); 

      /**
       * \brief constructor
       * ctor variant without a context
       */
      MBSimError(const std::string &mbsim_error_message_) throw(); 
      
      virtual ~MBSimError() throw() {}

      /* \brief set the context of the error
       * Use this function to set the context in a catch(...) block if the context is not known
       * at the original throw statement. */
      void setContext(const Element *context_);

      virtual const char* what() const throw();

    private:
      /**
       * \brief error message
       */
      std::string mbsim_error_message;

      // just a string to store the memory which is returned by the what() function
      std::string whatMsg;
  };

  // Helper to throw a error with this as the context of the error
  #define THROW_MBSIMERROR(msg) \
    throw MBSim::MBSimError(this, msg)
}

#endif /* _MBSIM_EVENT_H */

