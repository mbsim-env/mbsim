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

namespace MBSim {

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
       */
      MBSimError(const std::string &mbsim_error_message_) throw(); 
      
      virtual ~MBSimError() throw() {}

      virtual const char* what() const throw();

    private:
      /**
       * \brief error message
       */
      std::string mbsim_error_message;

  };
}

#endif /* _MBSIM_EVENT_H */

