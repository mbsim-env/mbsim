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

namespace MBSim {

  /**
   * \brief basic event class for MBSim
   * \author Thorsten Schindler
   * \date 2009-03-20 inital commit (Thorsten Schindler)
   */
  class MBSimEvent {
    public:
      /**
       * \brief constructor
       */
      MBSimEvent() {}
      
      /**
       * \brief destructor
       */
      virtual ~MBSimEvent() {}
  };

  /**
   * \brief basic exception interface for mbsim
   * \author Thorsten Schindler
   * \date 2009-03-20 inital commit (Thorsten Schindler)
   */
  class MBSimException : public MBSimEvent{
    public:
      /**
       * \brief constructor
       */
      MBSimException() : MBSimEvent() {}
      
      /**
       * \brief destructor
       */
      virtual ~MBSimException() {}

      /* INTERFACE */
      /**
       * \brief prints exception message on stdout
       */
      virtual void printExceptionMessage() = 0; 
  };

  /**
   * \brief basic error class for mbsim
   * \author Thorsten Schindler
   * \date 2009-03-20 inital commit (Thorsten Schindler)
   */
  class MBSimError : public MBSimException {
    public:
      /**
       * \brief constructor
       * \param message to be written
       */
      MBSimError(const std::string &mbsim_error_message_); 
      
      /**
       * \brief destructor
       */
      virtual ~MBSimError() {}

      /* INHERITED INTERFACE */
      virtual void printExceptionMessage();

    private:
      /**
       * \brief error message
       */
      std::string mbsim_error_message;

  };
}

#endif /* _MBSIM_EVENT_H */

