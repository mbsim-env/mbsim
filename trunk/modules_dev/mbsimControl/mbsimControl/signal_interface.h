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
 * Contact: schneidm@users.berlios.de
 */

#ifndef _SIGNAL_INTERFACE_H_
#define _SIGNAL_INTERFACE_H_

#include "fmatvec.h"

namespace MBSimControl {

  /*!
   * Interface for getting values out of objets, links and extra dynamic interfaces
   * \brief signal interface
   * \author Markus Schneider, 19.08.2009
   */
  class SignalInterface {
    public:
      /*!
       * \brief constructor 
       */
      SignalInterface() {}

      /*!
       * \brief destructor
       */
      virtual ~SignalInterface() {}

      /*!
       * \brief get the signal()
       */
      virtual fmatvec::Vec getSignal() = 0;

  };

}

#endif /* _SIGNAL_INTERFACE_H_ */

