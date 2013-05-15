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

#include <config.h>
#include <mbsim/mbsim_event.h>
#include <mbsim/element.h>
#include <iostream>

using namespace std;

namespace MBSim {
  
  MBSimError::MBSimError(const std::string &mbsim_error_message_) : MBSimException(), mbsim_error_message(mbsim_error_message_) {}

  void MBSimError::printExceptionMessage() {
    cerr << endl;
    cerr << endl;
    cerr << "A MBSimError was detected: " << mbsim_error_message << endl;
    cerr << endl;
    cerr << endl;
  }

}

