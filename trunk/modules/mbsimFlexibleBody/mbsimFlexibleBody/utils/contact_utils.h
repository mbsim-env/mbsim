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
 *          rzander@users.berlios.de
 */

#ifndef _CONTACT_UTILS_FLEXIBLE_H_
#define _CONTACT_UTILS_FLEXIBLE_H_

namespace MBSim {
  class ContactKinematics;
}

namespace MBSimFlexibleBody {

  /**
   * \brief defines contact kinematics between two contours with one flexible
   * \author Markus Schneider
   * \date 2010-11-05 initialCommit (Markus Schneider)
   */
  MBSim::ContactKinematics* findContactPairingFlexible(const char *contour0, const char *contour1);

}

#endif /* _CONTACT_UTILS_FLEXIBLE_H_ */

