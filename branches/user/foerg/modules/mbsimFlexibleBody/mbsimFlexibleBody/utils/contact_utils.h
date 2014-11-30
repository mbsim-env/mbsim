/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: markus.ms.schneider@googlemail.com
 */

#ifndef _MBSIMFLEXIBLEBODY_CONTACT_UTILS_H_
#define _MBSIMFLEXIBLEBODY_CONTACT_UTILS_H_

namespace MBSim {
  class ContactKinematics;
}

namespace MBSimFlexibleBody {

  /**
   * \brief defines contact kinematics between one rigid and one flexible contour
   * \param first contour
   * \param second contour
   * \return contact kinematics
   * \author Markus Schneider
   * \date 2010-11-05 initial commit (Markus Schneider)
   */
  MBSim::ContactKinematics* findContactPairingFlexible(const char *contour0, const char *contour1);

}

#endif /* _MBSIMFLEXIBLEBODY_CONTACT_UTILS_H_ */

