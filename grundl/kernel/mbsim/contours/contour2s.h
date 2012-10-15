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

#ifndef _CONTOUR2S_H_
#define _CONTOUR2S_H_

#include "mbsim/contours/contour_continuum.h"

namespace MBSim {

  /**
   * \brief basic contour described by two contour parameters \f$\vs\f$
   * \author Roland Zander
   * \date 2009-04-20 frame concept (Thorsten Schindler)
   * \date 2009-06-04 new file (Thorsten Schindler)
   */
  class Contour2s : public ContourContinuum<fmatvec::Vec> {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour2s(const std::string &name) : ContourContinuum<fmatvec::Vec>(name) {}

      /**
       * \return tangents in world frame
       * \param Lagrangian position
       */
      virtual fmatvec::Mat32 computeTangentialPlane(fmatvec::Vec alpha) { ContourPointData cp(alpha); updateKinematicsForFrame(cp,cosy); return cp.getFrameOfReference().getOrientation()(fmatvec::Range<fmatvec::Fixed<0>,fmatvec::Fixed<2> >(),fmatvec::Range<fmatvec::Fixed<1>,fmatvec::Fixed<2> >()); }
  };

}

#endif /* _CONTOUR2S_H_ */

