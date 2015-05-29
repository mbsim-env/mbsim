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
    protected:
     /**
       * \brief the lagrange parameters in U and V direction for the contact2ssearch
       */
      fmatvec::Vec nodesU, nodesV;

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
      virtual fmatvec::Mat3x2 computeTangentialPlane(double t, fmatvec::Vec alpha) { ContourPointData cp(alpha); return cp.getFrameOfReference().getOrientation(t)(fmatvec::Range<fmatvec::Fixed<0>,fmatvec::Fixed<2> >(),fmatvec::Range<fmatvec::Fixed<1>,fmatvec::Fixed<2> >()); }

      /**
       * \return nodes lagrange parameters for contact2sSearch in U direction
       */
      const fmatvec::Vec getNodesU() const { return nodesU; }

      /**
       * \return nodes lagrange parameters for contact2sSearch in V direction
       */
      const fmatvec::Vec getNodesV() const { return nodesU; }

      /**
       * \set nodes lagrange parameters for contact2sSearch in U direction
       */
      void setNodesU(const fmatvec::Vec& nodesU_) { nodesU = nodesU_; }

      /**
       * \set nodes lagrange parameters for contact2sSearch in V direction
       */
      void setNodesV(const fmatvec::Vec& nodesV_) { nodesV = nodesV_; }
  };

}

#endif /* _CONTOUR2S_H_ */

