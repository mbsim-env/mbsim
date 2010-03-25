/* Copyright (C) 2004-2010 MBSim Development Team
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

#ifndef NURBS_DISK_2S_MFR_MINDLIN_H_
#define NURBS_DISK_2S_MFR_MINDLIN_H_

#include "mbsim/contours/nurbs_disk_2s.h"

namespace MBSim {

  /*!  
   * \brief 2s flexible
   * \author Kilian Grundl
   * \date 2010-03-23 initial commit (Grundl)
   * \date 2010-03-25 small adaptations (Schindler)
   * \todo computeSurfaceJacobians / computeSurfaceVelocities only in contact case TODO
   * \todo angularVelocity TODO
   * \todo flexible body should only parametrise midplane -> other surfaces in contour TODO
   */
  class NurbsDisk2sMFRMindlin : public NurbsDisk2s {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      NurbsDisk2sMFRMindlin(const std::string &name): NurbsDisk2s(name) {}

      /**
       * \brief destructor
       */
      virtual ~NurbsDisk2sMFRMindlin() {}

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "NurbsDisk2sMFRMindlin"; }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff);
      virtual void updateJacobiansForFrame(ContourPointData &cp);
      /***************************************************/

#ifdef HAVE_NURBS
      /**
       * \brief initialize NURBS disk
       * \param stage of initialisation
       */
      void initContourFromBody(InitStage stage);
#endif

      /*!
       * \brief transformation cartesian to cylinder system
       * \param cartesian vector in world system
       * \return cylindrical coordinates
       */
      fmatvec::Vec transformCW(const fmatvec::Vec& WrPoint);

#ifdef HAVE_NURBS
      /*!
       * \brief interpolates the surface with node-data from body
       */
      void computeSurface();
#endif

#ifdef HAVE_NURBS
      /*!
       * \brief interpolates the velocities of the surface with the node-data from the body
       */
      void computeSurfaceVelocities();
#endif

#ifdef HAVE_NURBS
      /*!
       * \brief interpolates the Jacobians of translation of the surface with the node-data from the body
       */
      void computeSurfaceJacobiansOfTranslation();
#endif
  };

}

#endif /* NURBS_DISK_2S_MFR_MINDLIN_H_ */

