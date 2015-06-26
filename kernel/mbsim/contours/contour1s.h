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

#ifndef _CONTOUR1S_H_
#define _CONTOUR1S_H_

#include "mbsim/contours/contour_continuum.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#endif

namespace MBSim {

  /** 
   * \brief basic class for contours described by one contour parameter \f$s\f$
   * \author Roland Zander
   * \date 2009-04-20 frame-concept (Thorsten Schindler)
   * \date 2009-06-04 new file (Thorsten Schindler)
   */
  class Contour1s : public ContourContinuum<double> {
    public:
      /**
       * \brief constructor 
       * \param name of contour
       */
      Contour1s(const std::string &name);

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const {
        return "Contour1s";
      }
      void init(InitStage stage_);
      void plot(double t, double dt);
      /***************************************************/

      virtual fmatvec::Vec3 getRelativePosition(ContourPointData &cp) {
        THROW_MBSIMERROR("(Contour::getRelativePosition): Not implemented.");
        return 0;
      }

      virtual fmatvec::Vec3 getDerivativeOfRelativePosition(ContourPointData &cp) {
        THROW_MBSIMERROR("(Contour::getDerivativeOfRelativePosition): Not implemented.");
        return 0;
      }

      virtual fmatvec::Vec3 getSecondDerivativeOfRelativePosition(ContourPointData &cp) {
        THROW_MBSIMERROR("(Contour::getSecondDerivativeOfRelativePosition): Not implemented.");
        return 0;
      }

      /**
       * \return position in world frame
       * \param contour position
       */
     virtual fmatvec::Vec3 getPosition(double t, ContourPointData &cp);

      /**
       * \return first tangent in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getFirstTangent(double t, ContourPointData &cp);

      /**
       * \return second tangent in world frame
       * \param Lagrangian position
       */
      virtual fmatvec::Vec3 getSecondTangent(double t, ContourPointData &cp);

      /**
       * \return normal in world frame
       * \param contour position
       */
      virtual fmatvec::Vec3 getNormal(double t, ContourPointData &cp);

      /**
       * \return derivative of first tangent in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getDerivativeOfFirstTangent(double t, ContourPointData &cp);

      /**
       * \return derivative of first tangent in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getGlobalDerivativeOfRelativePosition(double t, ContourPointData &cp);

      /**
       * \return radius of contour in contour point
       * \param contour position
       */
      virtual double getCurvature(ContourPointData &cp) {
        THROW_MBSIMERROR("(Contour::getCurvature): Not implemented.");
        return 0;
      }

      /* GETTER / SETTER */
      void setDiameter(double diameter_) {
        diameter = diameter_;
      }
      double getDiameter() {
        return diameter;
      }
      /***************************************************/


#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpineExtrusion(const boost::shared_ptr<OpenMBV::SpineExtrusion> &spineExtrusion) {
        openMBVSpineExtrusion = spineExtrusion;
      }
      boost::shared_ptr<OpenMBV::SpineExtrusion>& getOpenMBVSpineExtrusion() {
        return openMBVSpineExtrusion;
      }
#endif

    protected:
      /**
       * \brief diameter of neutral fibre
       *
       * \todo: this is not really general --> diameter only valid for round neutral contours!
       */
      double diameter;

#ifdef HAVE_OPENMBVCPPINTERFACE
      /*!
       * \brief body for the spine extrusion for visualisation of the 1s-body
       */
      boost::shared_ptr<OpenMBV::SpineExtrusion> openMBVSpineExtrusion;

#endif

  };

}

#endif /* _CONTOUR1S_H_ */

