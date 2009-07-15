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
 *          besefeld@users.berlios.de
 */

#ifndef CIRCLE_FRUSTUM_H_
#define CIRCLE_FRUSTUM_H_

#include "mbsim/contact_kinematics/contact_kinematics.h"

namespace MBSim {

  class Circle;
  class Frustum;

  /*!
   * \brief contact kinematics for unilateral contact between circle and frustum with at most one contact point
   * \author Thorsten Schindler
   * \author Bastian Esefeld
   * \date 2009-07-13 initial commit kernel_dev
   */
  class ContactKinematicsCircleFrustum : public ContactKinematics {
    public:
      /*! 
       * \brief constructor 
       * \default no debugging
       * \default no warnings
       * \default global search 
       */
      ContactKinematicsCircleFrustum();

      /*! 
       * \brief destructor
       */
      virtual ~ContactKinematicsCircleFrustum();

      /* INHERITED INTERFACE OF CONTACTKINEAMTICS */
      void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec& g, ContourPointData *cpData);   
      virtual void updatewb(fmatvec::Vec& wb, const fmatvec::Vec &g,ContourPointData *cpData) { throw new MBSimError("ERROR (PointFlexibleBand::updateg): not implemented!"); }   
      /***************************************************/

      /* GETTER / SETTER */
      void setDebug(bool DEBUG_);
      void setWarnLevel(int warnLevel_);
      void setLocalSearch(bool LOCALSEARCH_);
      /***************************************************/

    private:
      /** 
       * \brief contour index
       */
      int icircle, ifrustum;

      /** 
       * \brief contour classes 
       */
      Frustum *frustum; 
      Circle *circle;

      /** 
       * \brief debug information during calculation? 
       */
      bool DEBUG;

      /**
       * \brief warnings during calculation?
       */
      int warnLevel;

      /**
       * \brief local contact search?
       */
      bool LOCALSEARCH;
  };

  inline void ContactKinematicsCircleFrustum::setDebug(bool DEBUG_) { DEBUG=DEBUG_; }
  inline void ContactKinematicsCircleFrustum::setWarnLevel(int warnLevel_) { warnLevel=warnLevel_; }
  inline void ContactKinematicsCircleFrustum::setLocalSearch(bool LOCALSEARCH_) { LOCALSEARCH=LOCALSEARCH_; }

}

#endif /* CIRCLE_FRUSTUM_H_ */

