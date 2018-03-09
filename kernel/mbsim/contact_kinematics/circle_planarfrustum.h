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
 */

#ifndef _CONTACT_KINEMATICS_CIRCLE_PLANAR_FRUSTUM_H_
#define _CONTACT_KINEMATICS_CIRCLE_PLANAR_FRUSTUM_H_

#include "contact_kinematics.h"
#include "mbsim/mbsim_event.h"

namespace MBSim {

  class Circle;
  class PlanarFrustum;

  /** 
   * \brief pairing circle outer side to planar frustum
   * \author Martin Foerg
   * \date 2009-04-02 some comments (Thorsten Schindler)
   * \date 2009-05-27 updateg() implementes (Bastian Esefeld)
   * \todo implementation of updatewb() TODO
   */
  class ContactKinematicsCirclePlanarFrustum : public ContactKinematics {
    public:
      /* INHERITED INTERFACE */
      void assignContours(const std::vector<Contour*> &contour) override;
      void updateg(double &g, std::vector<ContourFrame*> &cFrame, int index = 0) override;
      void updatewb(fmatvec::Vec &wb, double g, std::vector<ContourFrame*> &cFrame) override { throw std::runtime_error("(ContactKinematicsCirclePlanarFrustum:updatewb): Not implemented!"); }
      /***************************************************/

    private:
      /**
       * \brief contour index
       */
      int icircle, ifrustum;
      
      /**
       * \brief contour classes
       */
      Circle *circle;
      PlanarFrustum *frustum;

  };

}

#endif

