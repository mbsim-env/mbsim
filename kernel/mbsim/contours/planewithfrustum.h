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

#ifndef _PLANEWITHFRUSTUM_H_
#define _PLANEWITHFRUSTUM_H_

#include "mbsim/contours/rigid_contour.h"

namespace MBSim {

  /** 
   * \brief plane without borders and a frustum on reference kos
   * \author Markus Schneider
   * \date 2009-07-14 initial development
   *
   * normal equals first column in orientation matrix
   */
  class PlaneWithFrustum : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       * \param radius of frustum on contour side
       * \param radius of frustum on "free" side
       * \param height of frustum (positive in "free" space, negative in "solid" space)
       * \param radius of the (small) rouding 
       */
      PlaneWithFrustum(const std::string &name="", Frame *R=nullptr) : RigidContour(name,R) {
      }
      
      PlaneWithFrustum(const std::string &name, double rFrustumOnPlane_, double rFrustumOnTop_, double hFrustum_, double rho_, Frame *R=nullptr) : RigidContour(name,R), rFrustumOnPlane(rFrustumOnPlane_), rFrustumOnTop(rFrustumOnTop_), hFrustum(hFrustum_), rho(rho_) {
        checkInput();
      }

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      /* GETTER / SETTER */
      double getFrustumRadiusOnPlane() {return rFrustumOnPlane; }
      double getFrustumRadiusOnTop() {return rFrustumOnTop; }
      double getFrustumHeight() {return hFrustum; }
      double getRoundingRadius() {return rho; }
      /***************************************************/

      void enableOpenMBV();

      void initializeUsingXML(xercesc::DOMElement *element) override;

    private:
      double rFrustumOnPlane{0};
      double rFrustumOnTop{0};
      double hFrustum{0};
      double rho{0};

      void checkInput() {
        assert(rFrustumOnTop<rFrustumOnPlane); //TODO
        assert(rFrustumOnTop>1e-6); //TODO
        assert(rho>1e-6);
        assert(rho<fabs(hFrustum));
        assert(rho<rFrustumOnTop);
      }
  };
}

#endif /* _PLANEWITHFRUSTUM_H_ */
