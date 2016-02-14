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

#ifndef _CONTACT_KINEMATICS_CIRCLESOLIDCONTOUR1S_H_
#define _CONTACT_KINEMATICS_CIRCLESOLIDCONTOUR1S_H_

#include "contact_kinematics.h"

namespace MBSim {

  class SolidCircle;
  class FuncPairContour1sSolidCircle;

  /**
   * \brief pairing outer circle side to contour1s
   * \author Martin Foerg
   * \date 2009-07-28 pure virtual updates (Thorsten Schindler)
   * \todo change stage to new interface TODO
   */
  class ContactKinematicsSolidCircleContour1s : public ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsSolidCircleContour1s() : icircle(0), icontour1s(0), circle(NULL), contour1s(NULL), func(NULL), searchAllCP(false) {}

      /**
       * \brief destructor
       */
      virtual ~ContactKinematicsSolidCircleContour1s();

      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(double t, double &g, std::vector<Frame*> &cFrame, int index = 0);
      virtual void updatewb(double t, fmatvec::Vec &wb, double g, std::vector<Frame*> &cFrame);
      /***************************************************/

      void setSearchAllCP(bool searchAllCP_=true) {searchAllCP=searchAllCP_; }

    private:
      /**
       * \brief contour index
       */
      int icircle, icontour1s;

      /**
       * \brief contour classes
       */
      SolidCircle *circle;
      Contour *contour1s;

      /**
       * \brief root function
       */
      FuncPairContour1sSolidCircle *func;

      bool searchAllCP;

      fmatvec::Vec2 zeta;
  };

}

#endif /* _CONTACT_KINEMATICS_CIRCLESOLIDCONTOUR1S_H_ */

