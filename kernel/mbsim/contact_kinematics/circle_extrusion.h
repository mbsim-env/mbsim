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

#ifndef _CONTACT_KINEMATICS_CIRCLEEXTRUSION_H_
#define _CONTACT_KINEMATICS_CIRCLEEXTRUSION_H_

#include "contact_kinematics.h"

namespace MBSim {

  class Circle;
  class FuncPairPlanarContourCircle;

  /**
   * \brief pairing outer circle side to extrusion
   * \author Martin Foerg
   * \date 2009-07-28 pure virtual updates (Thorsten Schindler)
   */
  class ContactKinematicsCircleExtrusion : public ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsCircleExtrusion() : icircle(0), iextrusion(0), circle(NULL), extrusion(NULL), func(NULL), searchAllCP(false) { }

      /**
       * \brief destructor
       */
      virtual ~ContactKinematicsCircleExtrusion();

      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<Contour*> &contour);
      void updateg(SingleContact &contact, int i=0) override;
      void updatewb(SingleContact &contact, int i=0) override;
      /***************************************************/

      void setSearchAllContactPoints(bool searchAllCP_=true) { searchAllCP = searchAllCP_; }

    private:
      /**
       * \brief contour index
       */
      int icircle, iextrusion;

      /**
       * \brief contour classes
       */
      Circle *circle;
      Contour *extrusion;

      /**
       * \brief root function
       */
      FuncPairPlanarContourCircle *func;

      bool searchAllCP;
  };

}

#endif
