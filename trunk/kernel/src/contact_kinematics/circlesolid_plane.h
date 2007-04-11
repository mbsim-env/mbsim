/* Copyright (C) 2007  Martin Förg, Roland Zander
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *   rzander@users.berlios.de
 *
 */

#ifndef _CONTACT_KINEMATICS_CIRCLESOLID_PLANE_H_
#define _CONTACT_KINIMATICS_CIRCLESOLID_PLANE_H_

#include "contact_kinematics.h"

namespace MBSim {

  class CircleSolid;
  class Plane;

  /** pairing CircleSolid to Plane; author: Martin Foerg */
  class ContactKinematicsCircleSolidPlane : public ContactKinematics {
    private:
      int icircle, iplane;
      CircleSolid *circlesolid;
      Plane *plane;
      Vec genBuf;

    public:
      void stage1(Vec &g, vector<ContourPointData> &cpData);
      void stage2(const Vec &g, Vec &gd, vector<ContourPointData> &cpData);

      void assignContours(const vector<Contour*> &contour);
  };

}

#endif

