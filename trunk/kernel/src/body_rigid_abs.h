/* Copyright (C) 2004-2006  Martin Förg
 
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
 *
 */

#ifndef _BODY_RIGID_ABS_H_
#define _BODY_RIGID_ABS_H_

#include "body_rigid.h"
#include "fmatvec.h"
#include <vector>

namespace MBSim {

  /*! \brief Class for rigid bodies with absolute coordinates 
   *
   * */
  class BodyRigidAbs : public BodyRigid {

    protected:
      Vec WrOK0;
      SqrMat AWK0;

      virtual void updateCenterOfGravity(double t);
      virtual void updateh(double t);
      virtual void updateW(double t);
      virtual void updatezd(double t);
      virtual void updatedu(double t, double dt);
      void (BodyRigidAbs::*updateM)();
      void updateM1() {};
      void updateM2();
    public:

      BodyRigidAbs(const string &name);

      Vec computeJTqT() const {return JT*q(iT);}
      Vec computeJRqR() const {return JR*q(iR);}

      const Mat& getJT() const {return JT;}
      const Mat& getJR() const {return JR;}

      void init();

      const Vec& getWrOK() const {return WrOK;}
      const Vec& getWrOK0() const {return WrOK0;}
      const Vec& getWvK() const {return WvK;}
      const SqrMat& getAWK0() const {return AWK0;}

      const Vec& getWrOS0() const;

      /*! define initial position of center of gravity in world system
      */
      void setWrOS0(const Vec &WrOK0_);

      /*! define initial position of point of reference in world system
      */
      void setWrOK0(const Vec &WrOK0_);
      /*! define 
      */
      void setAWK0(const SqrMat &A); 
  };
}

#endif

