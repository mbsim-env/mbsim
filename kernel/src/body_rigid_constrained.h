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

#ifndef _BODY_RIGID_CONSTRAINED_H_
#define _BODY_RIGID_CONSTRAINED_H_

#include "body_rigid_abs.h"


namespace MBSim {

  class DataInterfaceBase;

  /*! \brief 
   * bodies with prescribed motion
   * \todo Vielleicht besser nicht von BodyRigid ableiten
   * */
  class BodyRigidConstrainedAcc : public BodyRigidAbs {

    private:
      DataInterfaceBase *pos;
      DataInterfaceBase *vel;
      DataInterfaceBase *acc;
      double delta, sqrtdelta;

    protected:

      void updateGb(double t);
      void updateh(double t);
      void updater(double t);
      void updatezd(double t);
      void updatedu(double t, double dt);
      void updatedq(double t, double dt);

    public:

      BodyRigidConstrainedAcc(const string &name);
      virtual ~BodyRigidConstrainedAcc();

      void setAcceleration(DataInterfaceBase *func);
      void setVelocity(DataInterfaceBase *func);
      void setPosition(DataInterfaceBase *func);

      void init();
  };

  /*! Comment
   *
   * */
  class BodyRigidConstrainedVel : public BodyRigidAbs {

    private:
      DataInterfaceBase *pos;
      DataInterfaceBase *vel;
      bool overwriteqWithpos;
      double delta, sqrtdelta;

    protected:

      void updateGb(double t);
      void updateh(double t);
      void updater(double t);
      void updatezd(double t);
      void updatedu(double t, double dt);
      void updatedq(double t, double dt);

      void updateKinematics(double t);

    public:

      BodyRigidConstrainedVel(const string &name);
      virtual ~BodyRigidConstrainedVel();

      void setVelocity(DataInterfaceBase *func);
      /** If option overwriteq is set to true, the integrator state q (Position) is overwriten with the Position given by func. This may cause integrator instability.*/
      void setPosition(DataInterfaceBase *func, bool  overwriteq=0); 

      void init();

      void plot(double t, double dt=1);
      void initPlotFiles();
  };

}

#endif
