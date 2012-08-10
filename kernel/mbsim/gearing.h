/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _GEARING_H_
#define _GEARING_H_

#include "mbsim/link_mechanics.h"
#include "mbsim/rigid_body.h"
#include "mbsim/frame.h"

namespace MBSim {

  class Gearing : public LinkMechanics {
    protected:
      Function2<double,double,double> *func;
      double r0, r1;
      bool flag;
      Frame Z0, Z1;
      Frame *P0, *P1;
      fmatvec::Vec3 WrP0Z, WrP1Z;
      fmatvec::Vec3 Wt;
    public:
      Gearing(const std::string &name, bool flag = false);
      void updateh(double, int i=0);
      void updateW(double, int i=0);
      void updateJacobians(double t, int j=0);
      void updateg(double);
      void updategd(double);
      void updatewb(double t, int i=0);
      void updatehRef(const fmatvec::Vec &hParent, int j=0);
      void updateWRef(const fmatvec::Mat &WParent, int j=0);
      void connect(double r1, Frame* frame1_, double r2, Frame* frame2_);
      virtual void updatexd(double t);
      virtual void updatedx(double t, double dt);
      virtual void calcxSize();

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      std::string getType() const { return "Gearing"; }
      void init(InitStage stage);
      bool isSetValued() const;
      virtual void calclaSize();
      virtual void calcgSize();
      virtual void calcgdSize();

      void setForceFunction(Function2<double,double,double> *func_) { func=func_; }

      void plot(double t, double dt=1);

    private:
  };

}

#endif
