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

#ifndef _KINEMATIC_EXCITATION_H_
#define _KINEMATIC_EXCITATION_H_

#include "mbsim/link_mechanics.h"
#include "mbsim/rigid_body.h"

namespace MBSim {

  class KinematicExcitation : public LinkMechanics {
    protected:
      Function1<fmatvec::VecV,double> *f, *fd, *fdd;
      Function2<fmatvec::VecV,fmatvec::VecV,fmatvec::VecV> *func;
      RigidBody* body;
    public:
      KinematicExcitation(const std::string &name);
      void updateh(double, int i=0);
      void updateW(double, int i=0);
      void updateg(double);
      void updategd(double);
      void updatewb(double t, int i=0);
      void updatehRef(const fmatvec::Vec &hParent, int j=0);
      void updateWRef(const fmatvec::Mat &WParent, int j=0);
      void setReferenceBody(RigidBody* body_) {body = body_;}

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      std::string getType() const { return "KinematicExcitation"; }
      void init(InitStage stage);
      bool isSetValued() const;
      virtual void calclaSize(int j);
      virtual void calcgSize(int j);
      virtual void calcgdSize(int j);

      void setKinematicFunction(Function1<fmatvec::VecV,double>* f_) { f = f_;}
      void setFirstDerivativeOfKinematicFunction(Function1<fmatvec::VecV,double>* fd_) { fd = fd_;}
      void setSecondDerivativeOfKinematicFunction(Function1<fmatvec::VecV,double>* fdd_) { fdd = fdd_;}
      void setForceFunction(Function2<fmatvec::VecV,fmatvec::VecV,fmatvec::VecV> *func_) { func=func_; }

      void plot(double t, double dt=1);

    private:
  };

}

#endif
