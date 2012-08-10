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

#ifndef _GEAR_H_
#define _GEAR_H_

#include "mbsim/link_mechanics.h"
#include "mbsim/rigid_body.h"

namespace MBSim {

  class Gear : public LinkMechanics {
    protected:
      Function2<double,double,double> *func;
      std::vector<RigidBody*> body;
      std::vector<double> ratio[2];
      Frame *frame;
    public:
      Gear(const std::string &name);
      void updateh(double, int i=0);
      void updateW(double, int i=0);
      void updateg(double);
      void updategd(double);
      void updatewb(double t, int i=0);
      void updatehRef(const fmatvec::Vec &hParent, int j=0);
      void updateWRef(const fmatvec::Mat &WParent, int j=0);
      void setDependentBody(RigidBody* body_) {body[0] = body_;}
      void addDependency(RigidBody* body_, double ratio1, double ratio2=0);
      void connect(Frame* frame_) {frame = frame_;}
     // {LinkMechanics::connect(frame);}

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      std::string getType() const { return "Gear"; }
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
