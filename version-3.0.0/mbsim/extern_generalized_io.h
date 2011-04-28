/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _EXTERNGENERALIZEDIO_H_
#define _EXTERNGENERALIZEDIO_H_

#include "mbsim/link.h"

namespace MBSim {

  class ExternGeneralizedIO : public Link {
    protected:
      Object *connectedObject;
      int qInd, uInd;
      double m, a, t0;
      struct ApplyForceAlsoTo {
        std::string saved_ref;
        Object *ref;
        double factor;
        int index;
      };
      std::vector<ApplyForceAlsoTo> applyForceAlsoTo;
    public:
      enum Type {
        CONSTANT, LINEAR
      };
      ExternGeneralizedIO(const std::string &name);
      void updateh(double);
      void updateg(double);
      void updategd(double);

      void calcxSize();
      void updatedx(double t, double dt);
      void updatexd(double t);

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      std::string getType() const { return "ExternGeneralizedIO"; }
      void init(InitStage stage);

      void connect(Object *obj, int qInd_, int uInd_) { connectedObject=obj; qInd=qInd_; uInd=uInd_; }
      void setGeneralizedForce(double h) { la(0)=h; }
      void setGeneralizedLinearForceParameters(double m_, double a_, double t0_) { m=m_; a=a_; t0=t0_; }
      double getGeneralizedPosition() { return g(0); }
      double getGeneralizedVelocity() { return gd(0); }

      void plot(double t, double dt=1);
      void initializeUsingXML(TiXmlElement *element);
      Type getType() { return type; }

      virtual void updateWRef(const fmatvec::Mat&, int) {}
      virtual void updateVRef(const fmatvec::Mat&, int) {}
      virtual void updatehRef(const fmatvec::Vec&, const fmatvec::Vec&, int) {}
      virtual void updatedhdqRef(const fmatvec::Mat&, int) {}
      virtual void updatedhduRef(const fmatvec::SqrMat&, int) {}
      virtual void updatedhdtRef(const fmatvec::Vec&, int) {}
      virtual void updaterRef(const fmatvec::Vec&, int) {}

      void addApplyForceAlsoTo(Object *ref, double factor, int index);

    private:
      std::string saved_connectedObject;
    protected:
      Type type;
  };

}

#endif /* _SPRINGDAMPER_H_ */
