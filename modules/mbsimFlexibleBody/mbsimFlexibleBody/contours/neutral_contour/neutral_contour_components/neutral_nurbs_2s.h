/*
 * neutral_nurbs_2s.h
 *
 *  Created on: 03.12.2013
 *      Author: zwang
 */

#ifndef NEUTRAL_NURBS_2S_H_
#define NEUTRAL_NURBS_2S_H_

#include <mbsim/numerics/nurbs/nurbs_surface.h>

namespace MBSim {
  class ContourFrame;
}

namespace MBSimFlexibleBody {
  
  class NeutralNurbs2s {
    public:
      NeutralNurbs2s(MBSim::Element* parent_,  const fmatvec::MatVI & nodes, double nodeOffset_, int degU_, int degV_, bool openStructure_);
      virtual ~NeutralNurbs2s();
      void setuk(const fmatvec::Vec &uk_) { uk = uk_; }
      void setvl(const fmatvec::Vec &vl_) { vl = vl_; }
      virtual void computeCurve(bool update = false);
      virtual void update(MBSim::ContourFrame *frame) = 0;
//      virtual const fmatvec::Vec getuVec() {
//        return surface.getuVec();
//      }
      void resetUpToDate();
    protected:
      virtual void buildNodelist() = 0;
      MBSim::NurbsSurface surface;
      MBSim::Element *parent;
      fmatvec::MatVI nodes;
      double nodeOffset;
      int numOfNodesU;
      int numOfNodesV;
      GeneralMatrix<Vec3> Nodelist;
      fmatvec::Vec uk, vl;
      int degU;
      int degV;
      bool openStructure;
      bool updSurface;
  };

} /* namespace MBSimFlexibleBody */
#endif
