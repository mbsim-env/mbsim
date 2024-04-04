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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _JOINT_CONSTRAINT_H
#define _JOINT_CONSTRAINT_H

#include "mechanical_constraint.h"
#include "mbsim/functions/function.h"
#include "mbsim/frames/floating_relative_frame.h"
#include "mbsim/utils/nonlinear_algebra.h"

namespace MBSim {

  class RigidBody;
  class Frame;

  /** 
   * \brief Joint contraint 
   * \author Martin Foerg
   * 2011-08-04 XML Interface added (Markus Schneider)
   */
  class JointConstraint : public MechanicalConstraint {
    public:
      enum FrameOfReference {
        firstFrame=0,
        secondFrame,
        unknown
      };

      JointConstraint(const std::string &name="");

      void calcisSize() override;

      void init(InitStage stage, const InitConfigSet &config) override;

      void resetUpToDate() override;

      void connect(Frame* frame0, Frame* frame1) { frame[0] = frame0; frame[1] = frame1; }
      void addDependentRigidBodyOnFirstSide(RigidBody* bd) { bd1.push_back(bd); }
      void addDependentRigidBodyOnSecondSide(RigidBody* bd) { bd2.push_back(bd); }
      void setIndependentRigidBody(RigidBody* bi_) { bi.resize(1); bi[0] = bi_; }
      void addIndependentRigidBody(RigidBody* bi_) { bi.push_back(bi_); }

      void setUpInverseKinetics() override;
      void setForceDirection(const fmatvec::Mat3xV& fd) { forceDir <<= fd; }
      void setMomentDirection(const fmatvec::Mat3xV& md) { momentDir <<= md; }

      /** \brief The frame of reference for the force/moment direction vectors.
       * If firstFrame (default) the first frame, if secondFrame the second frame is used.
       */
      void setFrameOfReference(FrameOfReference refFrame_) { refFrame = refFrame_; }

      void updatePositions(Frame *frame) override;
      void updateGeneralizedCoordinates() override;
      void updateGeneralizedJacobians(int jj=0) override;
      void updateForceDirections();
      void updateA();

      const fmatvec::Mat3xV& evalGlobalForceDirection() { if(updDF) updateForceDirections(); return DF; }
      const fmatvec::Mat3xV& evalGlobalMomentDirection() { if(updDF) updateForceDirections(); return DM; }
      const fmatvec::SqrMat& evalA() { if(updA) updateA(); return A; }

      fmatvec::Mat3xV& getGlobalForceDirection(bool check=true) { assert((not check) or (not updDF)); return DF; }
      fmatvec::Mat3xV& getGlobalMomentDirection(bool check=true) { assert((not check) or (not updDF)); return DM; }
      fmatvec::SqrMat& getA(bool check=true) { assert((not check) or (not updA)); return A; }

      void setInitialGuess(const fmatvec::VecV &q0_) { q0 <<= q0_; }

      void initializeUsingXML(xercesc::DOMElement *element) override;

    private:
      class Residuum : public Function<fmatvec::Vec(fmatvec::Vec)> {
        private:
          std::vector<RigidBody*> body1, body2;
          fmatvec::Mat3xV forceDir, momentDir;
          std::vector<Frame*> frame;
          FrameOfReference refFrame;
        public:
          Residuum(std::vector<RigidBody*> body1_, std::vector<RigidBody*> body2_, const fmatvec::Mat3xV &forceDir_, const fmatvec::Mat3xV &momentDir_, const std::vector<Frame*> frame_, FrameOfReference refFrame_);

          fmatvec::Vec operator()(const fmatvec::Vec &x) override;
      };
      std::unique_ptr<Residuum> residuum;
      std::unique_ptr<MultiDimNewtonMethod> newton;

      std::vector<RigidBody*> bd1, bd2, bi;

      std::vector<Frame*> frame;

      /**
       * \brief frame of reference the force is defined in
       */
      FrameOfReference refFrame{firstFrame};

      FloatingRelativeFrame C;

      fmatvec::Mat3xV DF, DM, forceDir, momentDir;

      fmatvec::RangeV iF, iM;
      std::vector<fmatvec::RangeV> Iq1, Iq2, Iu1, Iu2, Ih1, Ih2;
      int nq{0};
      int nu{0};
      int nh{0};
      fmatvec::Vec q0;
      fmatvec::Mat JT, JR;
      fmatvec::SqrMat A;

      bool updDF{true};
      bool updA{true};

      std::string saved_ref1, saved_ref2;
      std::vector<std::string> saved_RigidBodyFirstSide, saved_RigidBodySecondSide, saved_IndependentBody;
  };

}

#endif
