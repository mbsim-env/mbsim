/* Copyright (C) 2004-2019 MBSim Development Team
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

#ifndef _INVERSE_KINEMATICS_CONSTRAINT_H
#define _INVERSE_KINEMATICS_CONSTRAINT_H

#include "constraint.h"
#include "mbsim/functions/function.h"
#include "mbsim/frames/floating_relative_frame.h"

namespace MBSim {

  class RigidBody;
  class Frame;
  class MechanicalLink;

  /** 
   * \brief Inverse kinematics contraint 
   * \author Martin Foerg
   */
  class InverseKinematicsConstraint : public Constraint {
    public:
      enum Kinematics {
        planar=0,
        spatial,
        unknown
      };

      InverseKinematicsConstraint(const std::string &name="") : Constraint(name) { }
      ~InverseKinematicsConstraint() override;

      void init(InitStage stage, const InitConfigSet &config) override;

      void resetUpToDate() override;

      void setKinematics(Kinematics kinematics_) { kinematics = kinematics_; }
      void setFrame(Frame* frame_) { frame = frame_; }

      void setTranslation(Function<fmatvec::Vec3(double)> *fr_) {
        fr = fr_;
        fr->setParent(this);
        fr->setName("Translation");
      }
      void setRotation(Function<fmatvec::RotMat3(double)>* fA_) {
        fA = fA_;
        fA->setParent(this);
        fA->setName("Rotation");
      }
      void setUpInverseKinetics() override;

      void updateGeneralizedCoordinates() override;
      void updateGeneralizedJacobians(int jj=0) override;
      void updateA();

      const fmatvec::SqrMat& evalA() { if(updA) updateA(); return A; }

      fmatvec::SqrMat& getA(bool check=true) { assert((not check) or (not updA)); return A; }

      void setInitialGuess(const fmatvec::VecV &q0_) { q0 <<= q0_; }

      void initializeUsingXML(xercesc::DOMElement *element) override;

      MechanicalLink* getMechanicalLink(int i) { return link[i]; }
      int getNumberOfMechanicalLinks() const { return link.size(); }

    private:
      class Residuum : public Function<fmatvec::Vec(fmatvec::Vec)> {
        private:
          fmatvec::Vec3 r;
          fmatvec::SqrMat3 A;
          std::vector<RigidBody*> body;
          fmatvec::Mat3xV forceDir, momentDir;
          Frame* frame;
        public:
          Residuum(const fmatvec::Vec3 &r_, const fmatvec::RotMat3 &A_, std::vector<RigidBody*> body_, const fmatvec::Mat3xV &forceDir_, const fmatvec::Mat3xV &momentDir_, Frame* frame_);

          fmatvec::Vec operator()(const fmatvec::Vec &x) override;
      };
      std::vector<RigidBody*> bd;

      Kinematics kinematics{spatial};
      Frame* frame{nullptr};

      Function<fmatvec::Vec3(double)> *fr{nullptr};
      Function<fmatvec::RotMat3(double)>* fA{nullptr};

      fmatvec::Mat3xV DF, DM, forceDir, momentDir;

      fmatvec::RangeV iF, iM;
      std::vector<fmatvec::RangeV> Iq, Iu, Ih;
      int nq{0};
      int nu{0};
      int nh{0};
      fmatvec::Vec q, q0;
      fmatvec::Mat JT, JR;
      fmatvec::SqrMat A;

      std::vector<MechanicalLink*> link;

      bool updA{true};

      std::string saved_ref;
  };

}

#endif
