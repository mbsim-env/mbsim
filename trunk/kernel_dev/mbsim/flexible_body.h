/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: thschindler@users.berlios.de
 *          rzander@users.berlios.de
 */

#ifndef _FLEXIBLE_BODY_H_
#define _FLEXIBLE_BODY_H_

#include "mbsim/dynamic_system.h"
#include "mbsim/body.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/contour_pdata.h"
#include "mbsim/discretization_interface.h"
#include <vector>

namespace MBSim {

  /**
   * \brief upmost class for flexible body implementation
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-03-24 changes for new MBSim (Thorsten Schindler)
   * \date 2009-03-25 conflicts solved (Thorsten Schindler)
   * \date 2009-04-05 changed to non-template definition (Schindler / Zander)
   * \date 2009-04-20 frame concept (Thorsten Schindler)
   * \date 2009-06-14 OpenMP (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-27 implicit integration (Thorsten Schindler)
   * \todo OpenMP only static scheduling with intelligent reordering of vectors by dynamic test runs
   */
  class FlexibleBody : public Body {
    public:
      /**
       * \brief constructor
       * \param name of body
       */
      FlexibleBody(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~FlexibleBody();

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updatedq(double t, double dt) { qd = u*dt; }
      virtual void updateqd(double t) { qd = u; }
      virtual void updateh(double t);
      virtual void updateM(double t);
      virtual void updatedhdz(double t);
      virtual void updateStateDependentVariables(double t);
      virtual void updateJacobians(double t);
      virtual void updateInverseKineticsJacobians(double t);

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody"; }

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(InitStage stage);
      virtual double computeKineticEnergy();
      virtual double computePotentialEnergy();
      virtual void setFrameOfReference(Frame *frame) { if(dynamic_cast<DynamicSystem*>(frame->getParent())) frameOfReference = frame; else throw MBSimError("ERROR (FlexibleBody::setFrameOfReference): Only stationary reference frames are implemented at the moment!"); }
      void setq0(fmatvec::Vec q0_) { Body::setInitialGeneralizedPosition(q0_); q>>q0; }
      void setu0(fmatvec::Vec u0_) { Body::setInitialGeneralizedVelocity(u0_); u>>u0; }
      /***************************************************/

      /* INTERFACE TO BE DEFINED IN DERIVED CLASSES */
      /**
       * \brief references finite element coordinates to assembled coordinates
       */
      virtual void BuildElements() = 0;

      /** 
       * \brief insert 'local' information in global vectors
       * \param number of finite element
       * \param local vector
       * \param global vector
       */
      virtual void GlobalVectorContribution(int CurrentElement, const fmatvec::Vec &locVec, fmatvec::Vec &gloVec) = 0;

      /** 
       * \brief insert 'local' information in global matrices
       * \param number of finite element
       * \param local matrix 
       * \param global matrix
       */
      virtual void GlobalMatrixContribution(int CurrentElement, const fmatvec::Mat &locMat, fmatvec::Mat &gloMat) = 0;

      /** 
       * \brief insert 'local' information in global matrices
       * \param number of finite element
       * \param local matrix 
       * \param global matrix
       */
      virtual void GlobalMatrixContribution(int CurrentElement, const fmatvec::SymMat &locMat, fmatvec::SymMat &gloMat) = 0;

      /**
       * \brief cartesian kinematic for contour or external frame (normal, tangent, binormal) is set by implementation class
       * \param contour parameter
       * \param selection of specific calculations for frames
       * \param possible external frame, otherwise contour parameters are changed
       */
      virtual void updateKinematicsForFrame(ContourPointData &data, FrameFeature ff, Frame *frame=0) = 0;

      /**
       * \brief Jacobians and gyroscopes for contour or external frame are set by implementation class
       * \param contour parameter
       * \param possible external frame, otherwise contour parameters are changed
       */
      virtual void updateJacobiansForFrame(ContourPointData &data, Frame *frame=0) = 0;
      /***************************************************/

      /* GETTER / SETTER */
      /*! 
       * damping matrix computation, updated with changes in mass matrix \f$\vM\f$: \f$\vh_d=-d_{pm}\vM\vu\f$ 
       * \brief set mass proportional damping
       * \param d_ coefficient \f$d_{pm}\f$
       */
      void setMassProportionalDamping(const double d_) { d_massproportional = d_; }
      /***************************************************/

      /** 
       *  \param name of frame
       *  \param frame location
       */
      void addFrame(const std::string &name, const ContourPointData &S_);

      /**
       * \param frame
       * \param frame location
       */
      void addFrame(Frame *frame, const ContourPointData &S_);

      /** 
       *  \param name of frame
       *  \param node of frame 
       */
      void addFrame(const std::string &name, const int &id) {
        ContourPointData cp(id);
        addFrame(name,cp);
      }

      /**
       * \param frame
       * \param node of frame
       */
      void addFrame(Frame *frame, const  int &id) {
        ContourPointData cp(id);
        addFrame(frame,cp);
      }

    protected:
      /** 
       * \brief stl-vector of discretizations/finite elements
       */
      std::vector<DiscretizationInterface*> discretization;

      /** 
       * \brief stl-vector of finite element wise positions
       */
      std::vector<fmatvec::Vec> qElement;

      /** 
       * \brief stl-vector of finite element wise velocities
       */
      std::vector<fmatvec::Vec> uElement;

      /** 
       * \brief damping factor for mass proportion, see BodyFlexible::setMassProportionalDamping()
       */
      double d_massproportional;

      /** 
       * \brief vector of contour parameters each describing a frame
       */
      std::vector<ContourPointData> S_Frame;
  };

  /**
   * \brief flexible body entirely described within MBSim holding all informations about continuum approximations
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-04-05 initial definition (Schindler / Zander)
   */
  template <class AT>
    class FlexibleBodyContinuum : public FlexibleBody {
      public:
        /**
         * \brief constructor
         * \param name of flexible body
         */
        FlexibleBodyContinuum<AT>(const std::string &name) : FlexibleBody(name) {}

        /* INHERITED INTERFACE OF ELEMENT */
        virtual std::string getType() const { return "FlexibleBodyContinuum"; }

        /* GETTER / SETTER */
        void setContourNodes(const std::vector<AT> nodes) { userContourNodes = nodes; }

        using FlexibleBody::addFrame;

        /**
         * \param name of frame
         * \param location of frame
         */
        void addFrame(const std::string &name, const AT& alpha) {
          ContourPointData cp(alpha);
          FlexibleBody::addFrame(name,cp);
        }

        /**
         * \param frame
         * \param location of frame
         */
        void addFrame(Frame *frame, const AT& alpha) {
          ContourPointData cp(alpha);
          FlexibleBody::addFrame(frame,cp);
        }

      protected:
        /**
         * \brief grid for contact point detection
         */
        std::vector<AT> userContourNodes;
    };
}

#endif /* _FLEXIBLE_BODY_H_ */

