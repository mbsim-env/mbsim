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
 * Contact: thorsten.schindler@mytum.de
 *          rzander@users.berlios.de
 */

#ifndef _FLEXIBLE_BODY_H_
#define _FLEXIBLE_BODY_H_

#define MBSIMFLEXNS "{http://mbsim.berlios.de/MBSimFlexibleBody}"

#include "mbsim/body.h"
#include "mbsim/frame.h"

namespace MBSim {
  class DiscretizationInterface;
  class ContourPointData;
}

namespace MBSimFlexibleBody {

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
   * \date 2010-06-20 revision of doxygen comments: add parameter names (Roland Zander)
   * \todo OpenMP only static scheduling with intelligent reordering of vectors by dynamic test runs TODO
   * \todo mass proportional damping should be distributed on discretization and is not at the correct place (dependence on M) TODO
   */
  class FlexibleBody : public MBSim::Body {
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
      virtual void updateh(double t, int k=0);
      virtual void updateM(double t, int k=0);
      virtual void updatedhdz(double t);
      virtual void updateStateDependentVariables(double t);
      virtual void updateJacobians(double t, int k=0);

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody"; }
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(MBSim::InitStage stage);
      virtual double computeKineticEnergy();
      virtual double computePotentialEnergy();
      virtual void setFrameOfReference(MBSim::Frame *frame);
      void setq0(fmatvec::Vec q0_) { if(q0_.size()) MBSim::Body::setInitialGeneralizedPosition(q0_); q<<q0; }
      void setu0(fmatvec::Vec u0_) { if(u0_.size()) MBSim::Body::setInitialGeneralizedVelocity(u0_); u<<u0; }
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
       * \param CurrentElement number of current finite element
       * \param locMat local matrix
       * \param gloMat global matrix
       */
      virtual void GlobalMatrixContribution(int CurrentElement, const fmatvec::Mat &locMat, fmatvec::Mat &gloMat) = 0;

      /**
       * \brief insert 'local' information in global matrices
       * \param CurrentElement number of current finite element
       * \param locMat local matrix
       * \param gloMat global matrix
       */
      virtual void GlobalMatrixContribution(int CurrentElement, const fmatvec::SymMat &locMat, fmatvec::SymMat &gloMat) = 0;

      /**
       * \brief cartesian kinematic for contour or external frame (normal, tangent, binormal) is set by implementation class
       * \param data contour parameter
       * \param ff selection of specific calculations for frames
       * \param frame optional: external frame, otherwise contour parameters are changed
       */
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &data, MBSim::FrameFeature ff, MBSim::Frame *frame=0) = 0;

      /**
       * \brief Jacobians and gyroscopes for contour or external frame are set by implementation class
       * \param data contour parameter
       * \param frame: optional external frame, otherwise contour parameters are changed
       */
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &data, MBSim::Frame *frame=0) = 0;
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
      void addFrame(const std::string &name, const MBSim::ContourPointData &S_);

      /**
       * \param frame
       * \param frame location
       */
      void addFrame(MBSim::Frame *frame, const MBSim::ContourPointData &S_);

      /**
       *  \param name of frame
       *  \param node of frame
       */
      void addFrame(const std::string &name, const int &id);

      /**
       * \param frame
       * \param node of frame
       */
      void addFrame(MBSim::Frame *frame, const  int &id);

      void addContour(MBSim::Contour *contour);

      /**
       * \brief interpolates the position and optional the velocity coordinates of the flexible body with Nurbs-package and exports the nurbs curve in the specified file
       * \param filenamePos    Name of the exported position curve file
       * \param filenameVel    Name of the exported velocity curve file
       * \param deg            Degree of Nurbs interpolation
       * \param writePsFile A Postscript-file of the curve profile is created
       *
       * Remark: the knot vector is parametrized between [0,L]
       */
      virtual void exportPositionVelocity(const std::string & filenamePos, const std::string & filenameVel = std::string(), const int & deg = 3, const bool &writePsFile = false){throw  MBSim::MBSimError("exportPositionVelocity(const std::string& filenamePos, const std::string& filenameVel, const int& deg, const bool& writePsFile) is not implemented for " + this->getType()) ;}

      /**
       * \brief imports the interpolated position and optional the velocity files (created with exportPositionVelocity) and fits the rigid and flexible coordinate dofs and optional the translatory velocity components of flexible body to the imported nurbs curve
       * \param filenamePos    Name of the imported position curve file
       * \param filenameVel    Name of the imported velocity curve file
       */
      virtual void importPositionVelocity(const std::string& filenamePos, const std::string& filenameVel = std::string()){throw  MBSim::MBSimError("importPositionVelocity(const std::string& filenamePos, const std::string& filenameVel) is not implemented for " + this->getType()) ;}

    protected:
      /**
       * \brief stl-vector of discretizations/finite elements
       */
      std::vector<MBSim::DiscretizationInterface*> discretization;

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
      std::vector<MBSim::ContourPointData> S_Frame;
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
          MBSim::ContourPointData cp(alpha);
          FlexibleBody::addFrame(name,cp);
        }

        /**
         * \param frame
         * \param location of frame
         */
        void addFrame(MBSim::Frame *frame, const AT& alpha) {
          MBSim::ContourPointData cp(alpha);
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

