/* Copyright (C) 2004-2014 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _FRAME_LINK_H_
#define _FRAME_LINK_H_

#include "mbsim/links/link.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace OpenMBV {
  class Group;
  class Arrow;
}
#endif

namespace H5 {
  class Group;
}

namespace MBSim {
  /** 
   * \brief frame link
   * \author Martin Foerg
   */
  class FrameLink : public Link {
    public:
      /**
       * \brief constructor
       * \param name of link machanics
       */
      FrameLink(const std::string &name);

      virtual void init(InitStage stage);

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Link"; }
      virtual void plot();
      virtual void closePlot();
      /***************************************************/

      void initializeUsingXML(xercesc::DOMElement *element);

      void updateh(int i=0);
      void updatedhdz();

      /* INHERITED INTERFACE OF LINK */
      virtual void updateWRef(const fmatvec::Mat& ref, int i=0);
      virtual void updateVRef(const fmatvec::Mat& ref, int i=0);
      virtual void updatehRef(const fmatvec::Vec &hRef, int i=0);
      virtual void updatedhdqRef(const fmatvec::Mat& ref, int i=0);
      virtual void updatedhduRef(const fmatvec::SqrMat& ref, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& ref, int i=0);
      virtual void updaterRef(const fmatvec::Vec &ref, int i=0);
      /***************************************************/

      /* INTERFACE TO BE DEFINED IN DERIVED CLASS */
      /**
       * \param frame to add to link frame vector
       */
      void connect(Frame *frame0, Frame* frame1) {
        frame[0] = frame0;
        frame[1] = frame1;
      }

      Frame* getFrame(int i) { return frame[i]; }

      void resetUpToDate();
      virtual void updatePositions();
      virtual void updateVelocities();
      void updateGeneralizedPositions();
      void updateGeneralizedVelocities();
      void updateGeneralizedForces();
      void updateForce();
      void updateForceDirections();
      void updateR();
      virtual void updatelaF() { }
      virtual void updatelaM() { }
      const fmatvec::Vec3& evalGlobalRelativePosition() { if(updPos) updatePositions(); return WrP0P1; }
      const fmatvec::Vec3& evalGlobalRelativeVelocity() { if(updVel) updateVelocities(); return WvP0P1; }
      const fmatvec::Vec3& evalGlobalRelativeAngularVelocity() { if(updVel) updateVelocities(); return WomP0P1; }
      const fmatvec::Mat3xV& evalGlobalForceDirection() { if(updFD) updateForceDirections(); return DF; }
      const fmatvec::Vec3& evalForce() { if(updF) updateForce(); return F; }
      const fmatvec::Mat3xV& evalRF() { if(updRMV) updateR(); return RF; }
      const fmatvec::VecV& evallaF() { if(updlaF) updatelaF(); return lambdaF; }
      const fmatvec::VecV& evallaM() { if(updlaM) updatelaM(); return lambdaM; }

#ifdef HAVE_OPENMBVCPPINTERFACE
      /** \brief Visualize a force arrow */
     BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        setOpenMBVForce(ombv.createOpenMBV());
      }
      void setOpenMBVForce(const boost::shared_ptr<OpenMBV::Arrow> &arrow) { openMBVArrowF = arrow; }
#endif

    protected:
      /**
       * \brief difference vector of position, velocity and angular velocity
       */
      fmatvec::Vec3 WrP0P1, WvP0P1, WomP0P1;

      fmatvec::Mat3xV DF;

      fmatvec::Vec3 F;

      fmatvec::Mat3xV RF;

      fmatvec::VecV lambdaF, lambdaM;

      /**
       * \brief indices of forces and torques
       */
      fmatvec::Index iF;

      /**
       * \brief array in which all frames are listed, connecting bodies via a link
       */
      std::vector<Frame*> frame;

#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::Group> openMBVForceGrp;
      boost::shared_ptr<OpenMBV::Arrow> openMBVArrowF;
      boost::shared_ptr<OpenMBV::Arrow> openMBVArrowM;
#endif

      bool updPos, updVel, updFD, updF, updRMV, updlaF, updlaM;

    private:
      std::string saved_ref1, saved_ref2;
  };
}

#endif /* _LINK_MECHANICS_H_ */

