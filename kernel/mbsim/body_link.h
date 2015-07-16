/* Copyright (C) 2004-2015 MBSim Development Team
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

#ifndef _BODY_LINK_H_
#define _BODY_LINK_H_

#include "mbsim/link.h"
#include "mbsim/floating_relative_frame.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#endif

namespace MBSim {

  class RigidBody;

  class RigidBodyLink : public Link {
    protected:
      std::vector<RigidBody*> body;
      std::vector<double> ratio;
      std::vector<FloatingRelativeFrame> C;
      bool updPos, updVel, updFD, updFSV, updFMV, updRMV;
      std::vector<fmatvec::Mat3xV> DF, DM;
      std::vector<fmatvec::Vec3> F, M;
      std::vector<fmatvec::Mat3xV> RF, RM;
      fmatvec::Index iF, iM;
    public:
      RigidBodyLink(const std::string &name="");

      void calclaSize(int j) { laSize = 1; }
      void calcgSize(int j) { gSize = 1; }
      void calcgdSize(int j) { gdSize = 1; }

      void updateh(double t, int i=0);
      void updateW(double t, int i=0);
      void updateg(double t);
      void updategd(double t);
      void updatePositions(double t);
      void updateGeneralizedPositions(double t);
      void updateGeneralizedVelocities(double t);
      void updateForceDirections(double t);
      void updateSingleValuedForces(double t);
      void updateSetValuedForces(double t);
      void updateSetValuedForceDirections(double t);
      void updatewb(double t);
      const fmatvec::Mat3xV& getGlobalForceDirection(double t, int i) { if(updFD) updateForceDirections(t); return DF[i]; }
      const fmatvec::Mat3xV& getGlobalMomentDirection(double t, int i) { if(updFD) updateForceDirections(t); return DM[i]; }
      const fmatvec::Vec3& getSingleValuedForce(double t, int i) { if(updFSV) updateSingleValuedForces(t); return F[i]; }
      const fmatvec::Vec3& getSingleValuedMoment(double t, int i) { if(updFSV) updateSingleValuedForces(t); return M[i]; }
      const fmatvec::Vec3& getSetValuedForce(double t, int i) { if(updFMV) updateSetValuedForces(t); return F[i]; }
      const fmatvec::Vec3& getSetValuedMoment(double t, int i) { if(updFMV) updateSetValuedForces(t); return M[i]; }
      const fmatvec::Vec3& getForce(double t, int i) { return isSetValued()?getSetValuedForce(t,i):getSingleValuedForce(t,i); }
      const fmatvec::Vec3& getMoment(double t, int i) { return isSetValued()?getSetValuedMoment(t,i):getSingleValuedMoment(t,i); }
      const fmatvec::Mat3xV& getSetValuedForceDirection(double t, int i) { if(updRMV) updateSetValuedForceDirections(t); return RF[i]; }
      const fmatvec::Mat3xV& getSetValuedMomentDirection(double t, int i) { if(updRMV) updateSetValuedForceDirections(t); return RM[i]; }
      void updatehRef(const fmatvec::Vec &hParent, int j=0);
      void updaterRef(const fmatvec::Vec &hParent, int j=0);
      void updateWRef(const fmatvec::Mat &WParent, int j=0);
      void updateVRef(const fmatvec::Mat &WParent, int j=0);
      void connect(RigidBody *body);
      void connect(RigidBody *body, double ratio);

      std::string getType() const { return "RigidBodyLink"; }
      void init(InitStage stage);

      void plot(double t, double dt=1);

      void initializeUsingXML(xercesc::DOMElement * element);

      void resetUpToDate(); 

#ifdef HAVE_OPENMBVCPPINTERFACE
     /** \brief Visualize a force arrow */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        setOpenMBVForce(ombv.createOpenMBV());
      }
      void setOpenMBVForce(const boost::shared_ptr<OpenMBV::Arrow> &arrow) { FArrow[0]=arrow; }

      /** \brief Visualize a moment arrow */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMoment, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        setOpenMBVMoment(ombv.createOpenMBV());
      }
      void setOpenMBVMoment(const boost::shared_ptr<OpenMBV::Arrow> &arrow) { MArrow[0]=arrow; }
#endif

    protected:
#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::Group> openMBVForceGrp;
      std::vector<boost::shared_ptr<OpenMBV::Arrow> > FArrow, MArrow;
#endif

  };

}

#endif
