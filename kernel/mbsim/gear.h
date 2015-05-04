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

#include "mbsim/mechanical_link.h"
#include "mbsim/floating_relative_frame.h"
#include "mbsim/rigid_body.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#endif

namespace MBSim {

  class Transmission;

  class Gear : public MechanicalLink {
    protected:
      Function<double(double,double)> *func;
      std::vector<RigidBody*> body;
      std::vector<double> ratio;
      std::vector<FloatingRelativeFrame> C;
      std::string saved_DependentBody;
      std::vector<std::string> saved_IndependentBody;
    public:
      Gear(const std::string &name="");
      void updateh(double t, int i=0);
      void updateW(double t, int i=0);
      void updateg(double t);
      void updategd(double t);
      void updatePositions(double t);
      void updateVelocities(double t);
      void updateGeneralizedSingleValuedForces(double t);
      void updateGeneralizedSetValuedForces(double t);
      void updatewb(double t);
      void updatehRef(const fmatvec::Vec &hParent, int j=0);
      void updateWRef(const fmatvec::Mat &WParent, int j=0);
      void setDependentBody(RigidBody* body_) {body[0] = body_;}
      void addTransmission(const Transmission &transmission);

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      std::string getType() const { return "Gear"; }
      void init(InitStage stage);
      bool isSetValued() const;
      bool isSingleValued() const { return not(isSetValued()); }
      virtual void calclaSize(int j);
      virtual void calcgSize(int j);
      virtual void calcgdSize(int j);

      void setGeneralizedForceFunction(Function<double(double,double)> *func_) {
        func=func_;
        func->setParent(this);
        func->setName("GeneralizedForce");
      }

      void plot(double t, double dt=1);

      void initializeUsingXML(xercesc::DOMElement * element);

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

    void resetUpToDate();

    protected:
#ifdef HAVE_OPENMBVCPPINTERFACE
      std::vector<boost::shared_ptr<OpenMBV::Arrow> > FArrow, MArrow;
#endif

  };

}

#endif
