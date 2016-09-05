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

#ifndef _TORSIONAL_STIFFNESS_H_
#define _TORSIONAL_STIFFNESS_H_

#include "mbsim/link_mechanics.h"
#include <mbsim/frames/frame.h>
#include "mbsim/functions/function.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#endif

namespace MBSim {
  class RigidBody;
}

namespace MBSimPowertrain {

  class TorsionalStiffness : public MBSim::LinkMechanics {
    protected:
      MBSim::Function<double(double,double)> *func;
      std::vector<MBSim::RigidBody*> body;
      MBSim::Frame C;
#ifdef HAVE_OPENMBVCPPINTERFACE
      std::shared_ptr<OpenMBV::CoilSpring> coilspringOpenMBV;
#endif
    public:
      TorsionalStiffness(const std::string &name="");
      void updateh(double, int i=0);
      void updateg(double);
      void updategd(double);

      /** \brief Connect the RelativeRotationalSpringDamper to frame1 and frame2 */
      //void connect(Frame *frame1, Frame* frame2);

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      virtual bool isSingleValued() const { return true; }
      std::string getType() const { return "RotationalSpringDamper"; }
      void init(InitStage stage) {
        MBSim::LinkMechanics::init(stage);
        func->init(stage);
      }

      /** \brief Set function for the torque calculation.
       * The first input parameter to that function is the relative rotation g between frame2 and frame1.
       * The second input parameter to that function is the relative rotational velocity gd between frame2 and frame1.
       * The return value of that function is used as the torque of the RelativeRotationalSpringDamper.
       */
      void setGeneralizedForceFunction(MBSim::Function<double(double,double)> *func_) {
        func=func_;
        func->setParent(this);
        func->setName("GeneralizedForce");
      }

      /** \brief Set a projection direction for the resulting torque
       * If this function is not set, or frame is NULL, than torque calculated by setForceFunction
       * is applied on the two connected frames in the direction of the two connected frames.
       * If this function is set, than this torque is first projected in direction dir and then applied on
       * the two connected frames in the projected direction; (!) this might induce violation of the global equality of torques (!).
       * The direction vector dir is given in coordinates of frame refFrame.
       */
      void setRigidBodyFirstSide(MBSim::RigidBody* body_) { body[0] = body_; }
      void setRigidBodySecondSide(MBSim::RigidBody* body_) { body[1] = body_; }

      void plot();
      void initializeUsingXML(xercesc::DOMElement *element);

      void updatehRef(const fmatvec::Vec &hParent, int j=0);

#ifdef HAVE_OPENMBVCPPINTERFACE
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVCoilSpring, MBSim::tag, (optional (numberOfCoils,(int),3)(springRadius,(double),1)(crossSectionRadius,(double),-1)(nominalLength,(double),-1)(type,(OpenMBV::CoilSpring::Type),OpenMBV::CoilSpring::tube)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        MBSim::OpenMBVCoilSpring ombv(springRadius,crossSectionRadius,1,numberOfCoils,nominalLength,type,diffuseColor,transparency);
        coilspringOpenMBV=ombv.createOpenMBV();
      }

      /** \brief Visualize a force arrow acting on each of both connected frames */
     BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, MBSim::tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
       MBSim::OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        std::vector<bool> which; which.resize(2, true);
        LinkMechanics::setOpenMBVForceArrow(ombv.createOpenMBV(), which);
      }

      /** \brief Visualize a torque arrow acting on each of both connected frames */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMoment, MBSim::tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        MBSim::OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toDoubleHead,referencePoint,scaleLength,scaleSize);
        std::vector<bool> which; which.resize(2, true);
        LinkMechanics::setOpenMBVMomentArrow(ombv.createOpenMBV(), which);
      }
#endif
    private:
      std::string saved_body1, saved_body2;
  };

}

#endif 

