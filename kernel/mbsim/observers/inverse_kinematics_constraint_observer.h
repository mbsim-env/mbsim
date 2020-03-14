/* Copyright (C) 2004-2019 MBSim Development Team
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

#ifndef _INVERSE_KINEMATICS_CONSTRAINT_OBSERVER_H__
#define _INVERSE_KINEMATICS_CONSTRAINT_OBSERVER_H__

#include "mbsim/observers/observer.h"

#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  class InverseKinematicsConstraint;

  class InverseKinematicsConstraintObserver : public Observer {
    protected:
      InverseKinematicsConstraint* constraint;
      std::string saved_constraint;
      std::shared_ptr<OpenMBVArrow> ombv;
      std::vector<std::shared_ptr<OpenMBV::Arrow>> openMBVForce, openMBVMoment;
#ifndef SWIG
      double (InverseKinematicsConstraintObserver::*evalOMBVForceColorRepresentation[2])(int i);
      double (InverseKinematicsConstraintObserver::*evalOMBVMomentColorRepresentation[2])(int i);
#endif
      double evalNone(int i) { return 1; }
      double evalAbsoluteForce(int i);
      double evalAbsoluteMoment(int i);

    public:
      InverseKinematicsConstraintObserver(const std::string &name="");
      void setInverseKinematicsConstraint(InverseKinematicsConstraint *constraint_) { constraint = constraint_; } 

      void init(InitStage stage, const InitConfigSet &config) override;
      void plot() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::toPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),fmatvec::Vec3(std::vector<double>{-1,1,1}))(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombv = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }
  };

}  

#endif
