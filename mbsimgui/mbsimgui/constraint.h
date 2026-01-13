/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012-2016 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#ifndef _CONSTRAINT__H_
#define _CONSTRAINT__H_

#include "object.h"
#include "constraint_property_dialog.h"

namespace MBSimGUI {

  class Constraint : public Element {
    MBSIMGUI_OBJECTFACTORY_CLASS(Constraint, Element, MBSIM%"Constraint", "Constraint");
    public:
      Constraint();
      QMenu* createContextMenu() override { return new ConstraintContextMenu(this); }
  };

  class UnknownConstraint : public Constraint {
    MBSIMGUI_OBJECTFACTORY_CLASS(UnknownConstraint, Constraint, MBSIM%"UnknownConstraint_dummy", "Unknown constraint");
    public:
      UnknownConstraint();
      PropertyDialog* createPropertyDialog() override { return new UnknownItemPropertyDialog(this); }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement *element) override;
  };

  class MechanicalConstraint : public Constraint {
    MBSIMGUI_OBJECTFACTORY_CLASS(MechanicalConstraint, Constraint, MBSIM%"MechanicalConstraint", "Mechanical constraint");
  };

  class GeneralizedConstraint : public MechanicalConstraint {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedConstraint, MechanicalConstraint, MBSIM%"GeneralizedConstraint", "Generalized constraint");
  };

  class GeneralizedGearConstraint : public GeneralizedConstraint {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedGearConstraint, GeneralizedConstraint, MBSIM%"GeneralizedGearConstraint", "Generalized gear constraint");
    public:
      PropertyDialog* createPropertyDialog() override { return new GeneralizedGearConstraintPropertyDialog(this); }
  };

  class GeneralizedDualConstraint : public GeneralizedConstraint {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedDualConstraint, GeneralizedConstraint, MBSIM%"GeneralizedDualConstraint", "Generalized dual constraint");
    public:
      PropertyDialog* createPropertyDialog() override { return new GeneralizedDualConstraintPropertyDialog(this); }
  };

  class GeneralizedPositionConstraint : public GeneralizedDualConstraint {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedPositionConstraint, GeneralizedDualConstraint, MBSIM%"GeneralizedPositionConstraint", "Generalized position constraint");
    public:
      PropertyDialog* createPropertyDialog() override { return new GeneralizedPositionConstraintPropertyDialog(this); }
  };

  class GeneralizedVelocityConstraint : public GeneralizedDualConstraint {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedVelocityConstraint, GeneralizedDualConstraint, MBSIM%"GeneralizedVelocityConstraint", "Generalized velocity constraint");
    public:
      PropertyDialog* createPropertyDialog() override { return new GeneralizedVelocityConstraintPropertyDialog(this); }
  };

  class GeneralizedAccelerationConstraint : public GeneralizedDualConstraint {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedAccelerationConstraint, GeneralizedDualConstraint, MBSIM%"GeneralizedAccelerationConstraint", "Generalized acceleration constraint");
    public:
      PropertyDialog* createPropertyDialog() override { return new GeneralizedAccelerationConstraintPropertyDialog(this); }
  };

  class JointConstraint : public MechanicalConstraint {
    MBSIMGUI_OBJECTFACTORY_CLASS(JointConstraint, MechanicalConstraint, MBSIM%"JointConstraint", "Joint constraint");
    public:
      PropertyDialog* createPropertyDialog() override { return new JointConstraintPropertyDialog(this); }
  };

  class GeneralizedConnectionConstraint : public GeneralizedDualConstraint {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedConnectionConstraint, GeneralizedDualConstraint, MBSIM%"GeneralizedConnectionConstraint", "Generalized connection constraint");
    public:
      PropertyDialog* createPropertyDialog() override { return new GeneralizedConnectionConstraintPropertyDialog(this); }
  };

  class InverseKinematicsConstraint : public Constraint {
    MBSIMGUI_OBJECTFACTORY_CLASS(InverseKinematicsConstraint, Constraint, MBSIM%"InverseKinematicsConstraint", "Inverse kinematics constraint");
    public:
      PropertyDialog* createPropertyDialog() override { return new InverseKinematicsConstraintPropertyDialog(this); }
  };

}

#endif
