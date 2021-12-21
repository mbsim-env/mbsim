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

namespace MBSimGUI {

  class Constraint : public Element {
    public:
      Constraint();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"Constraint"; }
      QMenu* createContextMenu() override { return new ConstraintContextMenu(this); }
  };

  class UnknownConstraint : public Constraint {
    public:
      QString getType() const override { return "Unknown constraint"; }
      PropertyDialog* createPropertyDialog() override { return new UnknownItemPropertyDialog(this); }
  };

  class MechanicalConstraint : public Constraint {
  };

  class GeneralizedConstraint : public MechanicalConstraint {
  };

  class GeneralizedGearConstraint : public GeneralizedConstraint {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"GeneralizedGearConstraint"; }
      QString getType() const override { return "Generalized gear constraint"; }
      PropertyDialog* createPropertyDialog() override { return new GeneralizedGearConstraintPropertyDialog(this); }
  };

  class GeneralizedDualConstraint : public GeneralizedConstraint {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"GeneralizedDualConstraint"; }
      QString getType() const override { return "Generalized dual constraint"; }
      PropertyDialog* createPropertyDialog() override { return new GeneralizedDualConstraintPropertyDialog(this); }
  };

  class GeneralizedPositionConstraint : public GeneralizedDualConstraint {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"GeneralizedPositionConstraint"; }
      QString getType() const override { return "Generalized position constraint"; }
      PropertyDialog* createPropertyDialog() override { return new GeneralizedPositionConstraintPropertyDialog(this); }
  };

  class GeneralizedVelocityConstraint : public GeneralizedDualConstraint {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"GeneralizedVelocityConstraint"; }
      QString getType() const override { return "Generalized velocity constraint"; }
      PropertyDialog* createPropertyDialog() override { return new GeneralizedVelocityConstraintPropertyDialog(this); }
  };

  class GeneralizedAccelerationConstraint : public GeneralizedDualConstraint {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"GeneralizedAccelerationConstraint"; }
      QString getType() const override { return "Generalized acceleration constraint"; }
      PropertyDialog* createPropertyDialog() override { return new GeneralizedAccelerationConstraintPropertyDialog(this); }
  };

  class JointConstraint : public MechanicalConstraint {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"JointConstraint"; }
      QString getType() const override { return "Joint constraint"; }
      PropertyDialog* createPropertyDialog() override { return new JointConstraintPropertyDialog(this); }
  };

  class GeneralizedConnectionConstraint : public GeneralizedDualConstraint {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"GeneralizedConnectionConstraint"; }
      QString getType() const override { return "Generalized connection constraint"; }
      PropertyDialog* createPropertyDialog() override { return new GeneralizedConnectionConstraintPropertyDialog(this); }
  };

  class InverseKinematicsConstraint : public Constraint {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"InverseKinematicsConstraint"; }
      QString getType() const override { return "Inverse kinematics constraint"; }
      PropertyDialog* createPropertyDialog() override { return new InverseKinematicsConstraintPropertyDialog(this); }
  };

}

#endif
