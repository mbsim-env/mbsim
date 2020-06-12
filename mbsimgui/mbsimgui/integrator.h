/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef _INTEGRATOR__H_
#define _INTEGRATOR__H_

#include "solver.h"
#include "solver_property_dialog.h"

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

namespace MBSimGUI {

  class Integrator : public Solver {
    public:
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent) override;
  };

  class DOPRI5Integrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"DOPRI5Integrator"; }
      PropertyDialog* createPropertyDialog() override { return new DOPRI5IntegratorPropertyDialog(this); }
  };

  class DOP853Integrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"DOP853Integrator"; }
      PropertyDialog* createPropertyDialog() override { return new DOP853IntegratorPropertyDialog(this); }
  };

  class ODEXIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ODEXIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new ODEXIntegratorPropertyDialog(this); }
  };

  class RADAU5Integrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RADAU5Integrator"; }
      PropertyDialog* createPropertyDialog() override { return new RADAU5IntegratorPropertyDialog(this); }
  };

  class RADAUIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RADAUIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new RADAUIntegratorPropertyDialog(this); }
  };

  class RODASIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RODASIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new RODASIntegratorPropertyDialog(this); }
  };

  class SEULEXIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SEULEXIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new SEULEXIntegratorPropertyDialog(this); }
  };

  class PHEM56Integrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PHEM56Integrator"; }
      PropertyDialog* createPropertyDialog() override { return new PHEM56IntegratorPropertyDialog(this); }
  };

  class LSODEIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LSODEIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new LSODEIntegratorPropertyDialog(this); }
  };

  class LSODAIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LSODAIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new LSODAIntegratorPropertyDialog(this); }
  };

  class LSODIIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LSODIIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new LSODIIntegratorPropertyDialog(this); }
  };

  class DASPKIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"DASPKIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new DASPKIntegratorPropertyDialog(this); }
  };

  class TimeSteppingIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TimeSteppingIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new TimeSteppingIntegratorPropertyDialog(this); }
  };

  class ThetaTimeSteppingIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ThetaTimeSteppingIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new ThetaTimeSteppingIntegratorPropertyDialog(this); }
  };

  class TimeSteppingSSCIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TimeSteppingSSCIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new TimeSteppingSSCIntegratorPropertyDialog(this); }
  };

  class HETS2Integrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"HETS2Integrator"; }
      PropertyDialog* createPropertyDialog() override { return new HETS2IntegratorPropertyDialog(this); }
  };

  class ExplicitEulerIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ExplicitEulerIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new ExplicitEulerIntegratorPropertyDialog(this); }
  };

  class ImplicitEulerIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ImplicitEulerIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new ImplicitEulerIntegratorPropertyDialog(this); }
  };

  class RKSuiteIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RKSuiteIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new RKSuiteIntegratorPropertyDialog(this); }
  };

  class BoostOdeintDOS_RKDOPRI5 : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"BoostOdeintDOS_RKDOPRI5"; }
      PropertyDialog* createPropertyDialog() override { return new BoostOdeintDOS_RKDOPRI5PropertyDialog(this); }
  };

  class BoostOdeintDOS_BulirschStoer : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"BoostOdeintDOS_BulirschStoer"; }
      PropertyDialog* createPropertyDialog() override { return new BoostOdeintDOS_BulirschStoerPropertyDialog(this); }
  };

  class BoostOdeintDOS_Rosenbrock4 : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"BoostOdeintDOS_Rosenbrock4"; }
      PropertyDialog* createPropertyDialog() override { return new BoostOdeintDOS_Rosenbrock4PropertyDialog(this); }
  };

  class QuasiStaticIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"QuasiStaticIntegrator"; }
      PropertyDialog* createPropertyDialog() override { return new QuasiStaticIntegratorPropertyDialog(this); }
  };

}

#endif
