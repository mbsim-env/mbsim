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
      Integrator() = default;
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent) override;
  };

  class DOPRI5Integrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"DOPRI5Integrator"; }
      QString getType() const override { return "DOPRI5 integrator"; }
      PropertyDialog* createPropertyDialog() override { return new DOPRI5IntegratorPropertyDialog(this); }
  };

  class DOP853Integrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"DOP853Integrator"; }
      QString getType() const override { return "DOP853 integrator"; }
      PropertyDialog* createPropertyDialog() override { return new DOP853IntegratorPropertyDialog(this); }
  };

  class ODEXIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ODEXIntegrator"; }
      QString getType() const override { return "ODEX integrator"; }
      PropertyDialog* createPropertyDialog() override { return new ODEXIntegratorPropertyDialog(this); }
  };

  class RADAU5Integrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RADAU5Integrator"; }
      QString getType() const override { return "RADAU5 integrator"; }
      PropertyDialog* createPropertyDialog() override { return new RADAU5IntegratorPropertyDialog(this); }
  };

  class RADAUIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RADAUIntegrator"; }
      QString getType() const override { return "RADAU integrator"; }
      PropertyDialog* createPropertyDialog() override { return new RADAUIntegratorPropertyDialog(this); }
  };

  class RODASIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RODASIntegrator"; }
      QString getType() const override { return "RODAS integrator"; }
      PropertyDialog* createPropertyDialog() override { return new RODASIntegratorPropertyDialog(this); }
  };

  class SEULEXIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SEULEXIntegrator"; }
      QString getType() const override { return "SEULEX integrator"; }
      PropertyDialog* createPropertyDialog() override { return new SEULEXIntegratorPropertyDialog(this); }
  };

  class PHEM56Integrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PHEM56Integrator"; }
      QString getType() const override { return "PHEM56 integrator"; }
      PropertyDialog* createPropertyDialog() override { return new PHEM56IntegratorPropertyDialog(this); }
  };

  class LSODEIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LSODEIntegrator"; }
      QString getType() const override { return "LSODE integrator"; }
      PropertyDialog* createPropertyDialog() override { return new LSODEIntegratorPropertyDialog(this); }
  };

  class LSODAIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LSODAIntegrator"; }
      QString getType() const override { return "LSODA integrator"; }
      PropertyDialog* createPropertyDialog() override { return new LSODAIntegratorPropertyDialog(this); }
  };

  class LSODIIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LSODIIntegrator"; }
      QString getType() const override { return "LSODI integrator"; }
      PropertyDialog* createPropertyDialog() override { return new LSODIIntegratorPropertyDialog(this); }
  };

  class DASPKIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"DASPKIntegrator"; }
      QString getType() const override { return "DASPK integrator"; }
      PropertyDialog* createPropertyDialog() override { return new DASPKIntegratorPropertyDialog(this); }
  };

  class TimeSteppingIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TimeSteppingIntegrator"; }
      QString getType() const override { return "Time stepping integrator"; }
      PropertyDialog* createPropertyDialog() override { return new TimeSteppingIntegratorPropertyDialog(this); }
  };

  class ThetaTimeSteppingIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ThetaTimeSteppingIntegrator"; }
      QString getType() const override { return "Theta time stepping integrator"; }
      PropertyDialog* createPropertyDialog() override { return new ThetaTimeSteppingIntegratorPropertyDialog(this); }
  };

  class TimeSteppingSSCIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"TimeSteppingSSCIntegrator"; }
      QString getType() const override { return "Time stepping SSC integrator"; }
      PropertyDialog* createPropertyDialog() override { return new TimeSteppingSSCIntegratorPropertyDialog(this); }
  };

  class HETS2Integrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"HETS2Integrator"; }
      QString getType() const override { return "HETS2 integrator"; }
      PropertyDialog* createPropertyDialog() override { return new HETS2IntegratorPropertyDialog(this); }
  };

  class ExplicitEulerIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ExplicitEulerIntegrator"; }
      QString getType() const override { return "Explicit Euler integrator"; }
      PropertyDialog* createPropertyDialog() override { return new ExplicitEulerIntegratorPropertyDialog(this); }
  };

  class ImplicitEulerIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ImplicitEulerIntegrator"; }
      QString getType() const override { return "Implicit Euler integrator"; }
      PropertyDialog* createPropertyDialog() override { return new ImplicitEulerIntegratorPropertyDialog(this); }
  };

  class RKSuiteIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RKSuiteIntegrator"; }
      QString getType() const override { return "RKSuite integrator"; }
      PropertyDialog* createPropertyDialog() override { return new RKSuiteIntegratorPropertyDialog(this); }
  };

  class BoostOdeintDOS_RKDOPRI5 : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"BoostOdeintDOS_RKDOPRI5"; }
      QString getType() const override { return "Boost odeint DOS RKDOPRI5 integrator"; }
      PropertyDialog* createPropertyDialog() override { return new BoostOdeintDOS_RKDOPRI5PropertyDialog(this); }
  };

  class BoostOdeintDOS_BulirschStoer : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"BoostOdeintDOS_BulirschStoer"; }
      QString getType() const override { return "Boost odeint DOS Burlisch Stoer integrator"; }
      PropertyDialog* createPropertyDialog() override { return new BoostOdeintDOS_BulirschStoerPropertyDialog(this); }
  };

  class BoostOdeintDOS_Rosenbrock4 : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"BoostOdeintDOS_Rosenbrock4"; }
      QString getType() const override { return "Boost odeint DOS Rosenbrock4 integrator"; }
      PropertyDialog* createPropertyDialog() override { return new BoostOdeintDOS_Rosenbrock4PropertyDialog(this); }
  };

  class QuasiStaticIntegrator : public Integrator {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"QuasiStaticIntegrator"; }
      QString getType() const override { return "Quasi static integrator"; }
      PropertyDialog* createPropertyDialog() override { return new QuasiStaticIntegratorPropertyDialog(this); }
  };

}

#endif
