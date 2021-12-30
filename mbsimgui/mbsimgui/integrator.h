/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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
    MBSIMGUI_OBJECTFACTORY_CLASS(Integrator, Solver, MBSIM%"Integrator", "Integrator");
    public:
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent) override;
  };

  class DOPRI5Integrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(DOPRI5Integrator, Integrator, MBSIM%"DOPRI5Integrator", "DOPRI5 integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new DOPRI5IntegratorPropertyDialog(this); }
  };

  class DOP853Integrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(DOP853Integrator, Integrator, MBSIM%"DOP853Integrator", "DOP853 integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new DOP853IntegratorPropertyDialog(this); }
  };

  class ODEXIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(ODEXIntegrator, Integrator, MBSIM%"ODEXIntegrator", "ODEX integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new ODEXIntegratorPropertyDialog(this); }
  };

  class RADAU5Integrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(RADAU5Integrator, Integrator, MBSIM%"RADAU5Integrator", "RADAU5 integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new RADAU5IntegratorPropertyDialog(this); }
  };

  class RADAUIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(RADAUIntegrator, Integrator, MBSIM%"RADAUIntegrator", "RADAU integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new RADAUIntegratorPropertyDialog(this); }
  };

  class RODASIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(RODASIntegrator, Integrator, MBSIM%"RODASIntegrator", "RODAS integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new RODASIntegratorPropertyDialog(this); }
  };

  class SEULEXIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(SEULEXIntegrator, Integrator, MBSIM%"SEULEXIntegrator", "SEULEX integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new SEULEXIntegratorPropertyDialog(this); }
  };

  class PHEM56Integrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(PHEM56Integrator, Integrator, MBSIM%"PHEM56Integrator", "PHEM56 integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new PHEM56IntegratorPropertyDialog(this); }
  };

  class LSODEIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(LSODEIntegrator, Integrator, MBSIM%"LSODEIntegrator", "LSODE integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new LSODEIntegratorPropertyDialog(this); }
  };

  class LSODAIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(LSODAIntegrator, Integrator, MBSIM%"LSODAIntegrator", "LSODA integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new LSODAIntegratorPropertyDialog(this); }
  };

  class LSODIIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(LSODIIntegrator, Integrator, MBSIM%"LSODIIntegrator", "LSODI integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new LSODIIntegratorPropertyDialog(this); }
  };

  class DASPKIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(DASPKIntegrator, Integrator, MBSIM%"DASPKIntegrator", "DASPK integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new DASPKIntegratorPropertyDialog(this); }
  };

  class TimeSteppingIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(TimeSteppingIntegrator, Integrator, MBSIM%"TimeSteppingIntegrator", "Time stepping integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new TimeSteppingIntegratorPropertyDialog(this); }
  };

  class ThetaTimeSteppingIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(ThetaTimeSteppingIntegrator, Integrator, MBSIM%"ThetaTimeSteppingIntegrator", "Theta time stepping integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new ThetaTimeSteppingIntegratorPropertyDialog(this); }
  };

  class TimeSteppingSSCIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(TimeSteppingSSCIntegrator, Integrator, MBSIM%"TimeSteppingSSCIntegrator", "Time stepping SSC integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new TimeSteppingSSCIntegratorPropertyDialog(this); }
  };

  class HETS2Integrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(HETS2Integrator, Integrator, MBSIM%"HETS2Integrator", "HETS2 integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new HETS2IntegratorPropertyDialog(this); }
  };

  class ExplicitEulerIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(ExplicitEulerIntegrator, Integrator, MBSIM%"ExplicitEulerIntegrator", "Explicit Euler integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new ExplicitEulerIntegratorPropertyDialog(this); }
  };

  class ImplicitEulerIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(ImplicitEulerIntegrator, Integrator, MBSIM%"ImplicitEulerIntegrator", "Implicit Euler integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new ImplicitEulerIntegratorPropertyDialog(this); }
  };

  class RKSuiteIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(RKSuiteIntegrator, Integrator, MBSIM%"RKSuiteIntegrator", "RKSuite integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new RKSuiteIntegratorPropertyDialog(this); }
  };

  class BoostOdeintDOS_RKDOPRI5 : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(BoostOdeintDOS_RKDOPRI5, Integrator, MBSIM%"BoostOdeintDOS_RKDOPRI5", "Boost odeint DOS RKDOPRI5 integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new BoostOdeintDOS_RKDOPRI5PropertyDialog(this); }
  };

  class BoostOdeintDOS_BulirschStoer : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(BoostOdeintDOS_BulirschStoer, Integrator, MBSIM%"BoostOdeintDOS_BulirschStoer", "Boost odeint DOS Burlisch Stoer integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new BoostOdeintDOS_BulirschStoerPropertyDialog(this); }
  };

  class BoostOdeintDOS_Rosenbrock4 : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(BoostOdeintDOS_Rosenbrock4, Integrator, MBSIM%"BoostOdeintDOS_Rosenbrock4", "Boost odeint DOS Rosenbrock4 integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new BoostOdeintDOS_Rosenbrock4PropertyDialog(this); }
  };

  class QuasiStaticIntegrator : public Integrator {
    MBSIMGUI_OBJECTFACTORY_CLASS(QuasiStaticIntegrator, Integrator, MBSIM%"QuasiStaticIntegrator", "Quasi static integrator");
    public:
      PropertyDialog* createPropertyDialog() override { return new QuasiStaticIntegratorPropertyDialog(this); }
  };

}

#endif
