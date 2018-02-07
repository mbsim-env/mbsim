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
      IntegratorPropertyDialog* createPropertyDialog() override { return new IntegratorPropertyDialog(this); }
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMINT; }
  };

  class DOPRI5Integrator : public Integrator {
    public:
      QString getType() const override { return "DOPRI5Integrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new DOPRI5IntegratorPropertyDialog(this); }
  };

  class DOP853Integrator : public Integrator {
    public:
      QString getType() const override { return "DOP853Integrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new DOP853IntegratorPropertyDialog(this); }
  };

  class RADAU5Integrator : public Integrator {
    public:
      QString getType() const override { return "RADAU5Integrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new RADAU5IntegratorPropertyDialog(this); }
  };

  class ODEXIntegrator : public Integrator {
    public:
      QString getType() const override { return "ODEXIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new ODEXIntegratorPropertyDialog(this); }
  };

  class LSODEIntegrator : public Integrator {
    public:
      QString getType() const override { return "LSODEIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new LSODEIntegratorPropertyDialog(this); }
  };

  class LSODARIntegrator : public Integrator {
    public:
      QString getType() const override { return "LSODARIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new LSODARIntegratorPropertyDialog(this); }
  };

  class LSODKRIntegrator : public Integrator {
    public:
      QString getType() const override { return "LSODKRIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new LSODKRIntegratorPropertyDialog(this); }
  };

  class TimeSteppingIntegrator : public Integrator {
    public:
      QString getType() const override { return "TimeSteppingIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new TimeSteppingIntegratorPropertyDialog(this); }
  };

  class ThetaTimeSteppingIntegrator : public Integrator {
    public:
      QString getType() const override { return "ThetaTimeSteppingIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new ThetaTimeSteppingIntegratorPropertyDialog(this); }
  };

  class TimeSteppingSSCIntegrator : public Integrator {
    public:
      QString getType() const override { return "TimeSteppingSSCIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new TimeSteppingSSCIntegratorPropertyDialog(this); }
  };

  class HETS2Integrator : public Integrator {
    public:
      QString getType() const override { return "HETS2Integrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new HETS2IntegratorPropertyDialog(this); }
  };

  class EulerExplicitIntegrator : public Integrator {
    public:
      QString getType() const override { return "EulerExplicitIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new EulerExplicitIntegratorPropertyDialog(this); }
  };

  class EulerImplicitIntegrator : public Integrator {
    public:
      QString getType() const override { return "EulerImplicitIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new EulerImplicitIntegratorPropertyDialog(this); }
  };

  class RKSuiteIntegrator : public Integrator {
    public:
      QString getType() const override { return "RKSuiteIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new RKSuiteIntegratorPropertyDialog(this); }
  };

  class BoostOdeintDOS_RKDOPRI5 : public Integrator {
    public:
      QString getType() const override { return "BoostOdeintDOS_RKDOPRI5"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new BoostOdeintDOS_RKDOPRI5PropertyDialog(this); }
  };

  class BoostOdeintDOS_BulirschStoer : public Integrator {
    public:
      QString getType() const override { return "BoostOdeintDOS_BulirschStoer"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new BoostOdeintDOS_BulirschStoerPropertyDialog(this); }
  };

  class BoostOdeintDOS_Rosenbrock4 : public Integrator {
    public:
      QString getType() const override { return "BoostOdeintDOS_Rosenbrock4"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new BoostOdeintDOS_Rosenbrock4PropertyDialog(this); }
  };

  class QuasiStaticIntegrator : public Integrator {
    public:
      QString getType() const override { return "QuasiStaticIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override { return new QuasiStaticIntegratorPropertyDialog(this); }
  };

}

#endif
