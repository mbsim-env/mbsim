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
      IntegratorPropertyDialog* createPropertyDialog() override {return new IntegratorPropertyDialog(this);}
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMINT; }
  };

  class DOPRI5Integrator : public Integrator {
    public:
      QString getType() const override { return "DOPRI5Integrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override {return new DOPRI5IntegratorPropertyDialog(this);}
  };

  class RADAU5Integrator : public Integrator {
    public:
      QString getType() const override { return "RADAU5Integrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override {return new RADAU5IntegratorPropertyDialog(this);}
  };

  class LSODEIntegrator : public Integrator {
    public:
      QString getType() const override { return "LSODEIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override {return new LSODEIntegratorPropertyDialog(this);}
  };

  class LSODARIntegrator : public Integrator {
    public:
      QString getType() const override { return "LSODARIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override {return new LSODARIntegratorPropertyDialog(this);}
  };

  class TimeSteppingIntegrator : public Integrator {
    public:
      QString getType() const override { return "TimeSteppingIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override {return new TimeSteppingIntegratorPropertyDialog(this);}
  };

  class EulerExplicitIntegrator : public Integrator {
    public:
      QString getType() const override { return "EulerExplicitIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override {return new EulerExplicitIntegratorPropertyDialog(this);}
  };

  class RKSuiteIntegrator : public Integrator {
    public:
      QString getType() const override { return "RKSuiteIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() override {return new RKSuiteIntegratorPropertyDialog(this);}
  };

}

#endif
