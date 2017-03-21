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

//  class TolerancePropertyFactory: public PropertyFactory {
//    public:
//      TolerancePropertyFactory(const std::string &type);
//      Property* createProperty(int i=0);
//      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
//      int getSize() const { return name.size(); }
//    protected:
//      std::vector<MBXMLUtils::FQN> name;
//      std::string type;
//  };

  class Integrator : public Solver {
    protected:
      xercesc::DOMElement *element;
    public:
      Integrator();
      virtual ~Integrator();
      virtual xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent);
      virtual std::string getType() const { return "Integrator"; }
      virtual IntegratorPropertyDialog* createPropertyDialog() {return new IntegratorPropertyDialog(this);}
  };

  class DOPRI5Integrator : public Integrator {
    public:
      DOPRI5Integrator();
      virtual xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent);
      virtual std::string getType() const { return "DOPRI5Integrator"; }
      IntegratorPropertyDialog* createPropertyDialog() {return new DOPRI5IntegratorPropertyDialog(this);}
  };

  class RADAU5Integrator : public Integrator {
    public:
      RADAU5Integrator();
      virtual std::string getType() const { return "RADAU5Integrator"; }
      IntegratorPropertyDialog* createPropertyDialog() {return new RADAU5IntegratorPropertyDialog(this);}
  };

  class LSODEIntegrator : public Integrator {
    public:
      LSODEIntegrator();
      virtual std::string getType() const { return "LSODEIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() {return new LSODEIntegratorPropertyDialog(this);}
  };

  class LSODARIntegrator : public Integrator {
    public:
      LSODARIntegrator();
      virtual std::string getType() const { return "LSODARIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() {return new LSODARIntegratorPropertyDialog(this);}
  };

  class TimeSteppingIntegrator : public Integrator {
    public:
      TimeSteppingIntegrator();
      virtual std::string getType() const { return "TimeSteppingIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() {return new TimeSteppingIntegratorPropertyDialog(this);}
  };

  class EulerExplicitIntegrator : public Integrator {
    public:
      EulerExplicitIntegrator();
      virtual std::string getType() const { return "EulerExplicitIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() {return new EulerExplicitIntegratorPropertyDialog(this);}
  };

  class RKSuiteIntegrator : public Integrator {
    public:
      RKSuiteIntegrator();
      virtual std::string getType() const { return "RKSuiteIntegrator"; }
      IntegratorPropertyDialog* createPropertyDialog() {return new RKSuiteIntegratorPropertyDialog(this);}
  };

}

#endif
