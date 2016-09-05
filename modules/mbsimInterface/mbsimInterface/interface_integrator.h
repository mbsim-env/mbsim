/* Copyright (C) 2013 Markus Schneider

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
 * Contact:
 *   markus.ms.schneider@live.de
 *
 */

#ifndef _INTERFACE_INTEGRATOR_H_
#define _INTERFACE_INTEGRATOR_H_

#include <mbsim/integrators/integrator.h>

namespace MBSimControl {
  class Signal;
  class ExternSignalSource;
}

namespace MBSimInterface {

  extern const MBXMLUtils::NamespaceURI MBSIMINTERFACE;

  class MBSimServer;

  /** \brief Dummy-Integrator InterfaceIntegrator
    This integrator is an interface for other integration tool. */
  class InterfaceIntegrator : public MBSimIntegrator::Integrator {

    protected:
      std::ofstream integPlot;

    public:

      InterfaceIntegrator();
      ~InterfaceIntegrator() {}

      void setMBSimServer(MBSimServer* m_) {mbsimServer=m_; }

      void integrate(MBSim::DynamicSystemSolver& system);

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

      virtual std::string getType() const { return "InterfaceIntegrator"; }

      void integratorCommunication(const char* requestIdentifier, const char* interface2mbsim, unsigned int interface2mbsimLength, std::ostringstream* mbsim2interface);
      bool getExitRequest() {return exitRequest; }

    private:
      // get values
      void getz(double** z_);
      void getzdot(double** zdot_);
      void getsv(double** sv_);

      // set values
      void setTime(double t_);
      void setz(const fmatvec::Vec& z_);

      // internal routines
      int zSize, svSize;

      bool printCommunication;

      void dumpMemory(std::ostringstream *out, void *p, unsigned int N);
      void double2str(std::ostringstream *out, double *p, unsigned int N);
      void int2str(std::ostringstream *out, int *p, unsigned int N);

      bool exitRequest;

      MBSimServer* mbsimServer;
      std::vector<std::string> outputSignalRef, inputSignalRef, outputSignalName, inputSignalName;
      std::vector<MBSimControl::Signal*> outputSignal;
      std::vector<MBSimControl::ExternSignalSource*> inputSignal;
      fmatvec::VecInt outputSignalSize, inputSignalSize;
      void resolveInputOutputNames();
      fmatvec::Vec outputVector, inputVector;
  };
}

#endif // _SERVER_INTEGRATOR_H_
