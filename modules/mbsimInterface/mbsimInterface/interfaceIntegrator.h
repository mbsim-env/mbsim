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

namespace MBSimInterface {

  /** \brief Dummy-Integrator InterfaceIntegrator
    This integrator is an interface for other integration tool. */
  class InterfaceIntegrator : public MBSim::Integrator {

    protected:
      std::ofstream integPlot;

    public:

      InterfaceIntegrator();
      ~InterfaceIntegrator() {}

      void integrate(MBSim::DynamicSystemSolver& system);

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);

      virtual std::string getType() const { return "InterfaceIntegrator"; }

      void setIP(std::string IP_) {IP=IP_; }
      void setPort(std::string port_) {port=port_; }
      
      void integratorCommunication(const char* requestIdentifier, const char* interface2mbsim, unsigned int interface2mbsimLength, std::ostringstream* mbsim2interface);
      bool getExitRequest() {return exitRequest; }

    private:
      // get values
      void getz(double** z_);
      void getzdot(double** zdot_);
      void getsv(double** sv_);

      // set values
      void setTime(double t_) {t=t_; }
      void setz(const fmatvec::Vec& z_) {z << z_; }

      // internal routines
      std::string IP, port;
      int zSize, svSize;
      double t;
      fmatvec::Vec z, zd, sv;
      fmatvec::VecInt jsv;

      bool printCommunication;

      void dumpMemory(std::ostringstream *out, void *p, unsigned int N);
      void double2str(std::ostringstream *out, double *p, unsigned int N);

      bool exitRequest;
  };
}

#endif // _SERVER_INTEGRATOR_H_
