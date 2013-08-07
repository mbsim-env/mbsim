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

#ifndef _SERVER_INTEGRATOR_H_
#define _SERVER_INTEGRATOR_H_

#include "integrator.h"

namespace MBSim {

  /** \brief Dummy-Integrator ServerIntegrator
    This integrator is an interface for other integration tool. */
  class ServerIntegrator : public Integrator {

    protected:
      std::ofstream integPlot;

    public:

      ServerIntegrator();
      ~ServerIntegrator() {}

      void integrate(DynamicSystemSolver& system);

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);

      virtual std::string getType() const { return "ServerIntegrator"; }

      void setIP(std::string IP_) {IP=IP_; }
      void setPort(std::string port_) {port=port_; }

      DynamicSystemSolver* getDynamicSystemSolver() {return system; }

      int* getzSize() {return &zSize; }
      int* getsvSize() {return &nsv; }
      double* getTime() {return &t; }
      void setTime(double t_) {t=t_; }
      fmatvec::Vec& getz() {return z; }
      void setz(const fmatvec::Vec& z_) {z << z_; }
      fmatvec::VecInt& getjsv() {return jsv; }
      void setjsv(const fmatvec::VecInt& jsv_) {jsv << jsv_; }

    private:
      std::string IP, port;
      int zSize, nsv;
      double t;
      fmatvec::Vec z;
      fmatvec::VecInt jsv;
  };
}

#endif // _SERVER_INTEGRATOR_H_
