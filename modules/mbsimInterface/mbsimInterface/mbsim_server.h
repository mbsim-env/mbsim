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

#ifndef _MBSIMSERVER_H_
#define _MBSIMSERVER_H_

#include <mbxmlutilshelper/dom.h>

namespace MBSimInterface {

  const MBXMLUtils::NamespaceURI MBSIMINTERFACE("http://www.mbsim-env.de/MBSimInterface");

  class InterfaceIntegrator;

  class MBSimServer {
    public:
      MBSimServer(InterfaceIntegrator* ii_) {ii=ii_; }

      virtual void initializeUsingXML(xercesc::DOMElement *element) {};

      virtual void start() {};
    protected:
      InterfaceIntegrator* ii;
  };

  class MBSimTcpServer : public MBSimServer  {
    public:
      MBSimTcpServer(InterfaceIntegrator *ii);
      void setPort(unsigned short port_) {port=port_; }
      void setOutputPrecision(unsigned int p) {outputPrecision=p; }
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      void start();
    private:
      unsigned int port;
      unsigned int outputPrecision;
  };

  class MBSimUdpServer : public MBSimServer  {
    public:
      MBSimUdpServer(InterfaceIntegrator *ii);
      void setPort(unsigned short port_) {port=port_; }
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      void start();
    private:
      unsigned int port;
  };

}

#endif
