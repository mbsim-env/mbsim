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

namespace MBSimInterface {

  class InterfaceIntegrator;

  class MBSimServer {
    public:
      MBSimServer() {}
      MBSimServer(InterfaceIntegrator* ii_) {ii=ii_; }
      virtual void start() {};
    protected:
      InterfaceIntegrator* ii;
  };

  class MBSimTcpServer : public MBSimServer  {
    public:
      MBSimTcpServer(InterfaceIntegrator *ii, unsigned short port);
      void start();
    private:
      unsigned int port;
  };

  class MBSimUdpServer : public MBSimServer  {
    public:
      MBSimUdpServer(InterfaceIntegrator *ii, unsigned short port);
      void start();
    private:
      unsigned int port;
  };

}

#endif
