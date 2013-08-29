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

// this implementation is very close to http://www.boost.org/doc/libs/1_49_0/doc/html/boost_asio/example/echo/blocking_udp_echo_server.cpp

#include <config.h>

#include "mbsimServer.h"
#include "interfaceIntegrator.h"
#include <sstream>
#ifdef HAVE_BOOST_ASIO_HPP
#include <boost/asio.hpp>
#endif

using namespace std;

const int max_length = 1024;


namespace MBSimInterface {

  MBSimUdpServer::MBSimUdpServer(InterfaceIntegrator *ii_, unsigned short port_) : MBSimServer(ii_), port(port_) 
  {
  }

  void MBSimUdpServer::start() {
#ifdef HAVE_BOOST_ASIO_HPP
    boost::asio::io_service io_service;
    char data[max_length];
    const char* requestIdentifier=&data[0];
    const char* interface2mbsim=&data[1];

    boost::asio::ip::udp::socket sock(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));

    do
    {
      boost::asio::ip::udp::endpoint sender_endpoint;
      size_t length = sock.receive_from(boost::asio::buffer(data, max_length), sender_endpoint);

      data[length]='\0'; // set fix message end
      ostringstream mbsim2interface;
      ii->integratorCommunication(requestIdentifier, interface2mbsim, length-1, &mbsim2interface);

      sock.send_to(boost::asio::buffer(mbsim2interface.str().c_str(), mbsim2interface.str().length()), sender_endpoint);
    } while(!ii->getExitRequest());
#endif
  }
}

