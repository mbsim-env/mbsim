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

// this implementation is very close to http://www.boost.org/doc/libs/1_49_0/doc/html/boost_asio/example/echo/blocking_tcp_echo_server.cpp

#include <config.h>

#include "mbsim_server.h"
#include "defines.h"
#include "interface_integrator.h"
#include "mbsim/element.h"
#include <sstream>
#ifdef HAVE_BOOST_ASIO_HPP
#include <boost/asio.hpp>
#endif

using namespace std;
using namespace MBXMLUtils;

const int max_length = 1048576;

namespace MBSimInterface {

  MBSimTcpServer::MBSimTcpServer(InterfaceIntegrator *ii_) : MBSimServer(ii_), port(0), outputPrecision(18) 
  {
  }

  void MBSimTcpServer::start() {
#ifdef HAVE_BOOST_ASIO_HPP
    boost::asio::io_service io_service;
    char data[max_length];
    const char* requestIdentifier=&data[0];
    const char* interface2mbsim=&data[1];

    boost::asio::ip::tcp::acceptor tcp_acceptor(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port));
    boost::asio::ip::tcp::socket socket(io_service);
    tcp_acceptor.accept(socket);

    ostringstream mbsim2interface;
    mbsim2interface.precision(outputPrecision);
    mbsim2interface.setf( std::ios::scientific );

    do
    {
      boost::system::error_code error;
      size_t length = socket.read_some(boost::asio::buffer(data), error);
      if (error == boost::asio::error::eof)
        break; // Connection closed cleanly by peer.
      else if (error)
        throw boost::system::system_error(error); // Some other error.

      data[length]='\0'; // set fix message end
      mbsim2interface.str(std::string());
      ii->integratorCommunication(requestIdentifier, interface2mbsim, length-1, &mbsim2interface);

      boost::asio::write(socket, boost::asio::buffer(mbsim2interface.str().c_str(), mbsim2interface.str().length()));
    } while(!ii->getExitRequest());
#endif
  }

  void MBSimTcpServer::initializeUsingXML(xercesc::DOMElement *element) {
    xercesc::DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIMINTERFACE%"port");
    setPort(MBSim::Element::getInt(e));
    e=E(element)->getFirstElementChildNamed(MBSIMINTERFACE%"outputPrecision");
    if (e)
      setOutputPrecision(MBSim::Element::getInt(e));
  }

}

