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


#include <config.h>

#include <mbsim/dynamic_system_solver.h>
#include "server_integrator.h"
#include "server_integrator_messages.h"
#include <fstream>

#ifndef NO_ISO_14882
using namespace std;
#endif

#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;
const int max_length = 1024;
typedef boost::shared_ptr<tcp::socket> socket_ptr;

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;

namespace MBSimInterface {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(MBSim::Integrator, ServerIntegrator, MBSIMINTNS"ServerIntegrator")

  bool serverExitRequest=false;

  void session(ServerIntegrator *si, socket_ptr sock);
  void server(ServerIntegrator *si, boost::asio::io_service& io_service, short port);

  ServerIntegrator::ServerIntegrator(): MBSim::Integrator(), IP(""), port(""), zSize(0), nsv(0), t(tStart) {
  }

  void ServerIntegrator::initializeUsingXML(TiXmlElement *element) {
    /* TODO
    Integrator::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMINTNS"IP");
    if(e) setIP(e->ValueStr()); //Element::getString(e));
    e=element->FirstChildElement(MBSIMINTNS"port");
    if(e) setPort(e->ValueStr()); //Element::getString(e));
    */
  }

  TiXmlElement* ServerIntegrator::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
    /* TODO
    addElementText(ele0,MBSIMINTNS"IP",IP);
    addElementText(ele0,MBSIMINTNS"port",port);
    */
    return ele0;
  }

  void ServerIntegrator::integrate(MBSim::DynamicSystemSolver& system_) {
    system = &system_;

    zSize=system->getzSize();
    z.resize(zSize, INIT, 0);
    if((this->getInitialState()).size())
      z = this->getInitialState();
    else
      system->initz(z);
    system->computeInitialCondition();

    nsv=system->getsvSize();
    jsv.resize(nsv, INIT, 0);

    integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);

    try {
      boost::asio::io_service io_service;
      server(this, io_service, 4567);
    }
    catch (std::exception& e) {
      std::cerr << "Exception: " << e.what() << "\n";
    }

    integPlot.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }

  MBSim::DynamicSystemSolver* ServerIntegrator::getDynamicSystemSolver() {
    return system;
  }



  void session(ServerIntegrator *si, socket_ptr sock) {
    try {
      while (!serverExitRequest) {
        char receivedData[max_length];

        boost::system::error_code error;
        size_t receivedLength = sock->read_some(boost::asio::buffer(receivedData), error);
        if (error == boost::asio::error::eof) {
          break; // Connection closed cleanly by peer.
        }
        else if (error) {
          throw boost::system::system_error(error); // Some other error.
        }

        stringstream ss;
        ss << receivedData;
        string s=ss.str().substr(0, receivedLength);
        size_t sL=s.length();
        unsigned int messageID=atoi((s.substr(0, 2)).c_str());
        string message=s.substr(2, sL-2);

        ostringstream sendingData;
        sendingData.precision(18);
        sendingData.setf(std::ios::scientific);

        switch (messageID) {
          case _SI_getTime_SI_:
            sendingData << *(si->getTime());
            break;

          case _SI_getzSize_SI_:
            sendingData << *(si->getzSize());
            break;
          case _SI_getz_SI_:
            sendingData << "[";
            for (int iz=0; iz<(*(si->getzSize()))-1; iz++)
              sendingData << si->getz()(iz) << ";";
            sendingData << si->getz()(*(si->getzSize())-1) << "]";
            break;
          case _SI_getzdot_SI_:
            {
              Vec zd(*(si->getzSize()));
              si->getDynamicSystemSolver()->zdot(si->getz(), zd, *(si->getTime()));
              sendingData << "[";
              for (int iz=0; iz<(*(si->getzSize()))-1; iz++)
                sendingData << zd(iz) << ";";
              sendingData << zd(*(si->getzSize())-1) << "]";
            }
            break;

          case _SI_getsvSize_SI_:
            sendingData << *(si->getsvSize());
            break;
          case _SI_getsv_SI_:
            {
              Vec sv(*(si->getsvSize()));
              VecInt jsv(*(si->getsvSize()));
              si->getDynamicSystemSolver()->getsv(si->getz(), sv, *(si->getTime()));
              sendingData << "[";
              for (int isv=0; isv<(*(si->getsvSize()))-1; isv++)
              {
                const int svi=round(sv(isv));
                sendingData << svi << ";";
                jsv(isv)=svi;
              }
              const int svi=round(sv(*(si->getsvSize())-1));
              sendingData << svi << "]";
              jsv(*(si->getsvSize())-1)=svi;
              si->setjsv(jsv);
            }
            break;

          case _SI_setTime_SI_:
            {
              const Vec t=Vec(message.c_str());
              if (t.size()!=1)
                throw MBSim::MBSimError("wrong size of given double t!");
              si->setTime(t(0));
            }
            break;

          case _SI_setz_SI_:
            {
              const Vec z(message.c_str());
              if (z.size()!=(si->getz()).size())
                throw MBSim::MBSimError("wrong size of given vector z!");
              si->setz(z);
            }
            break;

          case _SI_plot_SI_:
            si->getDynamicSystemSolver()->plot(si->getz(), *(si->getTime()));
            break;

          case _SI_shift_SI_:
            si->getDynamicSystemSolver()->shift(si->getz(), si->getjsv(), *(si->getTime()));
            break;

          case _SI_exitRequest_SI_:
            serverExitRequest=true;
            break;

          default:
            cerr << "Unknown IPC message!!!" << endl;
        }

        sendingData << "#"; // the sign '#' is used for signalizing the end of message transfer
        boost::asio::write(*sock, boost::asio::buffer(sendingData.str().c_str(), sendingData.str().size()));
      }
    }
    catch (std::exception& e) {
      std::cerr << "Exception in thread: " << e.what() << "\n";
    }
  }

  void server(ServerIntegrator *si, boost::asio::io_service& io_service, short port) {
    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
    while (!serverExitRequest) {
      socket_ptr sock(new tcp::socket(io_service));
      a.accept(*sock);
      boost::thread t(boost::bind(session, si, sock));
    }
  }
}
