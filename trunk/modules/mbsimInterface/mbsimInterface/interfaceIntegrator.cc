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
#include "interfaceIntegrator.h"
#include "interfaceMessages.h"
#include <fstream>

#ifndef NO_ISO_14882
using namespace std;
#endif


#include "mbsimServer.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;

enum IPCmethods {
#ifdef HAVE_BOOST_ASIO_HPP
  TCP, UDP,
#endif
  NOMETHOD};

namespace MBSimInterface {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(MBSim::Integrator, InterfaceIntegrator, MBSIMINTNS"InterfaceIntegrator")

  InterfaceIntegrator::InterfaceIntegrator(): MBSim::Integrator(), IP(""), port(""), zSize(0), svSize(0), t(tStart), printCommunication(true), exitRequest(false) {
  }

  void InterfaceIntegrator::initializeUsingXML(TiXmlElement *element) {
    /* TODO
       Integrator::initializeUsingXML(element);
       TiXmlElement *e;
       e=element->FirstChildElement(MBSIMINTNS"IP");
       if(e) setIP(e->ValueStr()); //Element::getString(e));
       e=element->FirstChildElement(MBSIMINTNS"port");
       if(e) setPort(e->ValueStr()); //Element::getString(e));
       */
  }

  TiXmlElement* InterfaceIntegrator::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Integrator::writeXMLFile(parent);
    /* TODO
       addElementText(ele0,MBSIMINTNS"IP",IP);
       addElementText(ele0,MBSIMINTNS"port",port);
       */
    return ele0;
  }

  void InterfaceIntegrator::integrate(MBSim::DynamicSystemSolver& system_) {
    system = &system_;

    zSize=system->getzSize();
    z.resize(zSize, INIT, 0);
    if(getInitialState().size())
      z = getInitialState();
    else
      system->initz(z);
    system->computeInitialCondition();

    svSize=system->getsvSize();
    jsv.resize(svSize, INIT, 0);

    integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);

    try {
      MBSimServer* mbsimServer=NULL;

      IPCmethods ipcmethod=NOMETHOD;
      ipcmethod=TCP;
      switch (ipcmethod)
      {
      case TCP:
        mbsimServer=new MBSimTcpServer(this, 4567);
        break;
      case UDP:
        mbsimServer=new MBSimUdpServer(this, 4567);
        break;
      default:
        cout << "No IPC Methods was defined!" << endl;
      }

      if (mbsimServer!=NULL)
        mbsimServer->start();
    }
    catch (std::exception& e) {
      std::cerr << "Exception: " << e.what() << "\n";
    }

    integPlot.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }

  void InterfaceIntegrator::getz(double** z_) {
    *z_=&z(0);
  }

  void InterfaceIntegrator::getzdot(double** zd_) {
    zd.resize(zSize, NONINIT);
    system->zdot(z, zd, t);
    *zd_=&zd(0);
  }

  void InterfaceIntegrator::getsv(double** sv_) {
    sv.resize(svSize, NONINIT);
    system->getsv(z, sv, t);
    *sv_=&sv(0);

    jsv.resize(svSize, NONINIT);// jsv is necessary for shift 
    for (int i=0; i<svSize; i++)
      jsv(i)=round(sv(i));
  }

  void InterfaceIntegrator::integratorCommunication(const char* requestIdentifier, const char* interface2mbsim, unsigned int interface2mbsimLength, std::ostringstream* mbsim2interface) {
    if (printCommunication)
    {
      ostringstream rIS;
      switch (*requestIdentifier)
      {
#include "interfaceMessages.cc"
        default:
          rIS << "UNKNOWN IPC MESSAGE";
          break;
      }
      integPlot << "interface2mbsim" << endl;
      integPlot << "  requestIdentifier: " << rIS.str() << endl;
      integPlot << "  message: >>>";
      if (interface2mbsimLength>0)
        integPlot << interface2mbsim;
      else
        integPlot << " >>>NO-FURTHER-MESSAGE<<< ";
      integPlot << "<<<" << endl;
    }

    switch (*requestIdentifier) {
      // getSizeOf<differentTypes>
      case _SI_getSizeOfDoubleMemory_asciiString_SI_:
        (*mbsim2interface) << sizeof(double);
        break;
      case _SI_getSizeOfIntegerMemory_asciiString_SI_:
        (*mbsim2interface) << sizeof(int);
        break;
      case _SI_getSizeOfMemoryAdress_asciiString_SI_:
        (*mbsim2interface) << sizeof(char*);
        break;

        // getTime
      case _SI_getTime_asciiString_SI_:
          double2str(mbsim2interface, &t, 1);
        break;
      case _SI_getTime_memoryDump_SI_:
        dumpMemory(mbsim2interface, &t, sizeof(double));
        break;
      case _SI_getTime_memoryAdress_SI_:
        //TODO
        break;
        // setTime
      case _SI_setTime_asciiString_SI_:
        {
          const Vec t=Vec(interface2mbsim);
          if (t.size()!=1)
            throw MBSim::MBSimError("wrong size of given double t!");
          setTime(t(0));
        }
        break;
      case _SI_setTime_memoryDump_SI_:
        {
          const double *d=(double*)(interface2mbsim);
          setTime(*d);
        }
        break;

        // getStateVectorSize
      case _SI_getStateVectorSize_asciiString_SI_:
        (*mbsim2interface) << zSize;
        break;
      case _SI_getStateVectorSize_memoryDump_SI_:
        dumpMemory(mbsim2interface, &zSize, sizeof(int));
        break;

        // getStateVector
      case _SI_getStateVector_asciiString_SI_:
        {
          double* p=NULL;
          getz(&p);
          double2str(mbsim2interface, p, zSize);
        }
        break;
      case _SI_getStateVector_memoryDump_SI_:
        {
          double* p=NULL;
          getz(&p);
          dumpMemory(mbsim2interface, p, zSize*sizeof(double));
        }
        break;
      case _SI_getStateVector_memoryAdress_SI_:
        //TODO
        break;
        // setStateVector
      case _SI_setStateVector_asciiString_SI_:
        {
          const Vec z_(interface2mbsim);
          if (z_.size()!=zSize)
          {
            cerr << "==== wrong size of given vector z! ===" << endl;
            cout << "  interface2mbsim: " << interface2mbsim << endl;
            cout << "  z_: " << z_ << endl;
            cout << "  z_.size(): " << z_.size() << endl;
            cout << "  zSize: " << zSize << endl;
            throw MBSim::MBSimError("wrong size of given vector z!");
          }
          double* p=NULL;
          getz(&p);
          for (int i=0; i<zSize; i++)
          {
            *p=z_(i);
            p++;
          }
        }
        break;
        // getTimeDerivativeOfStateVector
      case _SI_getTimeDerivativeOfStateVector_asciiString_SI_:
        {
          double *p=NULL;
          getzdot(&p);
          double2str(mbsim2interface, p, zSize);
        }
        break;
      case _SI_getTimeDerivativeOfStateVector_memoryDump_SI_:
        {
          double* p=NULL;
          getzdot(&p);
          dumpMemory(mbsim2interface, p, zSize*sizeof(double));
        }
        break;
      case _SI_getTimeDerivativeOfStateVector_memoryAdress_SI_:
        // TODO
        break;

        // getStopVectorSize
      case _SI_getStopVectorSize_asciiString_SI_:
        (*mbsim2interface) << svSize;
        break;
      case _SI_getStopVectorSize_memoryDump_SI_:
        dumpMemory(mbsim2interface, &svSize, sizeof(int));
        break; 
        // getStopVector
      case _SI_getStopVector_asciiString_SI_:
        {
          double *p=NULL;
          getsv(&p);
          double2str(mbsim2interface, p, svSize);
        }
        break;
      case _SI_getStopVector_memoryDump_SI_:
        {
          double *p=NULL;
          getsv(&p);
          dumpMemory(mbsim2interface, p, svSize*sizeof(double));
        }
        break;
      case _SI_getStopVector_memoryAdress_SI_:
        //TODO
        break;

        // different mbsim actions
      case _SI_plot_SI_:
        system->plot(z, t);
        break;
      case _SI_shift_SI_:
        system->shift(z, jsv, t);
        break;
      case _SI_exitRequest_SI_:
        exitRequest=true;
        break;

        // usefull stuff
      case _SI_doPrintCommunication_SI_:
        printCommunication=true;
        break;
      case _SI_donotPrintCommunication_SI_:
        printCommunication=false;
        break;
      case _SI_setAsciiPrecision_asciiString_SI_:
        {
          const Vec p=Vec(interface2mbsim);
          if (p.size()!=1)
            throw MBSim::MBSimError("wrong size of given value!");
          int i=round(p(0));
          if (i<6)
            i=6;
          (*mbsim2interface).precision(i);
        }
        break;
      default:
        cerr << "Unknown IPC message!!!" << endl;
    }

    (*mbsim2interface) << ends; // the sign '\0' is used for signalizing the end of message transfer
    if (printCommunication)
    {
      integPlot << "mbsim2interface:" << endl;
      integPlot << "  length of message: " << (*mbsim2interface).str().length() << " chars." << endl;
      integPlot << "  message: >>>" << (*mbsim2interface).str() << "<<<" << endl;
      integPlot << "\n" << endl;
    }

  }


  void InterfaceIntegrator::dumpMemory(ostringstream *out, void *p, unsigned int N) 
  {
    char *c=(char*)p;
    for (unsigned int i=0; i<N; i++)
    {
      (*out) << *c;
      c++;
    }
  }

  void InterfaceIntegrator::double2str(std::ostringstream *out, double *p, unsigned int N)
  {
    double *d=p;
    (*out) << "[";
    for (unsigned int i=0; i<(N-1); i++)
    {
      (*out) << *d << ";";
      d++;
    }
    (*out) << *d << "]";
  }
}
