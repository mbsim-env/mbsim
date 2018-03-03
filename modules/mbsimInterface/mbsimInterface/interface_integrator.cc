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
#include "interface_integrator.h"
#include "interface_messages.h"
#include "mbsim_server.h"
#include <fstream>
#include "mbsimControl/signal_.h"
#include "mbsimControl/extern_signal_source.h"

#ifndef NO_ISO_14882
using namespace std;
#endif


using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;

enum IPCmethods {
#ifdef HAVE_BOOST_ASIO_HPP
  TCP, UDP,
#endif
  NOMETHOD};

namespace MBSimInterface {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMINTERFACE, InterfaceIntegrator)

    InterfaceIntegrator::InterfaceIntegrator(): MBSimIntegrator::Integrator(),  mbsimServer(nullptr) {
    }

  void InterfaceIntegrator::initializeUsingXML(xercesc::DOMElement *element) {
    Integrator::initializeUsingXML(element);
    xercesc::DOMElement *e, *ee;
    e=E(element)->getFirstElementChildNamed(MBSIMINTERFACE%"outputSignals");
    ee=E(e)->getFirstElementChildNamed(MBSIMINTERFACE%"signal");
    if(ee) {
      while (ee) {
        outputSignalRef.push_back(E(ee)->getAttribute("ref"));
        ee=ee->getNextElementSibling();
      }
    }
    e=E(element)->getFirstElementChildNamed(MBSIMINTERFACE%"inputSignals");
    ee=E(e)->getFirstElementChildNamed(MBSIMINTERFACE%"signal");
    if(ee) {
      while (ee) {
        inputSignalRef.push_back(E(ee)->getAttribute("ref"));
        ee=ee->getNextElementSibling();
      }
    }
    e=E(element)->getFirstElementChildNamed(MBSIMINTERFACE%"methodForIPC");
    ee=e->getFirstElementChild();
    if (E(ee)->getTagName()==MBSIMINTERFACE%"MBSimTcpServer")
      setMBSimServer(new MBSimTcpServer(this));
    else if (E(ee)->getTagName()==MBSIMINTERFACE%"MBSimUdpServer")
      setMBSimServer(new MBSimUdpServer(this));
    mbsimServer->initializeUsingXML(ee);
  }

  void InterfaceIntegrator::integrate() {
    zSize=system->getzSize();
    if(getInitialState().size())
      system->setState(getInitialState());
    else
      system->evalz0();
    system->computeInitialCondition();

    svSize=system->getsvSize();

    integPlot.open((name + ".plt").c_str());
    cout.setf(ios::scientific, ios::floatfield);

    resolveInputOutputNames();

    try {
      mbsimServer->start();
    }
    catch (std::exception& e) {
      std::cerr << e.what()<<endl;
    }

    integPlot.close();

    cout.unsetf (ios::scientific);
    cout << endl;
  }

  void InterfaceIntegrator::getz(double** z_) {
    *z_ = system->getState()();
  }

  void InterfaceIntegrator::getzdot(double** zd_) {
    system->resetUpToDate();
    Vec zd = system->evalzd();
    *zd_= zd();
  }

  void InterfaceIntegrator::getsv(double** sv_) {
    system->resetUpToDate();
    Vec sv = system->evalsv();
    *sv_ = sv();

    for (int i=0; i<svSize; i++)
      system->getjsv()(i)=round(sv(i));
  }

  void InterfaceIntegrator::setTime(double t_) {
    system->setTime(t_);
  }

  void InterfaceIntegrator::setz(const fmatvec::Vec& z_) {
    system->setState(z_);
  }

  void InterfaceIntegrator::resolveInputOutputNames() {

    std::vector<string> toErase;
    toErase.emplace_back("ExternSignalSource");
    toErase.emplace_back("Group");
    toErase.emplace_back("[");
    toErase.emplace_back("]");
    toErase.emplace_back("Signal");

    for (auto & i : outputSignalRef) {
      string lstr=i;
      size_t pos;
      for (auto & j : toErase) {
        size_t len=j.length();
        pos=lstr.find(j);
        while (pos!=string::npos) {
          lstr.erase(pos, len);
          pos=lstr.find(j);
        }
      }
      lstr.erase(0, 1);
      i=lstr;
    }
    for (auto & i : inputSignalRef) {
      string lstr=i;
      size_t pos;
      for (auto & j : toErase) {
        size_t len=j.length();
        pos=lstr.find(j);
        while (pos!=string::npos) {
          lstr.erase(pos, len);
          pos=lstr.find(j);
        }
      }
      lstr.erase(0, 1);
      i=lstr;
    }

    std::vector<MBSim::Link*> mbsLinks=system->getLinks();
    for (const auto & i : outputSignalRef) {
      MBSim::Link *ll=nullptr;
      unsigned int iL=0;
      do {
        if (mbsLinks[iL]->getName() == i)
          ll=mbsLinks[iL];
        iL++;
        if (iL==mbsLinks.size())
          break;
      } while (ll==nullptr);
      if (ll==nullptr)
        cerr << "Could not find Signal >>" << i << "<<" << endl;
      else
        if (dynamic_cast<MBSimControl::Signal*>(ll)) {
          outputSignal.push_back((MBSimControl::Signal*)ll);
          outputSignalName.push_back(outputSignal.back()->getName());
        }
        else
          cerr << "Link >>" << i << "<< is not of MBSimControl::Signal-Type." << endl;
    }
    for (const auto & i : inputSignalRef) {
      MBSim::Link *ll=nullptr;
      unsigned int iL=0;
      do {
        if (mbsLinks[iL]->getName() == i)
          ll=mbsLinks[iL];
        iL++;
        if (iL==mbsLinks.size())
          break;
      } while (ll==nullptr);
      if (ll==nullptr)
        cerr << "Could not find ExternSignalSource >>" << i << "<<" << endl;
      else
        if (dynamic_cast<MBSimControl::ExternSignalSource*>(ll)) {
          inputSignal.push_back((MBSimControl::ExternSignalSource*)ll);
          inputSignalName.push_back(inputSignal.back()->getName());
        }
        else
          cerr << "Link >>" << i << "<< is not of MBSimControl::ExternSignalSource-Type." << endl;
    }

    cout << "interfaceIntegrator: " << endl;
    cout << "  output signals: " << endl;
    int outDim=0, inDim=0;
    outputSignalSize.resize(outputSignal.size(), NONINIT);
    for (unsigned int i=0; i<outputSignal.size(); i++) {
      outputSignalSize(i)=(outputSignal[i]->getSignal()).size();
      outDim+=outputSignalSize(i);
      cout << "    " << i << ": " << outputSignalName[i] << " with size " << outputSignalSize(i) << endl;
    }
    cout << "  -> total size of output: " << outDim << endl;
    cout << "  input signals: " << endl;
    outputVector.resize(outDim, NONINIT);
    inputSignalSize.resize(inputSignal.size(), NONINIT);
    for (unsigned int i=0; i<inputSignal.size(); i++) {
      inputSignalSize(i)=(inputSignal[i]->getSignal()).size();
      inDim+=inputSignalSize(i);
      cout << "    " << i << ": " << inputSignalName[i] << " with size " << inputSignalSize(i) << endl;
    }
    cout << "  -> total size of input: " << inDim << endl;
    inputVector.resize(inDim, NONINIT);
  }

  void InterfaceIntegrator::integratorCommunication(const char* requestIdentifier, const char* interface2mbsim, unsigned int interface2mbsimLength, std::ostringstream* mbsim2interface) {
    if (printCommunication)
    {
      ostringstream rIS;
      switch (*requestIdentifier)
      {
#include "interface_messages.cc"
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
      case _SI_getSizeOfFloatMemory_asciiString_SI_:
        (*mbsim2interface) << sizeof(float);
        break;
      case _SI_getSizeOfIntegerMemory_asciiString_SI_:
        (*mbsim2interface) << sizeof(int);
        break;
      case _SI_getSizeOfMemoryAdress_asciiString_SI_:
        (*mbsim2interface) << sizeof(char*);
        break;

        // getTime
      case _SI_getTime_asciiString_SI_:
        double2str(mbsim2interface, &system->getTime(), 1);
        break;
      case _SI_getTime_memoryDump_SI_:
        dumpMemory(mbsim2interface, &system->getTime(), sizeof(double));
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
          double* p=nullptr;
          getz(&p);
          double2str(mbsim2interface, p, zSize);
        }
        break;
      case _SI_getStateVector_memoryDump_SI_:
        {
          double* p=nullptr;
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
          double* p=nullptr;
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
          double *p=nullptr;
          getzdot(&p);
          double2str(mbsim2interface, p, zSize);
        }
        break;
      case _SI_getTimeDerivativeOfStateVector_memoryDump_SI_:
        {
          double* p=nullptr;
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
          double *p=nullptr;
          getsv(&p);
          double2str(mbsim2interface, p, svSize);
        }
        break;
      case _SI_getStopVector_memoryDump_SI_:
        {
          double *p=nullptr;
          getsv(&p);
          dumpMemory(mbsim2interface, p, svSize*sizeof(double));
        }
        break;
      case _SI_getStopVector_memoryAdress_SI_:
        //TODO
        break;

        // different mbsim actions
      case _SI_plot_SI_:
        system->resetUpToDate();
        system->plot();
        break;
      case _SI_shift_SI_:
        system->resetUpToDate();
        system->plot();
        system->shift();
        break;
      case _SI_exitRequest_SI_:
        exitRequest=true;
        break;

        // outputSignals and inputSignals
      case _SI_getOutputSignalsSize_asciiString_SI_:
        int2str(mbsim2interface, &outputSignalSize(0), outputSignalSize.size());
        break;
      case _SI_getOutputSignals_asciiString_SI_:
        {
          int index0=0, index1=0;
          for (unsigned int i=0; i<outputSignal.size(); i++) {
            index1=index0+outputSignalSize(i);
            outputVector(RangeV(index0, index1))=outputSignal[i]->getSignal();
            index0=index1+1;
          }
          double2str(mbsim2interface, &outputVector(0), outputVector.size());
        }
        break;
      case _SI_getOutputSignals_memoryDump_SI_:
        //TODO
        break;
      case _SI_getOutputSignals_memoryAdress_SI_:
        //TODO
        break;
      case _SI_getInputSignalsSize_asciiString_SI_:
        int2str(mbsim2interface, &inputSignalSize(0), inputSignalSize.size());
        break;
      case _SI_setInputSignals_asciiString_SI_:
        {
          const Vec z_(interface2mbsim);
          if (z_.size()!=inputVector.size())
          {
            cerr << "==== wrong size of given vector z! ===" << endl;
            cout << "  interface2mbsim: " << interface2mbsim << endl;
            cout << "  z_: " << z_ << endl;
            cout << "  z_.size(): " << z_.size() << endl;
            cout << "  zSize: " << zSize << endl;
            throw MBSim::MBSimError("wrong size of given vector z!");
          }
          inputVector=z_;
          int index0=0, index1=0;
          for (unsigned int i=0; i<inputSignal.size(); i++) {
            index1=index0+inputSignalSize(i);
            inputSignal[i]->setSignal(inputVector(RangeV(index0, index1)));
            index0=index1+1;
          }
        }
        break;
      case _SI_setInputSignals_memoryDump_SI_:
        break;
      case _SI_setInputSignals_memoryAdress_SI_:
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


  void InterfaceIntegrator::dumpMemory(ostringstream *out, void *p, unsigned int N) {
    auto *c=(char*)p;
    for (unsigned int i=0; i<N; i++) {
      (*out) << *c;
      c++;
    }
  }

  void InterfaceIntegrator::double2str(std::ostringstream *out, double *p, unsigned int N) {
    double *d=p;
    (*out) << "[";
    for (unsigned int i=0; i<(N-1); i++) {
      (*out) << *d << ";";
      d++;
    }
    (*out) << *d << "]";
  }

  void InterfaceIntegrator::int2str(std::ostringstream *out, int *p, unsigned int N) {
    int *d=p;
    (*out) << "[";
    for (unsigned int i=0; i<(N-1); i++) {
      (*out) << *d << ";";
      d++;
    }
    (*out) << *d << "]";
  }
}
