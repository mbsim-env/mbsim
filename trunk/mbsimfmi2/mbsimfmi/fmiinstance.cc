#include "../config.h"
#include <fmiinstance.h>
#include <stdexcept>
#include <mbsimxml/mbsimflatxml.h>
#include <mbxmlutilshelper/dom.h>
#include <xercesc/dom/DOMDocument.hpp>
#include <mbsim/objectfactory.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsimControl/extern_signal_source.h>
#include <mbsimControl/extern_signal_sink.h>
#include <mbsim/extern_generalized_io.h>
#include <boost/format.hpp>

using namespace std;
using namespace boost::filesystem;
using namespace MBSim;
using namespace MBSimControl;
using namespace MBXMLUtils;

namespace MBSimFMI {

  FMIInstance::FMIInstance(fmiString instanceName_, fmiString GUID, fmiCallbackFunctions functions, fmiBoolean loggingOn) :
    instanceName(instanceName_),
    logger(functions.logger),
    infoBuffer(logger, this, instanceName, "info"),
    warnBuffer(logger, this, instanceName, "warning"),
    debugBuffer(logger, this, instanceName, "debug") {

    // use the per FMIInstance provided buffers for all subsequent fmatvec::Atom objects
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Info, boost::make_shared<ostream>(&infoBuffer));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn, boost::make_shared<ostream>(&warnBuffer));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn, boost::make_shared<ostream>(&debugBuffer));
    // also use these streams for this object.
    // Note: we can not create a FMIInstance object with the correct streams but we can adopt the streams now!
    adoptMessageStreams(); // note: no arg means adopt the current (static) message streams (set above)
    // Now we can use msg(...)<< to print messages using the FMI logger

    // set debug stream according loggingOn
    setMessageStreamActive(Debug, loggingOn);
    msg(Debug)<<"Enabling debug logging."<<endl;

    // check GUID: we use currently a constant GUID
    if(string(GUID)!="mbsimfmi_guid")
      throw runtime_error("GUID provided by caller and internal GUID does not match.");
  }

  FMIInstance::~FMIInstance() {
  }

  void FMIInstance::logException(const exception &ex) {
    logger(this, instanceName.c_str(), fmiOK, "error", ex.what());
  }

  void FMIInstance::setDebugLogging(fmiBoolean loggingOn) {
    if(!loggingOn)
      msg(Debug)<<"Disabling debug logging."<<endl;
    // set debug stream according loggingOn
    setMessageStreamActive(Debug, loggingOn);
    if(loggingOn)
      msg(Debug)<<"Enabling debug logging."<<endl;
  }

  void FMIInstance::setTime(fmiReal time_) {
    time=time_;
  }

  void FMIInstance::setContinuousStates(const fmiReal x[], size_t nx) {
    //MFMF
  }

  void FMIInstance::completedIntegratorStep(fmiBoolean* callEventUpdate) {
    *callEventUpdate=false;
  }

  void FMIInstance::setReal(const fmiValueReference vr[], size_t nvr, const fmiReal value[]) {
    if(dss) {
      for(size_t i=0; i<nvr; ++i) {
        if(vr[i]>=vrUnion.size())
          throw runtime_error(str(boost::format("No value reference #r%d#.")%vr[i]));
        switch(vrUnion[vr[i]].first) {
          case ValueReferenceMap::GeneralizedIO_h:
            vrUnion[vr[i]].second.generalizedIO->setGeneralizedForce(value[i]);
            break;
          case ValueReferenceMap::SignalSource:
            vrUnion[vr[i]].second.signalSource->setSignal(value[i]);
            break;
          default:
            throw runtime_error(str(boost::format("Setting #r%d# not allowed.")%vr[i]));
        }
      }
    }
    else {
      for(size_t i=0; i<nvr; ++i)
        vrReal[vr[i]]=value[i];
    }
  }

  void FMIInstance::setInteger(const fmiValueReference vr[], size_t nvr, const fmiInteger value[]) {
    throw runtime_error("No values of type integer.");
  }

  void FMIInstance::setBoolean(const fmiValueReference vr[], size_t nvr, const fmiBoolean value[]) {
    throw runtime_error("No values of type boolean.");
  }

  void FMIInstance::setString(const fmiValueReference vr[], size_t nvr, const fmiString value[]) {
    throw runtime_error("No values of type string.");
  }

  void FMIInstance::initialize(fmiBoolean toleranceControlled, fmiReal relativeTolerance, fmiEventInfo* eventInfo) {
    // after the ctor call another FMIInstance ctor may be called, hence we need to reset the message streams here
    // use the per FMIInstance provided buffers for all subsequent fmatvec::Atom objects
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Info, boost::make_shared<ostream>(&infoBuffer));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn, boost::make_shared<ostream>(&warnBuffer));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn, boost::make_shared<ostream>(&debugBuffer));

    // set eventInfo
    eventInfo->iterationConverged=true;
    eventInfo->stateValueReferencesChanged=false;
    eventInfo->stateValuesChanged=false;
    eventInfo->terminateSimulation=false;
    eventInfo->upcomingTimeEvent=false;
    eventInfo->nextEventTime=0;

    // get fmu directory: the .so/.dll is in <fmuDir>/binaries/[linux|win][32|64]
    path fmuDir=getSharedLibDir().parent_path().parent_path();
    // search the mbsimprj xml file in resources
    path mbsimflatxmlfile;
    for(directory_iterator it(fmuDir/"resources"); it!=directory_iterator(); ++it) {
      string name=it->path().filename().string();
      if(name.substr(name.length()-string(".mbsimprj.flat.xml").length())==".mbsimprj.flat.xml") {
        mbsimflatxmlfile=it->path();
        break;
      }
      if(name.substr(name.length()-string(".mbsimprj.xml").length())==".mbsimprj.xml" && name.substr(0, 4)==".pp.") {
        mbsimflatxmlfile=it->path();
        break;
      }
    }

    // load all plugins
    msg(Debug)<<"Load MBSim plugins."<<endl;
    MBSimXML::loadPlugins();
  
    // load MBSim project XML document
    msg(Debug)<<"Read MBSim XML model file."<<endl;
    boost::shared_ptr<DOMParser> parser=DOMParser::create(false);
    boost::shared_ptr<xercesc::DOMDocument> doc=parser->parse(mbsimflatxmlfile);
  
    // create object for DynamicSystemSolver
    msg(Debug)<<"Create DynamicSystemSolver."<<endl;
    dss.reset(ObjectFactory::createAndInit<DynamicSystemSolver>(doc->getDocumentElement()->getFirstElementChild()));

    // build list of value references
    msg(Debug)<<"Build valueReference list."<<endl;
    ValueReferenceMap::create(dss.get(), vrUnion);

    // initialize dss
    msg(Debug)<<"Initialize DynamicSystemSolver."<<endl;
    dss->initialize();

    // copy all set values (between fmiInstantiateModel and fmiInitialize (this func)) to the dss
    msg(Debug)<<"Copy values to DynamicSystemSolver."<<endl;
    size_t vr=0;
    for(ValueReferenceMap::VRMap::iterator vrIt=vrUnion.begin(); vrIt!=vrUnion.end(); ++vrIt, ++vr) {
      switch(vrIt->first) {
        case ValueReferenceMap::GeneralizedIO_h: {
          map<size_t, double>::iterator vrRealIt=vrReal.find(vr);
          if(vrRealIt==vrReal.end())
            vrIt->second.generalizedIO->setGeneralizedForce(0); // default value is 0, see also mbsimCreateFMU.cc
          else
            vrIt->second.generalizedIO->setGeneralizedForce(vrRealIt->second);
          break;
        }
        case ValueReferenceMap::GeneralizedIO_x: 
        case ValueReferenceMap::GeneralizedIO_v:
          break;
        case ValueReferenceMap::SignalSource: {
          map<size_t, double>::iterator vrRealIt=vrReal.find(vr);
          if(vrRealIt==vrReal.end())
            vrIt->second.signalSource->setSignal(0); // default value is 0, see also mbsimCreateFMU.cc
          else
            vrIt->second.signalSource->setSignal(vrRealIt->second);
          break;
        }
        case ValueReferenceMap::SignalSink: 
          break;
      }
    }

    // check illegal wrong previous calls
    if(vrReal.size()>0) {
      stringstream str;
      str<<"The following value reference where set/get previously but are not defined:"<<endl;
      for(map<size_t, double>::iterator it=vrReal.begin(); it!=vrReal.end(); ++it)
        str<<"#r"<<it->first<<"#"<<endl;
      throw runtime_error(str.str());
    }
  }

  void FMIInstance::getDerivatives(fmiReal derivatives[], size_t nx) {
    //MFMF
  }

  void FMIInstance::getEventIndicators(fmiReal eventIndicators[], size_t ni) {
    //MFMF
  }

  void FMIInstance::getReal(const fmiValueReference vr[], size_t nvr, fmiReal value[]) {
    if(dss) {
      for(size_t i=0; i<nvr; ++i) {
        if(vr[i]>=vrUnion.size())
          throw runtime_error(str(boost::format("No value reference #r%d#.")%vr[i]));
        switch(vrUnion[vr[i]].first) {
          case ValueReferenceMap::GeneralizedIO_h:
            value[i]=vrUnion[vr[i]].second.generalizedIO->getla()(0);
            break;
          case ValueReferenceMap::GeneralizedIO_x:
            value[i]=vrUnion[vr[i]].second.generalizedIO->getGeneralizedPosition();
            break;
          case ValueReferenceMap::GeneralizedIO_v:
            value[i]=vrUnion[vr[i]].second.generalizedIO->getGeneralizedVelocity();
            break;
          case ValueReferenceMap::SignalSource:
            value[i]=vrUnion[vr[i]].second.signalSource->getSignal()(0);
            break;
          case ValueReferenceMap::SignalSink:
            value[i]=vrUnion[vr[i]].second.signalSink->getSignal()(0);
            break;
        }
      }
    }
    else {
      for(size_t i=0; i<nvr; ++i) {
        if(vr[i]>=vrReal.size())
          value[i]=0; // not set till now, return default value 0, see also mbsimCreateFMU.cc
        else
          value[i]=vrReal[vr[i]];
      }
    }
  }

  void FMIInstance::getInteger(const fmiValueReference vr[], size_t nvr, fmiInteger value[]) {
    throw runtime_error("No values of type integer.");
  }

  void FMIInstance::getBoolean(const fmiValueReference vr[], size_t nvr, fmiBoolean value[]) {
    throw runtime_error("No values of type boolean.");
  }

  void FMIInstance::getString(const fmiValueReference vr[], size_t nvr, fmiString value[]) {
    throw runtime_error("No values of type string.");
  }

  void FMIInstance::eventUpdate(fmiBoolean intermediateResults, fmiEventInfo* eventInfo) {
    //MFMF
  }

  void FMIInstance::getContinuousStates(fmiReal states[], size_t nx) {
    //MFMF
  }

  void FMIInstance::getNominalContinuousStates(fmiReal x_nominal[], size_t nx) {
    for(size_t i=0; i<nx; ++i)
      x_nominal[i]=1;
  }

  void FMIInstance::getStateValueReferences(fmiValueReference vrx[], size_t nx) {
    for(size_t i=0; i<nx; ++i)
      vrx[i]=fmiUndefinedValueReference;
  }

  void FMIInstance::terminate() {
    dss.reset();
  }

}
