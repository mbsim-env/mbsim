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

  // A MBSIM FMI instance
  FMIInstance::FMIInstance(fmiString instanceName_, fmiString GUID, fmiCallbackFunctions functions, fmiBoolean loggingOn) :
    instanceName(instanceName_),
    logger(functions.logger),
    infoBuffer (logger, this, instanceName, fmiOK,      "info"),
    warnBuffer (logger, this, instanceName, fmiWarning, "warning"),
    debugBuffer(logger, this, instanceName, fmiOK,      "debug") {

    // use the per FMIInstance provided buffers for all subsequent fmatvec::Atom objects
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Info,  boost::make_shared<ostream>(&infoBuffer));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn,  boost::make_shared<ostream>(&warnBuffer));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Debug, boost::make_shared<ostream>(&debugBuffer));
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

  // destroy a MBSim FMI instance
  FMIInstance::~FMIInstance() {
  }

  // print exceptions using the FMI logger
  void FMIInstance::logException(const exception &ex) {
    logger(this, instanceName.c_str(), fmiError, "error", ex.what());
  }

  // FMI wrapper functions

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
    for(size_t i=0; i<nx; ++i)
      z(i)=x[i];
  }

  // is called when the current state is a valid/accepted integrator step.
  // (e.g. not a intermediate Runge-Kutta step of a step which will be rejected due to
  // the error tolerances of the integrator)
  void FMIInstance::completedIntegratorStep(fmiBoolean* callEventUpdate) {
    *callEventUpdate=false;
    // MISSING: currently we plot on each completed integrator step
    // this should be changed: make it configureable via a parmeter:
    // - plot only at each n-th completet integrator step OR
    // - do not plot here but set nextEventTime to plot at e.g. equidistent time steps OR
    // - ...???
    dss->plot(z, time);
  }

  // set a real variable
  void FMIInstance::setReal(const fmiValueReference vr[], size_t nvr, const fmiReal value[]) {
    // dss exists (after fmiInitialize) -> set the corresponding MBSim input
    if(dss) {
      for(size_t i=0; i<nvr; ++i) {
        if(vr[i]>=vrMap.size())
          throw runtime_error(str(boost::format("No value reference #r%d#.")%vr[i]));
        vrMap[vr[i]]->setRealValue(value[i]);
      }
    }
    // no dss exists (before fmiInitialize) -> just save the value in vrMapStore
    else {
      for(size_t i=0; i<nvr; ++i)
        vrMapStore[vr[i]]=value[i];
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
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Info,  boost::make_shared<ostream>(&infoBuffer));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn,  boost::make_shared<ostream>(&warnBuffer));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Debug, boost::make_shared<ostream>(&debugBuffer));

    // set eventInfo
    eventInfo->iterationConverged=true;
    eventInfo->stateValueReferencesChanged=false;
    eventInfo->stateValuesChanged=false;
    eventInfo->terminateSimulation=false;
    eventInfo->upcomingTimeEvent=false;
    eventInfo->nextEventTime=0;

    // get fmu directory: the .so/.dll is in <fmuDir>/binaries/[linux|win][32|64]
    path fmuDir=getSharedLibDir().parent_path().parent_path();
    // get the model file
    path mbsimflatxmlfile=fmuDir/"resources"/"Model.mbsimprj.flat.xml";

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
    msg(Debug)<<"Create all FMI variables."<<endl;
    createAllVariables(dss.get(), vrMap);

    // initialize dss
    msg(Debug)<<"Initialize DynamicSystemSolver."<<endl;
    dss->initialize();

    // initialize state
    z.resize(dss->getzSize());
    zd.resize(dss->getzSize());
    dss->initz(z);
    dss->computeInitialCondition();

    // between the fmiInstantiateModel and the fmiInitialize calls no dss is available (dss is generated in fmiInitialize).
    // Hence the values of all fmiSetReal, ... calls between these functions are just saved (see setReal(...)).
    // These values are not copied to the now existing dss as start values.
    msg(Debug)<<"Copy already set values to DynamicSystemSolver."<<endl;
    for(size_t vr=0; vr<vrMap.size(); ++vr) {
      map<size_t, double>::iterator vrMapStoreIt=vrMapStore.find(vr);
      if(vrMapStoreIt==vrMapStore.end()) {
        if(vrMap[vr]->getType()==Input) // only inputs are allowed to set (initialized with 0)
          vrMap[vr]->setRealValue(0); // default value is 0, see also mbsimCreateFMU.cc
      }
      else {
        vrMap[vr]->setRealValue(vrMapStoreIt->second); // set to value of last corresponding setReal call
        vrMapStore.erase(vrMapStoreIt); // value is processed now -> remove it
      }
    }

    // the above code should have removed all already set values. Hence if something was not removed its an error.
    if(vrMapStore.size()>0) {
      stringstream str;
      str<<"The following value reference where set/get previously but are not defined:"<<endl;
      for(map<size_t, double>::iterator it=vrMapStore.begin(); it!=vrMapStore.end(); ++it)
        str<<"#r"<<it->first<<"#"<<endl;
      throw runtime_error(str.str());
    }

    // initialize stop vector
    sv.resize(dss->getsvSize());
    svLast.resize(dss->getsvSize());
    jsv.resize(dss->getsvSize(), fmatvec::INIT, 0); // init with 0 = no shift in all indices
    // initialize last stop vector with initial stop vector state (see eventUpdate for details)
    dss->getsv(z, svLast, time);

    // plot initial state
    dss->plot(z, time);
  }

  void FMIInstance::getDerivatives(fmiReal derivatives[], size_t nx) {
    // calcualte MBSim zd and return it
    dss->zdot(z, zd, time);
    for(size_t i=0; i<nx; ++i)
      derivatives[i]=zd(i);
  }

  void FMIInstance::getEventIndicators(fmiReal eventIndicators[], size_t ni) {
    // calcualte MBSim stop vector and return it
    dss->getsv(z, sv, time);
    for(size_t i=0; i<ni; ++i)
      eventIndicators[i]=sv(i);
  }

  void FMIInstance::getReal(const fmiValueReference vr[], size_t nvr, fmiReal value[]) {
    // dss exists (after fmiInitialize) -> set the corresponding MBSim input
    if(dss) {
      for(size_t i=0; i<nvr; ++i) {
        if(vr[i]>=vrMap.size())
          throw runtime_error(str(boost::format("No value reference #r%d#.")%vr[i]));
        value[i]=vrMap[vr[i]]->getRealValue();
      }
    }
    // no dss exists (before fmiInitialize) -> just return the previously set value or 0 (the default value)
    else {
      for(size_t i=0; i<nvr; ++i) {
        if(vr[i]>=vrMapStore.size())
          value[i]=0; // not set till now, return default value 0, see also mbsimCreateFMU.cc
        else
          value[i]=vrMapStore[vr[i]];
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

  // MBSim shift
  // Note: The FMI interface does not provide a jsv integer vector as e.g. the LSODAR integrator does.
  // Since MBSim required this information we have to generate it here. For this we:
  // * store the stop vector (sv) of the initial state (see initialize(...)) or the
  //   last event (shift point) in the variable svLast.
  // * compare the current sv with the sv of the last event (or initial state) svLast
  // * set all entries in jsv to 1 if the corresponding entries in sv and svLast have a sign change.
  void FMIInstance::eventUpdate(fmiBoolean intermediateResults, fmiEventInfo* eventInfo) {
    // initialize eventInfo fields
    eventInfo->iterationConverged=true;
    eventInfo->stateValueReferencesChanged=false;
    eventInfo->stateValuesChanged=false;
    eventInfo->terminateSimulation=false;
    eventInfo->upcomingTimeEvent=false;
    eventInfo->nextEventTime=0;

    // MISSING: event handling must be conform to FMI and modellica!!!???
    // get current stop vector
    dss->getsv(z, sv, time);
    // compare last (svLast) and current (sv) stop vector: build jsv and set shiftRequired
    bool shiftRequired=false;
    for(int i=0; i<sv.size(); ++i) {
      jsv(i)=(svLast(i)*sv(i)<0); // on sign change in svLast and sv set jsv to 1 (shift this index i)
      if(jsv(i))
        shiftRequired=true; // set shiftRequired: a MBSim shift call is required (at least 1 entry in jsv is 1)
                            // (the environment may call eventUpdate of this system due to an event (shift) of another
                            // system (FMU, ...). Hence not all eventUpdate must generate a shift of this system)
    }
    if(shiftRequired) {
      // shift MBSim system
      dss->shift(z, jsv, time);
      // A MBSim shift always changes state values (non-smooth-mechanic)
      // This must be reported to the environment (the integrator must be resetted in this case).
      eventInfo->stateValuesChanged=true;
    }
  }

  void FMIInstance::getContinuousStates(fmiReal states[], size_t nx) {
    for(size_t i=0; i<nx; ++i)
      states[i]=z(i);
  }

  void FMIInstance::getNominalContinuousStates(fmiReal x_nominal[], size_t nx) {
    // we do not proved a nominal value for states in MBSim -> just return 1 as nominal value.
    for(size_t i=0; i<nx; ++i)
      x_nominal[i]=1;
  }

  void FMIInstance::getStateValueReferences(fmiValueReference vrx[], size_t nx) {
    // we do not assign any MBSim state to a FMI valueReference -> return fmiUndefinedValueReference for all states
    for(size_t i=0; i<nx; ++i)
      vrx[i]=fmiUndefinedValueReference;
  }

  void FMIInstance::terminate() {
    // plot end state
    dss->plot(z, time);

    // delete DynamicSystemSolver (requried here since after terminate a call to initialize is allowed without
    // calls to fmiFreeModelInstance and fmiInstantiateModel)
    dss.reset();
  }

}
