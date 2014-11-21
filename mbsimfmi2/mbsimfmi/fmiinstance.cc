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
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost::filesystem;
using namespace MBSim;
using namespace MBSimControl;
using namespace MBXMLUtils;

namespace MBSimFMI {

  FMIVariablePre::FMIVariablePre(Type type_, char datatype_, const std::string &defaultValue) :
    type(type_), datatype(datatype_) {
    if(!defaultValue.empty()) {
      switch(datatype) {
        case 'r': doubleValue =boost::lexical_cast<double>(defaultValue);
        case 'i': integerValue=boost::lexical_cast<int>   (defaultValue);
        case 'b': booleanValue=boost::lexical_cast<bool>  (defaultValue);
        case 's': stringValue =boost::lexical_cast<string>(defaultValue);
      }
    }
  }

  double FMIVariablePre::getValue(double) {
    if(datatype!='r') throw runtime_error("Internal error: Variable datatype differ.");
    return doubleValue;
  }

  int FMIVariablePre::getValue(int) {
    if(datatype!='i') throw runtime_error("Internal error: Variable datatype differ.");
    return integerValue;
  }

  bool FMIVariablePre::getValue(bool) {
    if(datatype!='b') throw runtime_error("Internal error: Variable datatype differ.");
    return booleanValue;
  }

  const char* FMIVariablePre::getValue(const char*) {
    if(datatype!='s') throw runtime_error("Internal error: Variable datatype differ.");
    return stringValue.c_str();
  }

  void FMIVariablePre::setValue(double v) {
    if(datatype!='r') throw runtime_error("Internal error: Variable datatype differ.");
    if(type==Output) throw runtime_error("Setting this variable is not allowed.");
    doubleValue=v;
  }

  void FMIVariablePre::setValue(int v) {
    if(datatype!='i') throw runtime_error("Internal error: Variable datatype differ.");
    if(type==Output) throw runtime_error("Setting this variable is not allowed.");
    integerValue=v;
  }

  void FMIVariablePre::setValue(bool v) {
    if(datatype!='b') throw runtime_error("Internal error: Variable datatype differ.");
    if(type==Output) throw runtime_error("Setting this variable is not allowed.");
    booleanValue=v;
  }

  void FMIVariablePre::setValue(const std::string &v) {
    if(datatype!='s') throw runtime_error("Internal error: Variable datatype differ.");
    if(type==Output) throw runtime_error("Setting this variable is not allowed.");
    stringValue=v;
  }

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

    // load modelDescription XML file
    parser=DOMParser::create(false);
    msg(Debug)<<"Read modelDescription file."<<endl;
    path modelDescriptionXMLFile=getSharedLibDir().parent_path().parent_path()/"modelDescription.xml";
    boost::shared_ptr<xercesc::DOMDocument> doc=parser->parse(modelDescriptionXMLFile);

    // create FMI variables from modelDescription.xml file
    msg(Debug)<<"Generate variables used before fmiInitialize."<<endl;
    size_t vr=0;
    for(xercesc::DOMElement *scalarVar=E(doc->getDocumentElement())->getFirstElementChildNamed("ModelVariables")->getFirstElementChild();
        scalarVar; scalarVar=scalarVar->getNextElementSibling(), ++vr) {
      if(vr!=boost::lexical_cast<size_t>(E(scalarVar)->getAttribute("valueReference")))
        throw runtime_error("Internal error: valueReference missmatch!");
      // get type
      Type type;
           if(E(scalarVar)->getAttribute("causality")=="internal" && E(scalarVar)->getAttribute("variability")=="parameter")  type=Parameter;
      else if(E(scalarVar)->getAttribute("causality")=="input"    && E(scalarVar)->getAttribute("variability")=="continuous") type=Input;
      else if(E(scalarVar)->getAttribute("causality")=="output"   && E(scalarVar)->getAttribute("variability")=="continuous") type=Output;
      else throw runtime_error("Internal error: Unknwon variable type.");
      // get datatype
      char datatype;
           if(E(scalarVar)->getFirstElementChildNamed("Real")) datatype='r';
      else if(E(scalarVar)->getFirstElementChildNamed("Integer")) datatype='i';
      else if(E(scalarVar)->getFirstElementChildNamed("Boolean")) datatype='b';
      else if(E(scalarVar)->getFirstElementChildNamed("String")) datatype='s';
      else throw runtime_error("Internal error: Unknown variable datatype.");
      // get default
      string defaultValue=E(scalarVar->getFirstElementChild())->getAttribute("start");
      // create pre varaible
      vrMapPre.push_back(boost::make_shared<FMIVariablePre>(type, datatype, defaultValue));
    }
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

  namespace {
    template<typename VRMap, typename Type>
    void setValue_vrMap_or_vrMapPre(VRMap &vrMap, const fmiValueReference vr[], size_t nvr, const Type value[]) {
      for(size_t i=0; i<nvr; ++i) {
        if(vr[i]>=vrMap.size())
          throw runtime_error(str(boost::format("No value reference #r%d#.")%vr[i]));
        vrMap[vr[i]]->setValue(value[i]);
      }
    }
  }

  // set a real/integer/boolean/string variable
  template<typename Type>
  void FMIInstance::setValue(const fmiValueReference vr[], size_t nvr, const Type value[]) {
    // dss exists (after fmiInitialize) -> use vrMap
    if(dss)
      setValue_vrMap_or_vrMapPre(vrMap, vr, nvr, value);
    // no dss exists (before fmiInitialize) -> use vrMapPre
    else
      setValue_vrMap_or_vrMapPre(vrMapPre, vr, nvr, value);
  }
  // explicitly instantiate all four FMI types
  template void FMIInstance::setValue(const fmiValueReference vr[], size_t nvr, const fmiReal value[]);
  template void FMIInstance::setValue(const fmiValueReference vr[], size_t nvr, const fmiInteger value[]);
  template void FMIInstance::setValue(const fmiValueReference vr[], size_t nvr, const fmiBoolean value[]);
  template void FMIInstance::setValue(const fmiValueReference vr[], size_t nvr, const fmiString value[]);

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

    // get the model file
    path mbsimflatxmlfile=getSharedLibDir().parent_path().parent_path()/"resources"/"Model.mbsimprj.flat.xml";

    // load all plugins
    msg(Debug)<<"Load MBSim plugins."<<endl;
    MBSimXML::loadPlugins();
  
    // load MBSim project XML document
    msg(Debug)<<"Read MBSim XML model file."<<endl;
    boost::shared_ptr<xercesc::DOMDocument> doc=parser->parse(mbsimflatxmlfile);
  
    // create object for DynamicSystemSolver
    msg(Debug)<<"Create DynamicSystemSolver."<<endl;
    dss.reset(ObjectFactory::createAndInit<DynamicSystemSolver>(doc->getDocumentElement()->getFirstElementChild()));

    // build list of value references
    msg(Debug)<<"Create all FMI variables."<<endl;
    createAllVariables(dss.get(), vrMap, fmiPar);

    // initialize dss
    msg(Debug)<<"Initialize DynamicSystemSolver."<<endl;
    dss->initialize();

    // copy all inputs from vrMapPre to vrMap
    if(vrMapPre.size()!=vrMap.size())
      throw runtime_error("Internal error: The number of parameters differ.");
    vector<boost::shared_ptr<Variable> >::iterator it=vrMap.begin();
    for(vector<boost::shared_ptr<FMIVariablePre> >::iterator itPre=vrMapPre.begin(); itPre!=vrMapPre.end(); ++itPre, ++it) {
      if((*itPre)->getType()!=Input)
        continue;
      if((*it)->getType()!=Input)
        throw runtime_error("Internal error: Variable type does not match.");
      (*it)->setValue((*itPre)->getValue(double(0)));
    }

    // initialize state
    z.resize(dss->getzSize());
    zd.resize(dss->getzSize());
    dss->initz(z);
    dss->computeInitialCondition();

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

  namespace {
    template<typename VRMap, typename Type>
    void getValue_vrMap_or_vrMapPre(VRMap &vrMap, const fmiValueReference vr[], size_t nvr, Type value[]) {
      for(size_t i=0; i<nvr; ++i) {
        if(vr[i]>=vrMap.size())
          throw runtime_error(str(boost::format("No value reference #r%d#.")%vr[i]));
        value[i]=vrMap[vr[i]]->getValue(static_cast<Type>(0));
      }
    }
  }

  // get a real/integer/boolean/string variable
  template<typename Type>
  void FMIInstance::getValue(const fmiValueReference vr[], size_t nvr, Type value[]) {
    // dss exists (after fmiInitialize) -> set the corresponding MBSim input
    if(dss)
      getValue_vrMap_or_vrMapPre(vrMap, vr, nvr, value);
    // no dss exists (before fmiInitialize) -> just return the previously set value or 0 (the default value)
    else
      getValue_vrMap_or_vrMapPre(vrMapPre, vr, nvr, value);
  }
  // explicitly instantiate all four FMI types
  template void FMIInstance::getValue(const fmiValueReference vr[], size_t nvr, fmiReal value[]);
  template void FMIInstance::getValue(const fmiValueReference vr[], size_t nvr, fmiInteger value[]);
  template void FMIInstance::getValue(const fmiValueReference vr[], size_t nvr, fmiBoolean value[]);
  template void FMIInstance::getValue(const fmiValueReference vr[], size_t nvr, fmiString value[]);

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
