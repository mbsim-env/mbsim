#include "../config.h"
#include <fmiinstance.h>
#include <stdexcept>
#include <mbxmlutilshelper/dom.h>
#include <xercesc/dom/DOMDocument.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/scope_exit.hpp>
#include <mbsim/dynamic_system_solver.h>

// rethrow a catched exception after prefixing the what() string with the FMI variable name
#define RETHROW_VR(vr) \
  catch(const std::exception &ex) { \
    rethrowVR(vr, ex); \
  } \
  catch(...) { \
    rethrowVR(vr); \
  }

using namespace std;
using namespace boost;
using namespace boost::filesystem;
using namespace MBSim;
using namespace MBSimControl;
using namespace MBXMLUtils;

namespace {

  template<class Datatype>
  void addPreInitVariable(const xercesc::DOMElement *scalarVar, vector<shared_ptr<MBSimFMI::Variable> > &var) {
    // get type
    MBSimFMI::Type type;
    if     (E(scalarVar)->getAttribute("causality")=="internal" && E(scalarVar)->getAttribute("variability")=="parameter")
      type=MBSimFMI::Parameter;
    else if(E(scalarVar)->getAttribute("causality")=="input"    && E(scalarVar)->getAttribute("variability")=="continuous")
      type=MBSimFMI::Input;
    else if(E(scalarVar)->getAttribute("causality")=="output"   && E(scalarVar)->getAttribute("variability")=="continuous")
      type=MBSimFMI::Output;
    else
      throw runtime_error("Internal error: Unknwon variable type.");
    // get default
    Datatype defaultValue;
    if(E(scalarVar->getFirstElementChild())->hasAttribute("start"))
      defaultValue=lexical_cast<Datatype>(E(scalarVar->getFirstElementChild())->getAttribute("start"));
    // create preprocessing variable
    var.push_back(make_shared<MBSimFMI::VariableStore<Datatype> >(E(scalarVar)->getAttribute("name"), type, defaultValue));
  }

}

namespace MBSimFMI {

  shared_ptr<FMIInstanceBase> fmiInstanceCreate(fmiString instanceName_, fmiString GUID,
                                                       fmiCallbackFunctions functions, fmiBoolean loggingOn) {
    return make_shared<FMIInstance>(instanceName_, GUID, functions, loggingOn);
  }

  // A MBSim FMI instance. Called by fmiInstantiateModel
  FMIInstance::FMIInstance(fmiString instanceName_, fmiString GUID, fmiCallbackFunctions functions, fmiBoolean loggingOn) :
    updateDerivativesRequired(true),
    updateEventIndicatorsRequired(true),
    updateValueRequired(true),
    instanceName(instanceName_),
    logger(functions.logger),
    infoBuffer (logger, this, instanceName, fmiOK,      "info"),
    warnBuffer (logger, this, instanceName, fmiWarning, "warning"),
    debugBuffer(logger, this, instanceName, fmiOK,      "debug") {

    // use the per FMIInstance provided buffers for all subsequent fmatvec::Atom objects
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Info,  make_shared<bool>(true),  make_shared<ostream>(&infoBuffer));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn,  make_shared<bool>(true),  make_shared<ostream>(&warnBuffer));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Debug, make_shared<bool>(false), make_shared<ostream>(&debugBuffer));
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
    path modelDescriptionXMLFile=path(MBXMLUtils::getFMUSharedLibPath()).parent_path().parent_path().parent_path().parent_path()/
      "modelDescription.xml";
    shared_ptr<xercesc::DOMDocument> doc=parser->parse(modelDescriptionXMLFile);

    // add all predefined parameters
    addPredefinedParameters(var, predefinedParameterStruct, true);
    size_t numPredefParam=var.size();
    // create FMI variables from modelDescription.xml file
    msg(Debug)<<"Generate call variables as VariableStore objects. Used until fmiInitialize is called."<<endl;
    size_t vr=0;
    for(xercesc::DOMElement *scalarVar=E(doc->getDocumentElement())->getFirstElementChildNamed("ModelVariables")->getFirstElementChild();
        scalarVar; scalarVar=scalarVar->getNextElementSibling(), ++vr) {
      // skip all predefined parameters which are already added by addPredefinedParameters above
      if(vr<numPredefParam)
        continue;

      // now add all other parameters
      msg(Debug)<<"Generate variable '"<<E(scalarVar)->getAttribute("name")<<"'"<<endl;
      if(vr!=lexical_cast<size_t>(E(scalarVar)->getAttribute("valueReference")))
        throw runtime_error("Internal error: valueReference missmatch!");
      // add variable
      if(E(scalarVar)->getFirstElementChildNamed("Real"))
        addPreInitVariable<double>(scalarVar, var);
      else if(E(scalarVar)->getFirstElementChildNamed("Integer") ||
              E(scalarVar)->getFirstElementChildNamed("Enumeration"))
        addPreInitVariable<int>(scalarVar, var);
      else if(E(scalarVar)->getFirstElementChildNamed("Boolean"))
        addPreInitVariable<bool>(scalarVar, var);
      else if(E(scalarVar)->getFirstElementChildNamed("String"))
        addPreInitVariable<string>(scalarVar, var);
      else
        throw runtime_error("Internal error: Unknown variable datatype.");
    }
  }

  // destroy a MBSim FMI instance. Called by fmiFreeModelInstance
  FMIInstance::~FMIInstance() {
  }

  // FMI wrapper functions, except fmiInstantiateModel and fmiFreeModelInstance see above

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
    // everything may depend on time -> update required on next getXXX
    updateDerivativesRequired=true;
    updateEventIndicatorsRequired=true;
    updateValueRequired=true;
  }

  void FMIInstance::setContinuousStates(const fmiReal x[], size_t nx) {
    for(size_t i=0; i<nx; ++i)
      z(i)=x[i];
    // everything may depend on the states -> update required on next getXXX
    updateDerivativesRequired=true;
    updateEventIndicatorsRequired=true;
    updateValueRequired=true;
  }

  // is called when the current state is a valid/accepted integrator step.
  // (e.g. not a intermediate Runge-Kutta step or a step which will be rejected due to
  // the error tolerances of the integrator)
  void FMIInstance::completedIntegratorStep(fmiBoolean* callEventUpdate) {
    *callEventUpdate=false;

    // plot the current system state dependent on plotMode
    switch(predefinedParameterStruct.plotMode) {
      // plot at each n-th completed step
      case EverynthCompletedStep:
        completedStepCounter++;
        if(completedStepCounter==predefinedParameterStruct.plotEachNStep) {
          completedStepCounter=0;
          dss->setTime(time);
          dss->setState(z);
          dss->resetUpToDate();
          dss->solveAndPlot();
        }
        break;
      // plot if this is the first completed step after nextPlotTime
      case NextCompletedStepAfterSampleTime:
        if(time>=nextPlotTime) {
          nextPlotTime += predefinedParameterStruct.plotStepSize * (floor((time-nextPlotTime)/predefinedParameterStruct.plotStepSize)+1);
          dss->setTime(time);
          dss->setState(z);
          dss->resetUpToDate();
          dss->solveAndPlot();
        }
        break;
      // do not plot at completed steps -> plot at discrete sample times
      case SampleTime:
        break;
    }
  }

  // set a real/integer/boolean/string variable
  template<typename CppDatatype, typename FMIDatatype>
  void FMIInstance::setValue(const fmiValueReference vr[], size_t nvr, const FMIDatatype value[]) {
    for(size_t i=0; i<nvr; ++i) {
      if(vr[i]>=var.size())
        throw runtime_error("No such value reference "+lexical_cast<string>(vr[i]));
      try { var[vr[i]]->setValue(CppDatatype(value[i])); } RETHROW_VR(vr[i])
    }
    // everything may depend on inputs -> update required on next getXXX
    updateDerivativesRequired=true;
    updateEventIndicatorsRequired=true;
    updateValueRequired=true;
  }
  // explicitly instantiate all four FMI types
  template void FMIInstance::setValue<double, fmiReal   >(const fmiValueReference vr[], size_t nvr, const fmiReal    value[]);
  template void FMIInstance::setValue<int,    fmiInteger>(const fmiValueReference vr[], size_t nvr, const fmiInteger value[]);
  template void FMIInstance::setValue<bool,   fmiBoolean>(const fmiValueReference vr[], size_t nvr, const fmiBoolean value[]);
  template void FMIInstance::setValue<string, fmiString >(const fmiValueReference vr[], size_t nvr, const fmiString  value[]);

  void FMIInstance::initialize(fmiBoolean toleranceControlled, fmiReal relativeTolerance, fmiEventInfo* eventInfo) {
    // after the ctor call another FMIInstance ctor may be called, hence we need to reset the message streams here
    // use the per FMIInstance provided buffers for all subsequent fmatvec::Atom objects
    shared_ptr<bool> a;
    shared_ptr<ostream> s;
    getMessageStream(fmatvec::Atom::Info,  a, s); fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Info,  a, s);
    getMessageStream(fmatvec::Atom::Warn,  a, s); fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn,  a, s);
    getMessageStream(fmatvec::Atom::Debug, a, s); fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Debug, a, s);

    // set eventInfo (except next event time)
    eventInfo->iterationConverged=true;
    eventInfo->stateValueReferencesChanged=false;
    eventInfo->stateValuesChanged=false;
    eventInfo->terminateSimulation=false;

    // predefined variables used during simulation
    vector<shared_ptr<Variable> > varSim;
    msg(Debug)<<"Create predefined parameters."<<endl;
    addPredefinedParameters(varSim, predefinedParameterStruct, false);
    
    // create output directory
    create_directories(predefinedParameterStruct.outputDir);

    // add model parmeters to varSim and create the DynamicSystemSolver (set the dss varaible)
    addModelParametersAndCreateDSS(varSim);

    // save the current dir and change to outputDir -> MBSim will create output files in the current dir
    // this must be done before the dss is initialized since dss->initialize creates files in the current dir)
    msg(Debug)<<"Write MBSim output files to "<<predefinedParameterStruct.outputDir<<endl;
    path savedCurDir=current_path();
    // restore current dir on scope exit
    BOOST_SCOPE_EXIT((&savedCurDir)) { current_path(savedCurDir); } BOOST_SCOPE_EXIT_END
    current_path(predefinedParameterStruct.outputDir);

    // initialize dss (must be done before addModelInputOutputs because references in MBSim may be resolved for this;
    // must be done after the current dir is set (temporarily) to the output dir)
    msg(Debug)<<"Initialize DynamicSystemSolver."<<endl;
    dss->initialize();

    // create model IO vars (before this call the dss must be initialized)
    msg(Debug)<<"Create model input/output variables."<<endl;
    addModelInputOutputs(varSim, dss.get());

    // Till now (between fmiInstantiateModel and fmiInitialize) we have only used objects of type VariableStore's in var.
    // Now we copy all values from var to varSim (varSim is generated above).
    if(var.size()!=varSim.size())
      throw runtime_error("The number of parameters from modelDescription.xml and model differ: "
                          +lexical_cast<string>(var.size())+", "+lexical_cast<string>(varSim.size())+". "+
                          "Maybe the model topologie has changed due to a parameter change but this is not allowed.");
    vector<shared_ptr<Variable> >::iterator varSimIt=varSim.begin();
    size_t vr=0;
    for(vector<shared_ptr<Variable> >::iterator varIt=var.begin(); varIt!=var.end(); ++varIt, ++varSimIt, ++vr) {
      try {
        // check for a change of the model topologie
        if((*varSimIt)->getName()!=(*varIt)->getName())
          throw runtime_error("Variable names from modelDescription.xml and model does not match: "
                              +(*varIt)->getName()+", "+(*varSimIt)->getName()+". "+
                              "Maybe the model topologie has changed due to a parameter change but this is not allowed.");
        if((*varSimIt)->getType()!=(*varIt)->getType())
          throw runtime_error("Variable type (parameter, input, output) from modelDescription.xml and model does not match: "
                              +lexical_cast<string>((*varIt)->getType())+", "
                              +lexical_cast<string>((*varSimIt)->getType())+". "+
                              "Maybe the model topologie has changed due to a parameter change but this is not allowed.");
        if((*varSimIt)->getDatatypeChar()!=(*varIt)->getDatatypeChar())
          throw runtime_error(string("Variable datatype from modelDescription.xml and model does not match: ")
                              +(*varIt)->getDatatypeChar()+", "+(*varSimIt)->getDatatypeChar()+". "+
                              "Maybe the model topologie has changed due to a parameter change but this is not allowed.");
        // copy variable values
        if((*varIt)->getType()==Output) // outputs are not allowed to be set -> skip
          continue;
        msg(Debug)<<"Copy variable '"<<(*varSimIt)->getName()<<"' from VariableStore object to the \"real\" varaible object."<<endl;
        switch((*varIt)->getDatatypeChar()) {
          case 'r': (*varSimIt)->setValue((*varIt)->getValue(double())); break;
          case 'i': (*varSimIt)->setValue((*varIt)->getValue(int())); break;
          case 'b': (*varSimIt)->setValue((*varIt)->getValue(bool())); break;
          case 's': (*varSimIt)->setValue((*varIt)->getValue(string())); break;
        }
      }
      RETHROW_VR(vr)
    }
    // var is now no longer needed since we use varSim now.
    var=varSim;

    // initialize state
    msg(Debug)<<"Initialize initial conditions of the DynamicSystemSolver."<<endl;
    z.resize(dss->getzSize());
    zd.resize(dss->getzSize());
    z = dss->evalz0();
    dss->computeInitialCondition();

    // initialize stop vector
    sv.resize(dss->getsvSize());
    svLast.resize(dss->getsvSize());
    jsv.resize(dss->getsvSize(), fmatvec::INIT, 0); // init with 0 = no shift in all indices
    // initialize last stop vector with initial stop vector state
    dss->setTime(time);
    dss->setState(z);
    dss->resetUpToDate();
    svLast = dss->evalsv();

    // plot initial state
    dss->solveAndPlot();

    // handling of plot mode
    switch(predefinedParameterStruct.plotMode) {
      case EverynthCompletedStep:
        // init
        completedStepCounter=0;
        // no next time event
        eventInfo->upcomingTimeEvent=false;
        break;
      case NextCompletedStepAfterSampleTime:
        // init
        nextPlotTime=time+predefinedParameterStruct.plotStepSize;
        // next time event
        eventInfo->upcomingTimeEvent=false;
        break;
      case SampleTime:
        // init
        nextPlotTime=time+predefinedParameterStruct.plotStepSize;
        // next time event
        eventInfo->upcomingTimeEvent=true;
        eventInfo->nextEventTime=nextPlotTime;
        break;
    }

    // everything may depend on inputs -> update required on next getXXX
    updateDerivativesRequired=true;
    updateEventIndicatorsRequired=true;
    updateValueRequired=true;
  }

  void FMIInstance::getDerivatives(fmiReal derivatives[], size_t nx) {
      // calcualte MBSim zd and return it
    if(updateDerivativesRequired) {
      dss->setTime(time);
      dss->setState(z);
      dss->resetUpToDate();
      zd = dss->evalzd();
      updateDerivativesRequired=false;
    }
    for(size_t i=0; i<nx; ++i)
      derivatives[i]=zd(i);
  }

  void FMIInstance::getEventIndicators(fmiReal eventIndicators[], size_t ni) {
    // calcualte MBSim stop vector and return it
    if(updateEventIndicatorsRequired) {
      dss->setTime(time);
      dss->setState(z);
      dss->resetUpToDate();
      sv = dss->evalsv();
      updateEventIndicatorsRequired=false;
    }
    for(size_t i=0; i<ni; ++i)
      eventIndicators[i]=sv(i);
  }

  namespace {
    // convert a CppDatatype to FMIDatatype: default implementaion: just use implicit conversion.
    template<class RetType, class ArgType>
    RetType cppDatatypeToFMIDatatype(const ArgType &a) { return a; }
    // convert a CppDatatype to FMIDatatype: specialization for string: return the c string part of the reference string.
    template<>
    fmiString cppDatatypeToFMIDatatype<fmiString, string>(const string &a) { return a.c_str(); }
  }

  // get a real/integer/boolean/string variable
  template<typename CppDatatype, typename FMIDatatype>
  void FMIInstance::getValue(const fmiValueReference vr[], size_t nvr, FMIDatatype value[]) {
    if(updateValueRequired) {
      // TODO: nothing to do currently since MBSimControl::Singal's is updated on demand but this will change
      // (current branch of Foerg; MBSim issue 39)
      updateValueRequired=false;
    }
    for(size_t i=0; i<nvr; ++i) {
      if(vr[i]>=var.size())
        throw runtime_error("No such value reference "+lexical_cast<string>(vr[i]));
      try { value[i]=cppDatatypeToFMIDatatype<FMIDatatype, CppDatatype>(var[vr[i]]->getValue(CppDatatype())); } RETHROW_VR(vr[i])
    }
  }
  // explicitly instantiate all four FMI types
  template void FMIInstance::getValue<double, fmiReal   >(const fmiValueReference vr[], size_t nvr, fmiReal value[]);
  template void FMIInstance::getValue<int,    fmiInteger>(const fmiValueReference vr[], size_t nvr, fmiInteger value[]);
  template void FMIInstance::getValue<bool,   fmiBoolean>(const fmiValueReference vr[], size_t nvr, fmiBoolean value[]);
  template void FMIInstance::getValue<string, fmiString> (const fmiValueReference vr[], size_t nvr, fmiString value[]);

  // shift point
  void FMIInstance::eventUpdate(fmiBoolean intermediateResults, fmiEventInfo* eventInfo) {
    // initialize eventInfo fields (except next time event)
    eventInfo->iterationConverged=true;
    eventInfo->stateValueReferencesChanged=false;
    eventInfo->stateValuesChanged=false;
    eventInfo->terminateSimulation=false;
    eventInfo->upcomingTimeEvent=false;

    // ***** state event = root = stop vector event *****

    // Note: The FMI interface does not provide a jsv integer vector as e.g. the LSODAR integrator does.
    // Since MBSim requires this information we have to generate it here. For this we:
    // * store the stop vector after the last event (shift point) in the variable svLast
    // * compare the current sv with svLast using the FMI state event condition (see below)
    // * set all entries in jsv to 1 if the this condition matches

    // get current stop vector
    if(updateEventIndicatorsRequired) {
      dss->setTime(time);
      dss->setState(z);
      dss->resetUpToDate();
      sv = dss->evalsv();
      updateEventIndicatorsRequired=false;
    }
    // compare last (svLast) and current (sv) stop vector, based on the FMI standard
    // shift equation: (sv(i)>0) != (svLast(i)>0): build jsv and set shiftRequired
    bool shiftRequired=false;
    for(int i=0; i<sv.size(); ++i)
      if((svLast(i)>0) != (sv(i)>0)) { // use 0 to check for ==0
        jsv(i)=1;
        shiftRequired=true; // set shiftRequired: a MBSim shift call is required (at least 1 entry in jsv is 1)
      }
      else
        jsv(i)=0;
    if(shiftRequired) {
      // shift MBSim system
      dss->setTime(time);
      dss->setState(z);
      dss->setjsv(jsv);
      dss->resetUpToDate();
      z = dss->shift();
      // A MBSim shift always changes state values (at least by a minimal projection)
      // This must be reported to the environment (the integrator must be resetted in this case).
      eventInfo->stateValuesChanged=true;

      // everything may depend on a changed (shifted) system -> update required on next getXXX
      updateDerivativesRequired=true;
      updateValueRequired=true;
      // get current stop vector with is now also the last stop vector
      dss->setState(z);
      dss->resetUpToDate();
      sv = dss->evalsv();
      svLast=sv;
      updateEventIndicatorsRequired=false; // we have the stop vector always updated
    }

    // ***** time event (currently only for plotting) *****

    switch(predefinedParameterStruct.plotMode) {
      case EverynthCompletedStep:
      case NextCompletedStepAfterSampleTime:
        // no next time event
        break;
      case SampleTime:
        // next time event
        eventInfo->upcomingTimeEvent=true;

        // next event wenn plotting with sample time and we currently match that time
        if(fabs(time-nextPlotTime)<1.0e-10) {
          // plot
          dss->setTime(time);
          dss->setState(z);
          dss->resetUpToDate();
          dss->solveAndPlot();
          // next time event
          nextPlotTime=time+predefinedParameterStruct.plotStepSize;
          eventInfo->nextEventTime=nextPlotTime;
        }
        else
          eventInfo->nextEventTime=nextPlotTime;
        break;
    }

    // ***** step event (currently only for plotting) *****

    // not used currently (see completedIntegratorStep) -> maybe check for required drift correction and apply it
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
    if(dss) {
      dss->setTime(time);
      dss->setState(z);
      dss->resetUpToDate();
      dss->solveAndPlot();
    }

    // delete DynamicSystemSolver (requried here since after terminate a call to initialize is allowed without
    // calls to fmiFreeModelInstance and fmiInstantiateModel)
    dss.reset();
  }

  // FMI helper functions

  // print exceptions using the FMI logger
  void FMIInstance::logException(const std::exception &ex) {
    logger(this, instanceName.c_str(), fmiError, "error", ex.what());
  }

  // rethrow a exception thrown during a operation on a valueReference: prefix the exception text with the variable name.
  void FMIInstance::rethrowVR(size_t vr, const std::exception &ex) {
    throw runtime_error(string("In variable '#")+var[vr]->getDatatypeChar()+lexical_cast<string>(vr)+"#': "+ex.what());
  }

}
