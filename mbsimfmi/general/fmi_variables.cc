#include "config.h"
#include "fmi_variables.h"
#include <mbsim/dynamic_system_solver.h>

namespace {
  //! get all links form sys, recursively
  void getAllLinks(const MBSim::DynamicSystem *sys, std::vector<MBSim::Link*> &link);
}

namespace MBSimFMI {

// create predefined parameters
void addPredefinedParameters(bool cosim, std::vector<std::shared_ptr<Variable> > &var,
                             PredefinedParameterStruct &predefinedParameterStruct,
                             bool setToDefaultValue) {
  // output directory
  var.push_back(std::make_shared<PredefinedParameter<std::string> >("'Output directory'",
    "MBSim output directory for all files: *.mbsim.h5, *.ombv.h5, *.ombv.xml, ...", std::ref(predefinedParameterStruct.outputDir)));
  // default value: current dir
  if(setToDefaultValue)
    (*--var.end())->setValue(std::string("."));

  if(!cosim) { // the following parameters are part of the Integrator config. For ME we need to defined these there explicitly
    // plot mode
    // generate enumeration list
    Variable::EnumList plotModeList=std::make_shared<Variable::EnumListCont>();
    // PlotMode::EverynthCompletedStep
    plotModeList->push_back(std::make_pair<std::string, std::string>("Every n-th completed step",
      "Plot each n-th completed integrator step, with n = 'Plot.each n-th step'."));
    // PlotMode::NextCompletedStepAfterSampleTime
    plotModeList->push_back(std::make_pair<std::string, std::string>("Next completed step after sample time",
      "Plot at the first completed step after sample time, with sample time = 'Plot.sample time'."));
    // PlotMode::SampleTime
    plotModeList->push_back(std::make_pair<std::string, std::string>("Constant sample time",
      "Plot in equidistant sample times, with sample time = 'Plot.sample time'."));
    // add variable
    var.push_back(std::make_shared<PredefinedParameter<int> >("Plot.mode",
      "Write to *.mbsim.h5 and *.ombv.h5 files at every ...", std::ref(predefinedParameterStruct.plotMode), plotModeList));
    // default value
    if(setToDefaultValue)
      (*--var.end())->setValue(int(NextCompletedStepAfterSampleTime));

    // plot at each n-th integrator step
    var.push_back(std::make_shared<PredefinedParameter<int> >("Plot.'each n-th step'",
      "... n-th completed integrator step", std::ref(predefinedParameterStruct.plotEachNStep)));
    // default value: every 5-th step
    if(setToDefaultValue)
      (*--var.end())->setValue(int(5));

    // plot every dt
    var.push_back(std::make_shared<PredefinedParameter<double> >("Plot.'sample time'",
      "... sample point with this sample time", std::ref(predefinedParameterStruct.plotStepSize)));
    // default value: every 1ms
    if(setToDefaultValue)
      (*--var.end())->setValue(double(0.001));

    // gMax
    var.push_back(std::make_shared<PredefinedParameter<double> >("'Constraint tolerance'.position",
      "Tolerance for position constraints", std::ref(predefinedParameterStruct.gMax)));
    // default value: 1e-5
    if(setToDefaultValue)
      (*--var.end())->setValue(double(1e-5));

    // gdMax
    var.push_back(std::make_shared<PredefinedParameter<double> >("'Constraint tolerance'.velocity",
      "Tolerance for velocity constraints", std::ref(predefinedParameterStruct.gdMax)));
    // default value: 1e-5
    if(setToDefaultValue)
      (*--var.end())->setValue(double(1e-5));
  }

  // ADD HERE MORE PREDEFINED PARAMETERS
}

void addModelInputOutputs(std::vector<std::shared_ptr<Variable> > &var,
                          const MBSim::DynamicSystemSolver *dss) {
  // create all input/output variables for links in the dss
  std::vector<MBSim::Link*> l;
  getAllLinks(dss, l);
  for(std::vector<MBSim::Link*>::const_iterator it=l.begin(); it!=l.end(); ++it) {
    // for ExternSignalSource create one input variable for each element of the signal vector
    auto *sigSource=dynamic_cast<MBSimControl::ExternSignalSource*>(*it);
    if(sigSource)
      for(int idx=0; idx<sigSource->evalSignal().size(); idx++) {
        var.push_back(std::make_shared<ExternSignalSourceInput>(sigSource, idx));
        (*--var.end())->setValue(double(0.0)); // default value
      }
    // for ExternSignalSink create one output variable for each element of the signal vector
    auto *sigSink=dynamic_cast<MBSimControl::ExternSignalSink*>(*it);
    if(sigSink)
      for(int idx=0; idx<sigSink->evalSignal().size(); idx++)
        var.push_back(std::make_shared<ExternSignalSinkOutput>(sigSink, idx));
    // ADD HERE MORE MBSIM TYPES WHICH SHOULD BECOME FMI INPUTS/OUTPUTS
  }
}

}

namespace {

  void getAllLinks(const MBSim::DynamicSystem *sys, std::vector<MBSim::Link*> &link) {
    // add all link of dss
    const std::vector<MBSim::Link*> &l=sys->getLinks();
    link.insert(link.end(), l.begin(), l.end());
  
    // call recursively for all DynamicSystem
    const std::vector<MBSim::DynamicSystem*> &s=sys->getDynamicSystems();
    for(auto it : s)
      getAllLinks(it, link);
  }

}
