/* Copyright (C) 2004-2014 MBSim Development Team
 *
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
 * Contact: thorsten.schindler@mytum.de
 */

#include <config.h>
#include <map>
#include <string>
#include <sstream>
#include <mbsim/object.h>
#include "model_instance.h"
#include "fmi_utils.h"
#include "fmi_enum.h"
#include <mbsim/utils/eps.h>
#include "fmiModelTypes.h"

#define MODEL_GUID "TEST"

using namespace MBSim;

namespace fmi {
  using std::cout;
  using std::endl;
  using std::string;

  FmuParameters::FmuParameters():
    model_guid(MODEL_GUID),
    n_of_reals(0),
    n_of_integers(0),
    n_of_booleans(0),
    n_of_strings(0),
    n_of_event_indicators(0),
    n_of_states(0),
    n_of_inputs(0),
    n_of_outputs(0)
  {
  }

  int LoggerBuffer::sync() {
    // print the current buffer using the FMI logger using type type
    // (skip a trailing new line according the FMI spec)
    string s=str();
    if(*--s.end()=='\n')
      s.resize(s.size()-1);
    logger(c, instanceName.c_str(), fmiOK, category.c_str(), s.c_str());
    // clear the buffer and return
    str("");
    return 0;
  }

  ModelInstance::ModelInstance(MBSim::DynamicSystemSolver *s_,FmuParameters *p_,fmiString instanceName_, fmiCallbackFunctions functions_, fmiBoolean loggingOn_, const string &mbsimflatxmlfile):
    system(s_),
    params(p_),
    r(),
    i(),
    b(),
    s(),
    eventFlags(),
    time(0),
    instanceName(instanceName_),
    functions(functions_),
    loggingOn(loggingOn_),
    state(modelError),
    step(stepUndefined),
    infoBuffer(functions_.logger, this, instanceName, "info"),
    warnBuffer(functions_.logger, this, instanceName, "warning") {
      // use the per FMIInstance provided buffers for all subsequent fmatvec::Atom objects
      fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Info, boost::make_shared<std::ostream>(&infoBuffer));
      fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn, boost::make_shared<std::ostream>(&warnBuffer));
      // also use these streams for this object.
      // Note: we can not create a FMIInstance object with the correct streams but we can adopt the streams now!
      //adoptMessageStreams(); //note: no arg means adopt the current (static) message streams (set above)

      if(!system)
        initDssWithXml(mbsimflatxmlfile,&system);
      system->setPlotFeatureRecursive(DynamicSystemSolver::plotRecursive,DynamicSystemSolver::disabled);//(loggingOn?enabled:disabled));
      //system->setInformationOutput(loggingOn);
      //system->setConstraintSolver(MBSim::RootFinding);
      //system->setImpactSolver(MBSim::RootFinding);
      system->initialize();

      setupR();
      setupI();
      setupB();
      setupS();

      params->setNStates(system);
      params->setNEvents(system);
      setN(nr,r.size());
      setN(ni,i.size());
      setN(ns,s.size());
      setN(nb,b.size());

      z.resize(getN(nstates));
      system->initz(z);
      system->computeInitialCondition();
      setupE();
      setupzNames();
    }

  ModelInstance::~ModelInstance(){
    if(system!=NULL) {
      delete system;
      system=NULL;
    }
    if(params!=NULL) {
      delete params;
      params=NULL;
    }
  }

  void ModelInstance::setupIO(){
  }

  void ModelInstance::setupR() {
    /* add IO of dynamic system */
    std::vector<MBSim::Link*> links;
    system->buildListOfLinks(links);
    // get links from different dyn sys in the same order as the generated xml
    for(size_t j=0;j<links.size();j++) {
      if(links[j]->getType()=="ExternGeneralizedIO")
        r.add(links[j]);
    }
    for(size_t j=0;j<links.size();j++) {
      if(links[j]->getType()=="ExternSignalSource")
        r.add(links[j]);
    }
    for(size_t j=0;j<links.size();j++) {
      if(links[j]->getType()=="ExternSignalSink")
        r.add(links[j]);
    }
    /*****************************************************/
    r.setup();
  }
  void ModelInstance::setupI() {
    i.setup();
  }
  void ModelInstance::setupB() {
    b.setup();
  }
  void ModelInstance::setupS() {
    s.add(string("OutputDir"));
    s.setup();
  }

  void ModelInstance::setupE() {
    eventFlags.resize(getN(ne));
    eventIndicators[0].resize(getN(ne));
    eventIndicators[1].resize(getN(ne),fmatvec::INIT,1.);
    system->getsv(z,eventIndicators[0],time);
  }

  void ModelInstance::setupzNames(){
    const std::vector<MBSim::Object*> objList = system->getObjects();
    int qSize=0,uSize=0;
    for(size_t i = 0; i < objList.size(); i++){
      for(int j = 0; j < objList[i]->getqSize(); j++){
        string name = string(objList[i]->getName()+".q."+NumberToString(j));
        std::replace(name.begin(), name.end(), '/', '.');
        zName.push_back(name);
      }
      qSize+=objList[i]->getqSize();
    }
    for(size_t i = 0; i < objList.size(); i++){
      for(int j = 0; j < objList[i]->getuSize(); j++){
        string name = string(objList[i]->getName()+".u."+NumberToString(j));
        std::replace(name.begin(), name.end(), '/', '.');
        zName.push_back(name);
      }
      uSize+=objList[i]->getuSize();
    }
  }

  void ModelInstance::initialize() {
  }

  void ModelInstance::update(paramChoice c, fmiBoolean pullPush) {
    updateSV(time);
    switch(c) {
      case nr : pullPush?r.push():r.pull();return;
      case ni : pullPush?i.push():i.pull();return;
      case nb : pullPush?b.push():b.pull();return;
      case ns : pullPush?s.push():s.pull();return;
      default : return;
    }
  }
  void ModelInstance::update(fmiBoolean pullPush) {
    updateSV(time);
    pullPush?r.push():r.pull();return;
    pullPush?i.push():i.pull();return;
    pullPush?b.push():b.pull();return;
    pullPush?s.push():s.pull();return;
  }

  void ModelInstance::eventUpdate(fmiReal t) {
    system->getsv(z,eventIndicators[0],t);
    for(int i= 0 ; i < getN(ne); i++) {
      if(((eventIndicators[0])(i)>0.)not_eq(((eventIndicators[1])(i))>0.)) // definition SPEC p. 10
        eventFlags(i)=fmiTrue;
    }
    this->getFunctions().logger(this, this->getName(), fmiOK, "log","SHIFT");
    system->shift(z,eventFlags,t);
    system->getsv(z,eventIndicators[0],t);
    eventIndicators[1]=eventIndicators[0];
    for(int i= 0 ; i < getN(ne); i++) {
      eventFlags(i)=fmiFalse;
    }
  }

  fmiBoolean ModelInstance::updateSV(fmiReal t) {
    system->getsv(z,eventIndicators[0],t);
    return fmiTrue;
  }

  /// ScalarVariable setter definition
  fmiReal& ModelInstance::R(fmiInteger index) { return getValue(index,r); }
  fmiStatus ModelInstance::R(fmiInteger index, fmiReal value) { return setValue(index,value,r); }
  fmiInteger& ModelInstance::I(fmiInteger index) { return getValue(index,i); }
  fmiStatus ModelInstance::I(fmiInteger index, fmiInteger value) { return setValue(index,value,i); }
  fmiBoolean& ModelInstance::B(fmiInteger index) { return getValue(index,b); }
  fmiStatus ModelInstance::B(fmiInteger index, fmiBoolean value) { return setValue(index,value,b); }
  string& ModelInstance::S(fmiInteger index) { return getValue(index,s); }
  fmiStatus ModelInstance::S(fmiInteger index, fmiString value) { return setValue(index,string(value),s); }

  /// Template specialization definition
  template<>
    const fmiBoolean ModelInstance::isAuthorized(fmiInteger i, ScalarVariable<fmiReal> &r) {
      if((state & modelInstantiated) && !r.isConstant(i)) return fmiTrue;
      if((step & stepInProgress) && r.isContinuous(i) && r.isInput(i)) return fmiTrue;
      if((step & setInputs) && r.isInput(i) && (r.isDiscrete(i) | r.isContinuous(i))) return fmiTrue;
      if(r.isNone(i)) return fmiTrue;
      else return fmiFalse;
    }

  template<>
    fmiStatus ModelInstance::setValue(fmiInteger idx,fmiReal value, ScalarVariable<fmiReal>& r) {
      if(isAuthorized(idx,r)) {
        r(idx)=value;
        return fmiOK;
      }
      else {
        state=modelError;
        return fmiError;
      }
    }

}//end namespace fmi
