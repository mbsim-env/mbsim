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

#ifndef _MODELINSTANCE_H
#define _MODELINSTANCE_H

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <fmatvec/fmatvec.h>
#include <mbsim/dynamic_system_solver.h>
#include "fmiModelFunctions.h"
#include "fmi_scalar_variable.h"
#include "fmi_enum.h"

#define not_modelError (modelInstantiated|modelInitialized|modelTerminated)

namespace fmi {

/**
 * \brief Contains the parameters of the model in the FMU
 * \author F.Péan
 * \date 2014-03-28 Adding doxygen comments (F.Péan)
 */
class FmuParameters {
public:

  FmuParameters();
  virtual ~FmuParameters(){};

  fmiString guid() { return model_guid; }
  fmiString xmlpath() {return mbsimflatxmlfile.c_str(); }
  fmiInteger& get(paramChoice c) {
    switch(c){
    case nr : return n_of_reals;
    case ni : return n_of_integers;
    case nb : return n_of_booleans;
    case ns : return n_of_strings;
    case ne : return n_of_event_indicators;
    case nstates : return n_of_states;
    default : return n_of_reals;
    }
  }

  void setNStates(const MBSim::DynamicSystemSolver *dss) {
    fmiInteger nq = dss->getqSize();
    fmiInteger nu = dss->getuSize();
    fmiInteger nx = dss->getxSize();
    n_of_states = nq + nu + nx;
  }

  void setNEvents(const MBSim::DynamicSystemSolver *dss) {
    n_of_event_indicators=dss->getsvSize();
  }

private:
  std::string mbsimflatxmlfile;
  fmiString model_guid;
  fmiInteger n_of_reals;
  fmiInteger n_of_integers;
  fmiInteger n_of_booleans;
  fmiInteger n_of_strings;
  fmiInteger n_of_event_indicators;
  fmiInteger n_of_states;
  fmiInteger n_of_inputs;
  fmiInteger n_of_outputs;
};

#include <sstream>
#include <string>
#include <fmatvec/atom.h>
#include <mbsim/mbsim_event.h>
/**
 * \brief Allow to display msg only through callback logger
 * \author F.Péan
 * \date 2014-07-23 Adding doxygen comments (F.Péan)
 */
class LoggerBuffer : public std::stringbuf {
 public:
   LoggerBuffer(fmiCallbackLogger logger_, void *c_, const std::string &instanceName_, const std::string &category_) :
       std::stringbuf(std::ios_base::out),
       logger(logger_),
       c(c_),
       instanceName(instanceName_),
       category(category_) {
   }
 protected:
   fmiCallbackLogger logger;
   void *c;
   std::string instanceName;
   std::string category;

   int sync(); // overwrite the sync function from stringbuf
};

/**
 * \brief Contains the model from MBSim and allows interaction with it from FMI
 * \author F.Péan
 * \date 2014-03-28 Adding doxygen comments (F.Péan)
 */
class ModelInstance {
public:
  ModelInstance(MBSim::DynamicSystemSolver *s_,FmuParameters *p_, fmiString instanceName_, fmiCallbackFunctions functions_, fmiBoolean loggingOn_);
  virtual ~ModelInstance();

  /**
   * \brief configures the initial data required for interface
   */
  void setupIO();
  void setupR();
  void setupI();
  void setupB();
  void setupS();
  void setupE();
  void setupzNames();
  /**
   * \brief called at init, applies parameter in the model
   */
  void initialize();
  /*****************************************************/

  /**
   * \brief performs the recaching of the selected data
   * \param paramChoice what kind of data is to be recached
   * \param pull data from inner model or push newly changed data in the inner model
   */
  void update(paramChoice c, fmiBoolean pullPush);
  void update(fmiBoolean pullPush);
  void eventUpdate(fmiReal t);
  fmiBoolean updateSV(fmiReal t);

  /* Derivative GETTERS */
  const fmiReal* Zdot() { return system->zdot(z,time)(); }
  const fmiReal* Zdot() const { return system->zdot(z,time)(); }
  /*****************************************************/

  /* Miscellaneous SETTERS */
  void setFmiCallbacksFunctions(fmiCallbackFunctions f) {this->functions=f; }
  void setTime(fmiReal t) { time = t; }
  void setLog(fmiBoolean b) { loggingOn = b; }
  void setState(ModelState s) { state = s; }
  void setStep(ModelStep s) { step = s; }
  /*****************************************************/

  /* Miscellaneous GETTERS */
  const MBSim::DynamicSystemSolver& getSystem() const { return *system; }
  const fmiCallbackFunctions& getFunctions() { return functions; }
  const fmiReal getTime() { return time; }
  const fmiString getName() { return instanceName.c_str(); }
  const fmiBoolean getLog() { return loggingOn; }
  const ModelState getState () { return state; }
  const ModelStep getStep() { return step; }
  const fmiString getzName(fmiInteger i) { return zName.at(i).c_str(); }
  /*****************************************************/

  /* ScalarVariable GETTERS */
  const std::vector<fmiReal>& getR() { return r(); }
  const fmiReal& R(fmiInteger index) const { return r(index); }
  const std::vector<fmiInteger>& getI() { return i(); }
  const fmiInteger& I(fmiInteger index) const { return i(index); }
  const std::vector<fmiBoolean>& getB() { return b(); }
  const fmiBoolean& B(fmiInteger index) const { return b(index); }
  const std::vector<std::string>& getS() { return s(); }
  const fmiString S(fmiInteger index) const { return s(index).c_str(); }
  /* States GETTER */
  const fmiReal* Z(void) const { return z(); }
  /* Events GETTER */
  const fmiReal* E(void) const { return eventIndicators[0](); }
  /*****************************************************/

  /* ScalarVariable SETTERS */
  fmiReal& R(fmiInteger index);
  fmiStatus R(fmiInteger index, fmiReal value);
  fmiInteger& I(fmiInteger index);
  fmiStatus I(fmiInteger index, fmiInteger value);
  fmiBoolean& B(fmiInteger index);
  fmiStatus B(fmiInteger index, fmiBoolean value);
  std::string& S(fmiInteger index);
  fmiStatus S(fmiInteger index, fmiString value);
  /* States SETTER */
  fmiReal* Z(void) { return z(); }
  /*****************************************************/

  /* FMU parameters GETTERS / SETTERS */
  const fmiInteger& getN(const paramChoice c) const { return params->get(c); }
  void setN(const paramChoice c, fmiInteger n) { params->get(c)=n; }
  /* xml-path GETTER */
  fmiString xmlpath() { return params->xmlpath(); }
  /*****************************************************/

  // helper function to print exceptions message texts via the FMI logger
  void printErrorMessage(const std::string &msg) {
    functions.logger(this, instanceName.c_str(), fmiError, "error", msg.c_str());
  }

private:
  MBSim::DynamicSystemSolver *system;
  FmuParameters *params;

  ScalarVariable<fmiReal> r;
  ScalarVariable<fmiInteger> i;
  ScalarVariable<fmiBoolean> b;
  ScalarVariable<std::string> s;
  
  //corresponds to jsv in MBSim: result of event indication
  fmatvec::Vector<fmatvec::Ref, fmiInteger> eventFlags;
  //contains values of event indicators at t and t+1 used to decide if there is an event
  fmatvec::Vector<fmatvec::Ref, fmiReal> eventIndicators[2];
  
  //state vector
  fmatvec::Vector<fmatvec::Ref, fmiReal> z;
  //state names
  std::vector< std::string > zName;

  fmiReal time;
  std::string instanceName;
  fmiCallbackFunctions functions;
  fmiBoolean loggingOn;
  //state describing model as seen in FMI_for_ModelExchange - 2.9 State Machine of calling sequence
  ModelState state;
  ModelStep step;
  //buffers to redirect message of MBSim into callback function
  LoggerBuffer infoBuffer;
  LoggerBuffer warnBuffer;

  /* Template functions */
  /**
   * \brief checks if the call for index i in scalar variable sv is authorized for current state of the model
   * \param idx index of the value to be considered
   * \param sv ScalarVariable object to look in
   */
  template<class T>
  const fmiBoolean isAuthorized(fmiInteger idx, ScalarVariable<T>& sv) {
    if((state & modelInstantiated) && !sv.isConstant(idx)) return fmiTrue;
    if((step & setInputs) && sv.isInput(idx) && sv.isDiscrete(idx)) return fmiTrue;
    if(sv.isNone(idx)) return fmiTrue;
    else return fmiFalse;
  }
  /**
   * \brief get the value of idx from scalar variable sv
   * \param idx index of the value to be considered
   * \param sv ScalarVariable object to look in
   */
  template<class T>
  T& getValue(fmiInteger idx, ScalarVariable<T>& sv) {
    if(isAuthorized(idx,sv)) return sv(idx); else {state=modelError;return sv.nullRef;}
  }
  /**
   * \brief set the value value of index idx in scalar variable sv
   * \param idx index of the value to be considered
   * \param value to set
   * \param sv ScalarVariable object to look in
   */
  template<class T>
  fmiStatus setValue(fmiInteger idx, T value, ScalarVariable<T>& sv) {
    if(isAuthorized(idx,sv)) {
      sv(idx)=value;
      return fmiOK;
    } else {
      state=modelError;
      return fmiError;
    }
  }
  /*****************************************************/

};
/* Template specialization for Reals*/
template<> const fmiBoolean ModelInstance::isAuthorized(fmiInteger, ScalarVariable<fmiReal>&);
template<> fmiStatus ModelInstance::setValue(fmiInteger idx,fmiReal value, ScalarVariable<fmiReal>& r);

}//end namespace fmi

#endif // MODELINSTANCE_H
