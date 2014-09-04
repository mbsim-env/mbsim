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
#include "fmiModelTypes.h"
#ifdef __cplusplus
extern "C" {
#endif
#define MODEL_IDENTIFIER mbsim
#include "fmiModelFunctions.h"
#ifdef __cplusplus
}
#endif
#include "model_instance.h"
#include "system.h"
#include <string>
#include <boost/filesystem.hpp>
#include <mbsim/mbsim_event.h>

#define n_of_reals comp->getN(nr)
#define n_of_integers comp->getN(ni)
#define n_of_booleans comp->getN(nb)
#define n_of_strings comp->getN(ns)
#define n_of_event_indicators comp->getN(ne)
#define n_of_states comp->getN(nstates)

#define CATCH_PRINT_AND_RETURN(value) \
 catch(const MBSim::MBSimError &ex) { \
   comp->printErrorMessage(std::string("MBSim exception: ")+ex.what()); \
 } \
 catch(const std::runtime_error &ex) { \
   comp->printErrorMessage(std::string("Exception: ")+ex.what()); \
 } \
 catch(...) { \
   comp->printErrorMessage("Unknown exception"); \
 } \
 return value;

using namespace fmi;

// ---------------------------------------------------------------------------
// Private helpers used below to validate function arguments
// ---------------------------------------------------------------------------

static fmiBoolean invalidNumber(ModelInstance* comp, const char* f, const char* arg, int n, int nExpected){
    if (n != nExpected) {
        comp->setState(modelError);
        comp->getFunctions().logger(comp, comp->getName(), fmiError, "error",
                "%s: Invalid argument %s = %d. Expected %d.", f, arg, n, nExpected);
        return fmiTrue;
    }
    return fmiFalse;
}

static fmiBoolean invalidState(ModelInstance* comp, const char* f, int statesExpected){
    if (!comp)
        return fmiTrue;
    if (!(comp->getState() & statesExpected)) {
        comp->setState(modelError);
        comp->getFunctions().logger(comp, comp->getName(), fmiError, "error",
                "%s: Illegal call sequence. Wrong state.", f);
        return fmiTrue;
    }
    return fmiFalse;
}

static fmiBoolean invalidStep(ModelInstance* comp, const char* f, int stepExpected){
    if (!comp)
        return fmiTrue;
    if (!(comp->getStep() & stepExpected)) {
        comp->setState(modelError);
        comp->getFunctions().logger(comp, comp->getName(), fmiError, "error",
                "%s: Illegal call sequence. Wrong step.", f);
        return fmiTrue;
    }
    return fmiFalse;
}

static fmiBoolean nullPointer(ModelInstance* comp, const char* f, const char* arg, const void* p){
    if (!p) {
        comp->setState(modelError);
        comp->getFunctions().logger(comp, comp->getName(), fmiError, "error",
                "%s: Invalid argument %s = NULL.", f, arg);
        return fmiTrue;
    }
    return fmiFalse;
}

static fmiBoolean vrOutOfRange(ModelInstance* comp, const char* f, fmiValueReference vr, int end) {
    if (vr >= (fmiValueReference) end) {
        comp->getFunctions().logger(comp, comp->getName(), fmiError, "error",
                "%s: Illegal value reference %u.", f, vr);
        comp->setState(modelError);
        return fmiTrue;
    }
    return fmiFalse;
}

// ---------------------------------------------------------------------------
// FMI functions: class methods not depending of a specific model instance
// ---------------------------------------------------------------------------

const char* fmiGetModelTypesPlatform() {
    return fmiModelTypesPlatform;
}

const char* fmiGetVersion() {
    return fmiVersion;
}

// ---------------------------------------------------------------------------
// FMI functions: creation and destruction of a model instance
// ---------------------------------------------------------------------------

fmiComponent fmiInstantiateModel (fmiString            instanceName,
                                  fmiString            GUID,
                                  fmiCallbackFunctions functions,
                                  fmiBoolean           loggingOn){

  ModelInstance* comp;
  FmuParameters* fmuParams = new FmuParameters();
//  fmuParams = (FmuParameters*)functions.allocateMemory(1, sizeof(FmuParameters));
//  new(fmuParams) FmuParameters();

  /* CHECK */
  if (!functions.logger)
    return NULL;
  if (!functions.allocateMemory || !functions.freeMemory){
    functions.logger(NULL, instanceName, fmiError, "error",
            "fmiInstantiateModel: Missing callback function.");
    return NULL;
  }
  if (!instanceName || strlen(instanceName)==0) {
    functions.logger(NULL, instanceName, fmiError, "error",
            "fmiInstantiateModel: Missing instance name.");
    return NULL;
  }
  if (strcmp(GUID, fmuParams->guid())) {
    functions.logger(NULL, instanceName, fmiError, "error",
            "fmiInstantiateModel: Wrong GUID %s. Expected %s.", GUID, fmuParams->guid());
    return NULL;
  }
  /*****************************************************/

  System* sys = 0;
  if(!boost::filesystem::exists(fmuParams->xmlpath())){
    functions.logger(NULL, instanceName, fmiWarning, "log",
            "No mbsim xml flat input file, creates system with compile-time system implemented.");
    sys = new System(instanceName);
  }

  comp = (ModelInstance*)functions.allocateMemory(1, sizeof(ModelInstance));
  new(comp) ModelInstance(sys,fmuParams, instanceName, functions, loggingOn);

  if( !comp ) {
    comp->getFunctions().logger(NULL, instanceName, fmiError, "error",
              "fmiInstantiateModel: Out of memory.");
      return NULL;
  }
  try {
    if (comp->getLog()) comp->getFunctions().logger(NULL, instanceName, fmiOK, "log",
        "fmiInstantiateModel: GUID=%s", GUID);
    comp->setState(modelInstantiated);
    comp->update(0);//pull
    return comp;
  } CATCH_PRINT_AND_RETURN(NULL)
}

void fmiFreeModelInstance(fmiComponent c) {
    ModelInstance* comp = (ModelInstance *)c;
    if (!comp) return;
    if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
            "fmiFreeModelInstance");
    comp->getFunctions().freeMemory(comp);
    comp=NULL;
}

fmiStatus fmiSetDebugLogging  (fmiComponent c, fmiBoolean loggingOn){
  ModelInstance* comp = (ModelInstance *)c;
  try {
    /* CHECK */
    if (invalidState(comp, "fmiSetDebugLogging", not_modelError))
         return fmiError;
    /*****************************************************/
    if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
            "fmiSetDebugLogging: loggingOn=%d", loggingOn);
    comp->setLog(loggingOn);
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

/* Providing independent variables and re-initialization of caching */
fmiStatus fmiSetTime(fmiComponent c, fmiReal time){
  ModelInstance* comp = (ModelInstance *)c;
  try {
    /* CHECK */
    if (invalidState(comp, "fmiSetTime", modelInstantiated|modelInitialized))
         return fmiError;
    if (invalidStep(comp, "fmiSetTime", stepAccepted | stepInProgress))
         return fmiError;
    /*****************************************************/
    if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
            "fmiSetTime: time=%.16g", time);
    comp->update(0);//pull
    comp->setTime(time);
    comp->setStep(stepInProgress);
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

fmiStatus fmiSetContinuousStates    (fmiComponent c, const fmiReal x[], size_t nx){
  ModelInstance* comp = (ModelInstance *)c;
  try {
    /* CHECK */
    if (invalidState(comp, "fmiSetContinuousStates", modelInitialized))
         return fmiError;
    if (invalidStep(comp, "fmiSetContinuousStates", stepInProgress))
         return fmiError;
    if (invalidNumber(comp, "fmiSetContinuousStates", "nx", nx, n_of_states))
        return fmiError;
    if (nullPointer(comp, "fmiSetContinuousStates", "x[]", x))
         return fmiError;
    /*****************************************************/
    fmiReal* z=comp->Z();
    std::memcpy(z,x,nx*sizeof(fmiReal));
    if (comp->getLog())
      for(int i=0; i<(int)nx;i++) {
        // LOG
        comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
            "fmiSetContinuousStates: %s=%.16g", comp->getzName(i), x[i]);
      }
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}
fmiStatus fmiCompletedIntegratorStep(fmiComponent c, fmiBoolean* callEventUpdate){
  ModelInstance* comp = (ModelInstance *)c;
  try {
    /* CHECK */
    if (invalidState(comp, "fmiCompletedIntegratorStep", modelInitialized))
         return fmiError;
    if (invalidStep(comp, "fmiCompletedIntegratorStep", stepInProgress))
         return fmiError;
    if (nullPointer(comp, "fmiCompletedIntegratorStep", "callEventUpdate", callEventUpdate))
         return fmiError;
    /*****************************************************/
    // LOG
    if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
            "fmiCompletedIntegratorStep");
    // COMPUTATION
    comp->update(0);//pull
    if(*callEventUpdate) {
      comp->setStep(setInputs);
      *callEventUpdate = fmiFalse;
    }
    else comp->setStep(stepAccepted);
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

// ---------------------------------------------------------------------------
// FMI functions: set variable values in the FMU
// ---------------------------------------------------------------------------

fmiStatus fmiSetReal(fmiComponent c, const fmiValueReference vr[], size_t nvr, const fmiReal    value[]){
  int i;
  ModelInstance* comp = (ModelInstance *)c;
  try {
    /* CHECK */
    if (invalidState(comp, "fmiSetReal", modelInstantiated|modelInitialized))
         return fmiError;
    if (invalidStep(comp, "fmiSetReal", stepUndefined|setInputs|stepInProgress))
         return fmiError;
    if (nvr>0 && nullPointer(comp, "fmiSetReal", "vr[]", vr))
         return fmiError;
    if (nvr>0 && nullPointer(comp, "fmiSetReal", "value[]", value))
         return fmiError;
    /*****************************************************/
    // LOG
    if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
            "fmiSetReal: nvr = %d", nvr);

    for (i = 0; i < (int)nvr; i++) {
      // CHECK
      if (vrOutOfRange(comp, "fmiSetReal", vr[i], n_of_reals))
         return fmiError;
      // LOG
      if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
          "fmiSetReal: #r%d# = %.16g", vr[i], value[i]);
      // UPDATE
      comp->R(vr[i]) = value[i];
    }
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

fmiStatus fmiSetInteger(fmiComponent c, const fmiValueReference vr[], size_t nvr, const fmiInteger value[]){
  int i;
  ModelInstance* comp = (ModelInstance *)c;
  try {
    /* CHECK */
    if (invalidState(comp, "fmiSetInteger", modelInstantiated|modelInitialized))
         return fmiError;
    if (invalidStep(comp, "fmiSetReal", stepUndefined|setInputs))
         return fmiError;
    if (nvr>0 && nullPointer(comp, "fmiSetInteger", "vr[]", vr))
         return fmiError;
    if (nvr>0 && nullPointer(comp, "fmiSetInteger", "value[]", value))
         return fmiError;
    /*****************************************************/
    // LOG
    if (comp->getLog())
        comp->getFunctions().logger(c, comp->getName(), fmiOK, "log", "fmiSetInteger: nvr = %d",  nvr);
    for (i = 0; i < (int)nvr; i++) {
      // CHECK
      if (vrOutOfRange(comp, "fmiSetInteger", vr[i], n_of_integers))
        return fmiError;
      // LOG
      if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
          "fmiSetInteger: #i%d# = %d", vr[i], value[i]);
      // UPDATE
      comp->I(vr[i]) = value[i];
    }
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}
fmiStatus fmiSetBoolean(fmiComponent c, const fmiValueReference vr[], size_t nvr, const fmiBoolean value[]){
  int i;
  ModelInstance* comp = (ModelInstance *)c;
  try {
    /* CHECK */
    if (invalidState(comp, "fmiSetBoolean", modelInstantiated|modelInitialized))
         return fmiError;
    if (invalidStep(comp, "fmiSetReal", stepUndefined|setInputs))
         return fmiError;
    if (nvr>0 && nullPointer(comp, "fmiSetBoolean", "vr[]", vr))
         return fmiError;
    if (nvr>0 && nullPointer(comp, "fmiSetBoolean", "value[]", value))
         return fmiError;
    /*****************************************************/
    // LOG
    if (comp->getLog())
        comp->getFunctions().logger(c, comp->getName(), fmiOK, "log", "fmiSetBoolean: nvr = %d",  nvr);
    for (i = 0; i < (int)nvr; i++) {
      // CHECK
      if(vrOutOfRange(comp, "fmiSetBoolean", vr[i], n_of_booleans))
            return fmiError;
      // LOG
      if(comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
          "fmiSetBoolean: #b%d# = %s", vr[i], value[i] ? "true" : "false");
      // UPDATE
      comp->B(vr[i]) = value[i];
    }
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

fmiStatus fmiSetString(fmiComponent c, const fmiValueReference vr[], size_t nvr, const fmiString  value[]){
  int i;
  ModelInstance* comp = (ModelInstance *)c;
  try {
    /* CHECK */
    if(invalidState(comp, "fmiSetString", modelInstantiated|modelInitialized))
      return fmiError;
    if(invalidStep(comp, "fmiSetReal", stepUndefined|setInputs))
      return fmiError;
    if(nvr>0 && nullPointer(comp, "fmiSetString", "vr[]", vr))
      return fmiError;
    if(nvr>0 && nullPointer(comp, "fmiSetString", "value[]", value))
      return fmiError;
    /*****************************************************/
    // LOG
    if (comp->getLog())
      comp->getFunctions().logger(c, comp->getName(), fmiOK, "log", "fmiSetString: nvr = %d",  nvr);
    for (i = 0; i < (int)nvr; i++) {
      // CHECK
      if (vrOutOfRange(comp, "fmiSetString", vr[i], n_of_strings))
        return fmiError;
      // LOG
      if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
           "fmiSetString: #s%d# = '%s'", vr[i], value[i]);
      // UPDATE
      comp->S(vr[i]) = value[i];
    }
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

/* Evaluation of the model equations */
fmiStatus fmiInitialize(fmiComponent c, fmiBoolean toleranceControlled,
                        fmiReal relativeTolerance, fmiEventInfo* eventInfo){
  ModelInstance* comp = (ModelInstance *)c;
  try {
    /* CHECK */
    if (invalidState(comp, "fmiInitialize", modelInstantiated))
         return fmiError;
    if (nullPointer(comp, "fmiInitialize", "eventInfo", eventInfo))
         return fmiError;
    if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
        "fmiInitialize: toleranceControlled=%d relativeTolerance=%g",
        toleranceControlled, relativeTolerance);
    /*****************************************************/
    // UPDATE
    eventInfo->iterationConverged  = fmiTrue;
    eventInfo->stateValueReferencesChanged = fmiFalse;
    eventInfo->stateValuesChanged  = fmiFalse;
    eventInfo->terminateSimulation = fmiFalse;
    eventInfo->upcomingTimeEvent   = fmiFalse;
    comp->initialize();
    comp->update(1);//push
    comp->setState(modelInitialized);
    comp->setStep(stepAccepted);
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

fmiStatus fmiGetDerivatives    (fmiComponent c, fmiReal derivatives[]    , size_t nx){
  ModelInstance* comp = (ModelInstance *)c;
  const ModelInstance* c_comp = (const ModelInstance*) comp;
  try {
  /* CHECK */
  if (invalidState(comp, "fmiGetDerivatives", not_modelError))
       return fmiError;
  if (invalidNumber(comp, "fmiGetDerivatives", "nx", nx, n_of_states))
      return fmiError;
  if (nullPointer(comp, "fmiGetDerivatives", "derivatives[]", derivatives))
       return fmiError;
  /*****************************************************/
  // UPDATE
  const fmiReal* zdot = c_comp->Zdot();
  std::memcpy(derivatives,zdot,nx*sizeof(fmiReal));
  // LOG
  if (comp->getLog())
    for(int i=0;i<n_of_states;i++) {
      comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
          "fmiGetDerivatives: %s = %.16g", comp->getzName(i), derivatives[i]);
    }
  return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

fmiStatus fmiGetEventIndicators(fmiComponent c, fmiReal eventIndicators[], size_t ni){
  ModelInstance* comp = (ModelInstance *)c;
  const ModelInstance* c_comp = (const ModelInstance*) comp;
  try {
    /* CHECK */
    if (invalidState(comp, "fmiGetEventIndicators", not_modelError))
        return fmiError;
    if (invalidNumber(comp, "fmiGetEventIndicators", "ni", ni, n_of_event_indicators))
        return fmiError;
    /*****************************************************/
    // UPDATE
    if(n_of_event_indicators > 0){
      comp->updateSV(comp->getTime());
      std::memcpy(eventIndicators,c_comp->E(),ni*sizeof(fmiReal));
      // LOG
      for (int i=0; i< (int)ni; i++) {
          if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
              "fmiGetEventIndicators: z%u = %.16g", i, c_comp->E()[i]);
      }
    }
  return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

fmiStatus fmiGetReal   (fmiComponent c, const fmiValueReference vr[], size_t nvr, fmiReal value[]){
  ModelInstance* comp = (ModelInstance *)c;
  const ModelInstance* c_comp = (const ModelInstance*) comp;
  try {
    /* CHECK */
    if (invalidState(comp, "fmiGetReal", not_modelError))
        return fmiError;
    if (nvr>0 && nullPointer(comp, "fmiGetReal", "vr[]", vr))
         return fmiError;
    if (nvr>0 && nullPointer(comp, "fmiGetReal", "value[]", value))
         return fmiError;
    /*****************************************************/
    int i;
    for (i=0; i<(int)nvr; i++) {
      // CHECK
      if (vrOutOfRange(comp, "fmiGetReal", vr[i], n_of_reals))
        return fmiError;
      // UPDATE
      value[i] = c_comp->R(vr[i]);
      // LOG
      if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
            "fmiGetReal: #r%u# = %.16g", vr[i], value[i]);
    }
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

fmiStatus fmiGetInteger(fmiComponent c, const fmiValueReference vr[], size_t nvr, fmiInteger value[]){
  ModelInstance* comp = (ModelInstance *)c;
  const ModelInstance* c_comp = (const ModelInstance*) comp;
  try {
    /* CHECK */
    if(invalidState(comp, "fmiGetInteger", not_modelError))
      return fmiError;
    if(nvr>0 && nullPointer(comp, "fmiGetInteger", "vr[]", vr))
      return fmiError;
    if(nvr>0 && nullPointer(comp, "fmiGetInteger", "value[]", value))
     return fmiError;
    /*****************************************************/

    int i;
    for (i = 0; i < (int)nvr; i++) {
      //CHECK
      if(vrOutOfRange(comp, "fmiGetInteger", vr[i], n_of_integers))
        return fmiError;
      // UPDATE
      value[i] = c_comp->I(vr[i]);
      // LOG
      if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
              "fmiGetInteger: #i%u# = %d", vr[i], value[i]);
    }
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

fmiStatus fmiGetBoolean(fmiComponent c, const fmiValueReference vr[], size_t nvr, fmiBoolean value[]){
  ModelInstance* comp = (ModelInstance *)c;
  const ModelInstance* c_comp = (const ModelInstance*) comp;
  try {
    /* CHECK */
    if(invalidState(comp, "fmiGetBoolean", not_modelError))
      return fmiError;
    if(nvr>0 && nullPointer(comp, "fmiGetBoolean", "vr[]", vr))
      return fmiError;
    if(nvr>0 && nullPointer(comp, "fmiGetBoolean", "value[]", value))
      return fmiError;
    /*****************************************************/
    int i;
    for (i = 0; i < (int)nvr; i++) {
      // CHECK
      if(vrOutOfRange(comp, "fmiGetBoolean", vr[i], n_of_booleans))
        return fmiError;
      // UPDATE
      value[i] = c_comp->B(vr[i]);
      // LOG
      if (comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
            "fmiGetBoolean: #b%u# = %s", vr[i], value[i]? "true" : "false");
    }
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

fmiStatus fmiGetString (fmiComponent c, const fmiValueReference vr[], size_t nvr, fmiString  value[]){
  int i;
  ModelInstance* comp = (ModelInstance *)c;
  const ModelInstance* c_comp = (const ModelInstance*) comp;
  try {
    /* CHECK */
    if(invalidState(comp, "fmiGetString", not_modelError))
      return fmiError;
    if(nvr>0 && nullPointer(comp, "fmiGetString", "vr[]", vr))
      return fmiError;
    if(nvr>0 && nullPointer(comp, "fmiGetString", "value[]", value))
      return fmiError;
    /*****************************************************/
    for (i = 0; i< (int)nvr; i++) {
      // CHECK
      if(vrOutOfRange(comp, "fmiGetString", vr[i], n_of_strings))
        return fmiError;
      // UPDATE
      std::strcpy(const_cast<char*>(value[i]), c_comp->S(vr[i]));
      // LOG
      if(comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
            "fmiGetString: #s%u# = '%s'", vr[i], value[i]);
    }
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

fmiStatus fmiEventUpdate (fmiComponent c, fmiBoolean intermediateResults, fmiEventInfo* eventInfo){
  ModelInstance* comp = (ModelInstance *)c;
  try {
    /* CHECK */
    if(invalidState(comp, "fmiEventUpdate", modelInitialized))
      return fmiError;
    if(invalidStep(comp, "fmiEventUpdate", setInputs | eventPending))
      return fmiError;
    if(nullPointer(comp, "fmiEventUpdate", "eventInfo", eventInfo))
      return fmiError;
    if(comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
        "fmiEventUpdate: intermediateResults = %d", intermediateResults);
    /*****************************************************/

    // Update model step
    if(comp->getStep() & setInputs) {
      comp->setStep(eventPending);
      comp->update(1);//push
    }
    if(comp->getStep() & eventPending) {
      comp->update(0);//pull
    }
    // Update event information
    eventInfo->stateValueReferencesChanged = fmiFalse;
    eventInfo->upcomingTimeEvent = fmiFalse;
    eventInfo->iterationConverged  = fmiTrue;
    eventInfo->stateValuesChanged  = fmiTrue;
    eventInfo->terminateSimulation = fmiFalse;
    
    comp->eventUpdate(comp->getTime());

    // Update model step
    if(eventInfo->iterationConverged == fmiTrue) comp->setStep(stepAccepted);

    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

fmiStatus fmiGetContinuousStates (fmiComponent c, fmiReal states[], size_t nx){
  ModelInstance* comp = (ModelInstance *)c;
  const ModelInstance* c_comp = (const ModelInstance*) comp;
  try {
    /* CHECK */
    if(invalidState(comp, "fmiGetContinuousStates", not_modelError))
      return fmiError;
    if(invalidNumber(comp, "fmiGetContinuousStates", "nx", nx, n_of_states))
      return fmiError;
    if(nullPointer(comp, "fmiGetContinuousStates", "states[]", states))
      return fmiError;
    /*****************************************************/
    // UPDATE
    const fmiReal* z = c_comp->Z();
    std::memcpy(states,z,nx*sizeof(fmiReal));
    // LOG
    if (comp->getLog())
      for(int i=0;i<(int)nx;i++) {
        comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
            "fmiGetContinuousStates: %s = %.16g", comp->getzName(i), states[i]);
      }
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

fmiStatus fmiGetNominalContinuousStates(fmiComponent c, fmiReal x_nominal[], size_t nx){
  int i;
  ModelInstance* comp = (ModelInstance *)c;
  //const ModelInstance* c_comp = (const ModelInstance*) comp;
  try {
    /* CHECK */
    if(invalidState(comp, "fmiGetNominalContinuousStates", not_modelError))
      return fmiError;
    if(invalidNumber(comp, "fmiGetNominalContinuousStates", "nx", nx, n_of_states))
      return fmiError;
    if(nullPointer(comp, "fmiGetNominalContinuousStates", "x_nominal[]", x_nominal))
      return fmiError;
    /*****************************************************/
    // LOG
    if(comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
        "fmiGetNominalContinuousStates: x_nominal[0..%d] = 1.0", nx-1);
    // UPDATE
    for(i = 0; i < (int)nx; i++)
        x_nominal[i] = 1;
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}
fmiStatus fmiGetStateValueReferences (fmiComponent c, fmiValueReference vrx[], size_t nx){
  ModelInstance* comp = (ModelInstance *)c;
  try {
    /* CHECK */
    if(invalidState(comp, "fmiGetStateValueReferences", not_modelError))
      return fmiError;
    if(invalidNumber(comp, "fmiGetStateValueReferences", "nx", nx, n_of_states))
      return fmiError;
    if(nullPointer(comp, "fmiGetStateValueReferences", "vrx[]", vrx))
      return fmiError;
    /*****************************************************/
    // UPDATE
    for(int i=0;i<n_of_states;i++){
      vrx[i]=fmiUndefinedValueReference;
    }
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}
fmiStatus fmiTerminate (fmiComponent c){
  ModelInstance* comp = (ModelInstance *)c;
  try {
    /* CHECK */
    if(invalidState(comp, "fmiTerminate", modelInitialized))
       return fmiError;
    /*****************************************************/
    // LOG
    if(comp->getLog()) comp->getFunctions().logger(c, comp->getName(), fmiOK, "log",
        "fmiTerminate");
    comp->update(0);//pull
    comp->setState(modelTerminated);
    return fmiOK;
  } CATCH_PRINT_AND_RETURN(fmiError)
}

