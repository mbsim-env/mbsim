#ifndef _MODELINSTANCE_H
#define _MODELINSTANCE_H

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <fmatvec/fmatvec.h>
#include <mbsim/dynamic_system_solver.h>
#include "fmiModelFunctions.h"

#ifndef MODEL_GUID
#define MODEL_GUID "TEST"
#endif
#ifndef NUMBER_OF_REALS
#define NUMBER_OF_REALS 4
#endif
#ifndef NUMBER_OF_INTEGERS
#define NUMBER_OF_INTEGERS 0
#endif
#ifndef NUMBER_OF_BOOLEANS
#define NUMBER_OF_BOOLEANS 0
#endif
#ifndef NUMBER_OF_STRINGS
#define NUMBER_OF_STRINGS 0
#endif
#ifndef NUMBER_OF_STATES
#define NUMBER_OF_STATES 4
#endif
#ifndef NUMBER_OF_EVENT_INDICATORS
#define NUMBER_OF_EVENT_INDICATORS 0
#endif

#define not_modelError (modelInstantiated|modelInitialized|modelTerminated)

namespace fmi {

typedef enum {
    modelInstantiated = 1<<0,
    modelInitialized  = 1<<1,
    modelTerminated   = 1<<2,
    modelError        = 1<<3
} ModelState;

class ModelInstance {
public:
  ModelInstance(MBSim::DynamicSystemSolver *s, fmiString instanceName, fmiString GUID, fmiCallbackFunctions functions, fmiBoolean loggingOn);

  virtual ~ModelInstance();

  static fmiCallbackFunctions functions; // STATIC, VOLATILE ?
  static void setFmiCallbacksFunctions(fmiCallbackFunctions functions){ModelInstance::functions=functions;}

  void initialize();
  std::vector<fmiReal> update();


  const MBSim::DynamicSystemSolver& getSystem() const { return *system;}

  void setTime(fmiReal t) {time = t;}
  void enableLog(fmiBoolean b) {loggingOn = b;}
  void setState(ModelState s) {state = s;}

  fmiReal getTime() {return time;}
  fmiString getName() {return instanceName;}
  fmiString getGUID() {return GUID;}
  fmiBoolean getLog() {return loggingOn;}
  ModelState getState () {return state;}

  const fmatvec::Vec& getR() {return r;}
  const std::vector<fmiInteger>& getI() {return i;};
  const std::vector<fmiBoolean>& getB() {return b;};
  const std::vector<fmiString>& getS() {return s;};

  fmatvec::Vector<fmatvec::Ref,fmiReal> r;
  std::vector<fmiInteger> i;
  std::vector<fmiBoolean> b;
  std::vector<fmiString> s;
  std::vector<fmiBoolean> isPositive;
private:
  MBSim::DynamicSystemSolver *system; //or more directly the user-system ?
  fmiReal time;
  fmiString instanceName;
  fmiString GUID;
  fmiBoolean loggingOn;
  ModelState state;
};

}//end namespace fmi

#endif // MODELINSTANCE_H
