//#include <mbsim/integrators/integrators.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <mbsim/integrators/integrators.h>
#include <fmatvec/fmatvec.h>
#include "fmiModelTypes.h"
#ifdef __cplusplus
extern "C" {
#endif
#define MODEL_IDENTIFIER mbsim
#include "fmiModelFunctions.h"
#ifdef __cplusplus
}
#endif

#define RESULT_FILE "result.csv"

using namespace std;

void* alloc(size_t num, size_t size){printf("alloc with external calloc\n");return calloc(num,size);}

void wait() {printf("Please press <ENTER> to continue."); while(getchar() != '\n');}

void fmuLogger(fmiComponent c, fmiString instanceName, fmiStatus status,
               fmiString category, fmiString message, ...);
int error(const char* message);
const char* fmiStatusToString(fmiStatus status);

int main (int argc, char* argv[]) {
    int i;
    fmiReal dt, tPre;
    fmiBoolean timeEvent, stateEvent, stepEvent;
    fmiReal time;  
    int nx;                          // number of state variables
    int nz;                          // number of state event indicators
    fmiReal *x;                       // continuous states
    fmiReal *xdot;                    // the crresponding derivatives in same order
    fmiReal *z = NULL;                // state event indicators
    fmiReal *prez = NULL;             // previous values of state event indicators
    fmiEventInfo eventInfo;          // updated by calls to initialize and eventUpdate
    fmiString id;
    fmiString guid;                // global unique id of the fmu
    fmiBoolean loggingOn = 1;
    fmiCallbackFunctions callbacks;  // called by the model during simulation
    fmiComponent c;                  // instance of the fmu 
    fmiStatus fmiFlag;               // return code of the fmu functions
    fmiReal t0 = 0;                  // start time
    fmiReal h = 1e-8;
    fmiReal tEnd = 4.0;
    fmiBoolean toleranceControlled = fmiFalse;
    int nSteps = 0;
    int nTimeEvents = 0;
    int nStepEvents = 0;
    int nStateEvents = 0;
    FILE* file;

    // instantiate the fmu
    id="MBSim"; //known from xml
    guid="TEST"; //known from xml
    callbacks.logger = fmuLogger;
    callbacks.allocateMemory = alloc;
    callbacks.freeMemory = free;
    wait();

    c = mbsim_fmiInstantiateModel(id, guid, callbacks, loggingOn);
    if (!c) return error("could not instantiate model");
    wait();

    // allocate memory 
    nx = 4; //known from xml getNumberOfStates(md);
    nz = 0; //known from xml getNumberOfEventIndicators(md);
    x    = (fmiReal *) calloc(nx, sizeof(fmiReal));
    xdot = (fmiReal *) calloc(nx, sizeof(fmiReal));
    if (nz>0) {
        z    =  (fmiReal *) calloc(nz, sizeof(fmiReal));
        prez =  (fmiReal *) calloc(nz, sizeof(fmiReal));
    }
    if (!x || !xdot || (nz>0 && (!z || !prez))) return error("out of memory");
    
    // open result file
    if (!(file=fopen(RESULT_FILE, "w"))) {
        printf("could not write %s because:\n", RESULT_FILE);
        printf("    %s\n", strerror(errno));
        return 0; // failure
    }
    
    // set the start time and initialize
    time = t0;
    fmiFlag =  mbsim_fmiSetTime(c, t0);
    if (fmiFlag > fmiWarning) return error("could not set time");
    fmiFlag =  mbsim_fmiInitialize(c, toleranceControlled, t0, &eventInfo);
    if (fmiFlag > fmiWarning)  return error("could not initialize model");
    if (eventInfo.terminateSimulation) {
        printf("model requested termination at init");
        tEnd = time;
    }

    // enter the simulation loop
    while (time < tEnd) {
      wait();
     // get current state and derivatives
     fmiFlag = mbsim_fmiGetContinuousStates(c, x, nx);
     if (fmiFlag > fmiWarning) return error("could not retrieve states");
     fmiFlag = mbsim_fmiGetDerivatives(c, xdot, nx);
     if (fmiFlag > fmiWarning) return error("could not retrieve derivatives");

     // advance time
     tPre = time;
     time = fmin(time+h, tEnd);
     timeEvent = eventInfo.upcomingTimeEvent && eventInfo.nextEventTime < time;     
     if (timeEvent) time = eventInfo.nextEventTime;
     dt = time - tPre; 
     fmiFlag = mbsim_fmiSetTime(c, time);
     if (fmiFlag > fmiWarning) error("could not set time");

     // perform one step

     for (i=0; i<nx; i++) x[i] += dt*xdot[i]; // forward Euler method
     fmiFlag = mbsim_fmiSetContinuousStates(c, x, nx);
     if (fmiFlag > fmiWarning) return error("could not set states");
     if (loggingOn) printf("Step %d to t=%.16g\n", nSteps, time);
    
     // Check for step event, e.g. dynamic state selection
     fmiFlag = mbsim_fmiCompletedIntegratorStep(c, &stepEvent);
     if (fmiFlag > fmiWarning) return error("could not complete intgrator step");

     // Check for state event
     for (i=0; i<nz; i++) prez[i] = z[i]; 
     fmiFlag = mbsim_fmiGetEventIndicators(c, z, nz);
     if (fmiFlag > fmiWarning) return error("could not retrieve event indicators");
     stateEvent = 0;
     for (i=0; i<nz; i++) 
         stateEvent = stateEvent || (prez[i] * z[i] < 0);  
     
     // handle events
     if (timeEvent || stateEvent || stepEvent) {
        
        if (timeEvent) {
            nTimeEvents++;
            if (loggingOn) printf("time event at t=%.16g\n", time);
        }
        if (stateEvent) {
            nStateEvents++;
            if (loggingOn) for (i=0; i<nz; i++)
                printf("state event %s z[%d] at t=%.16g\n", 
                        (prez[i]>0 && z[i]<0) ? "-\\-" : "-/-", i, time);
        }
        if (stepEvent) {
            nStepEvents++;
            if (loggingOn) printf("step event at t=%.16g\n", time);
        }

        // event iteration in one step, ignoring intermediate results
        fmiFlag = mbsim_fmiEventUpdate(c, fmiFalse, &eventInfo);
        if (fmiFlag > fmiWarning) return error("could not perform event update");
        
        // terminate simulation, if requested by the model
        if (eventInfo.terminateSimulation) {
            printf("model requested termination at t=%.16g\n", time);
            break; // success
        }

        // check for change of value of states
        if (eventInfo.stateValuesChanged && loggingOn) {
            printf("state values changed at t=%.16g\n", time);
        }
        
        // check for selection of new state variables
        if (eventInfo.stateValueReferencesChanged && loggingOn) {
            printf("new state variables selected at t=%.16g\n", time);
        }
       
     } // if event
     nSteps++;
  } // while  

  // cleanup
  if(! eventInfo.terminateSimulation) mbsim_fmiTerminate(c);
  mbsim_fmiFreeModelInstance(c);
  fclose(file);
  if (x!=NULL) free(x);
  if (xdot!= NULL) free(xdot);
  if (z!= NULL) free(z);
  if (prez!= NULL) free(prez);
  
  
  
  

//   // build single modules
//   System *sys = new System("TS");
// 
//   // add modules to overall dynamical system
//   sys->initialize();
// 
//   // integration
//   TimeSteppingIntegrator integrator;
//   integrator.setStepSize(1e-4);
//   integrator.setEndTime(4.0);
//   integrator.setPlotStepSize(1e-3);
// 
//   integrator.integrate(*sys);
//   cout << "finished"<<endl;
// 
//   delete sys;

  return 0;
}


#define MAX_MSG_SIZE 1000
void fmuLogger(fmiComponent c, fmiString instanceName, fmiStatus status,
               fmiString category, fmiString message, ...) {
    char msg[MAX_MSG_SIZE];
//     char* copy;
    va_list argp;

    // replace C format strings
	  va_start(argp, message);
    vsprintf(msg, message, argp);

    // replace e.g. ## and #r12#  
//     copy = strdup(msg);
//     replaceRefsInMessage(copy, msg, MAX_MSG_SIZE, &fmu);
//     free(copy);
    
    // print the final message
    if (!instanceName) instanceName = "?";
    if (!category) category = "?";
    printf("%s %s (%s): %s\n", fmiStatusToString(status), instanceName, category, msg);
}

int error(const char* message){
    printf("%s\n", message);
    return 0;
}

const char* fmiStatusToString(fmiStatus status){
    switch (status){
        case fmiOK:      return "ok";
        case fmiWarning: return "warning";
        case fmiDiscard: return "discard";
        case fmiError:   return "error";
        case fmiFatal:   return "fatal";
#ifdef FMI_COSIMULATION
        case fmiPending: return "fmiPending";
#endif
        default:         return "?";
    }
}
