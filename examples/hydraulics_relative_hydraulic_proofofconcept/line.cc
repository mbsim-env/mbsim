#include "line.h"
#include <mbsim/dynamic_system.h>

using namespace MBSim;
using namespace std;
using namespace fmatvec;

Line::Line(string name) : Object(name) {
  setPlotFeature(state,disabled);
}

void Line::updateStateDependentVariables(double) {
  if(dependency.size()==0)
    flowrate=u(0);
  else {
    flowrate=0;
    for(size_t i=0; i<dependency.size(); i++)
      flowrate+=((Line*)dependency[i])->getFlowrate();
  }
}

void Line::calcuSize(int j) {
  if(dependency.size()==0)
    uSize[j]=1;
  else
    uSize[j]=0;
}

void Line::updateM(double, int k) {
  M[k]+=1*JTJ(J);
}

void Line::updateJacobians(double, int k) {
  if(k!=0) return; // How to calcualte the reactive forces for relative hydraulic lines?
                   // Don't know => do nothing => this leads to wrong reactive forces in the whole model
                   // but this does not influence the dynamics.

  if(dependency.size()==0) {
    if(M[k].size()==1)
      J=Mat(1,1,INIT,1);
    else {
      J=Mat(1,M[k].size());
      J(0,uInd[0])=1;
    }
  }
  else {
    J=Mat(1,M[k].size());
    for(size_t i=0; i<dependency.size(); i++) {
      Mat Jdep=((Line*)dependency[i])->getJ();
      J(0,Index(0,Jdep.cols()-1))+=Jdep;
    }
  }
}

void Line::init(InitStage stage) {
  if(stage==MBSim::plot) {
    updatePlotFeatures();

    if(getPlotFeature(plotRecursive)==enabled) {
      plotColumns.push_back("flowrate");

      Object::init(stage);
    }
  }
  else
    Object::init(stage);
}

void Line::plot(double t, double dt) {
  if(getPlotFeature(plotRecursive)==enabled) {
    plotVector.push_back(flowrate);

    Object::plot(t,dt);
  }
}
