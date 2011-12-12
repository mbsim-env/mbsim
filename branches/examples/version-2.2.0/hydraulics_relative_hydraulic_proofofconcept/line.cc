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

void Line::updateM(double) {
  M+=1*JTJ(J);
}

void Line::updateJacobians(double) {
  if(dependency.size()==0) {
    if(M.size()==1)
      J=Mat(1,1,INIT,1);
    else {
      J=Mat(1,M.size());
      J(0,uInd[0])=1;
    }
  }
  else {
    J=Mat(1,M.size());
    for(size_t i=0; i<dependency.size(); i++) {
      Mat Jdep=((Line*)dependency[i])->getJ();
      J(0,Index(0,Jdep.cols()-1))+=Jdep;
    }
  }
}

void Line::init(InitStage stage) {
  if(stage==MBSim::plot) {
    updatePlotFeatures(parent);

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
