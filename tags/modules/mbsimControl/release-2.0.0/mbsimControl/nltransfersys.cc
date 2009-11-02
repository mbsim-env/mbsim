/* Copyright (C) 2006  Mathias Bachmayer

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
 *
 * Contact:
 *   mbachmayer@gmx.de
 *
 */ 
#include <config.h>
#include "mbsim/dynamic_system.h"
#include "mbsimControl/nltransfersys.h"

using namespace std;
using namespace fmatvec;

NLTransferSys::NLTransferSys(const string& name) : SPSys(name) {
  OutForm=&NLTransferSys::SystemOutput;
  xSize=0;
  Single_Input=false;
}

void NLTransferSys::init(InitStage stage) {
  if (stage==MBSim::plot) {
    updatePlotFeatures(parent);
    if(getPlotFeature(plotRecursive)==enabled) {
      plotColumns.push_back("OutputSignal");
      plotColumns.push_back("Inputsignal");
      SPSys::init(stage);
    }
  }
  else
    SPSys::init(stage);
}

void NLTransferSys::updatedx(double t, double dt){
  xd=DE(t,(this->*Uin)(t))*dt;
}

void NLTransferSys::updatexd(double t){
  xd=DE(t,(this->*Uin)(t));
}

void NLTransferSys::updateStage1(double t){
  y=(this->*OutForm)(t,(this->*Uin)(t));
  //TFOutputSignal->setSignal(y);
}

//DYNAMIK DEFINITIONEN ANFANG ***************************************************

Vec NLTransferSys::Saturation(double t, Vec U){
  Vec Input;
  Input=SystemOutput(t,U);    
  for (int k=0;k<Input.rows();k++){
    if (Input(k)>MaxLimit) {Input(k)=MaxLimit;}
    if (Input(k)<MinLimit) {Input(k)=MinLimit;}
  }
  return Input;
}

Vec NLTransferSys::SystemOutput(double t,Vec U){
  Vec Y(1);
  Y(0)=U(0);
  return Y;
}

Vec NLTransferSys::DE(double t, Vec U){
  Vec Dot(xSize);
  return Dot;
}

void NLTransferSys::setMinMaxOut(double MinOut, double MaxOut){
  modus="Saturation Mode";
  if (MinOut>MaxOut){cout<<"ERROR in setMinMaxOut Method of Object "<<name<<"! MinOut > MaxOut !"<<endl; throw 55;}
  MaxLimit=MaxOut;
  MinLimit=MinOut;
  OutForm=&NLTransferSys::Saturation;
}

void NLTransferSys::activateDynamics(){
  Vec A(100);
  x=Vec(100);
  Vec Zwerg;
  Zwerg=DE(1,A);
  xSize=Zwerg.rows();
  x.resize()=Vec(xSize);
  if (xNull.size()>0)
  {x=xNull;}
  else 
  {x=Vec(xSize,INIT,0);}
  cout<<"Inner Dynamic of nonlinear System "<<name<<" has "<<xSize<<" States."<<endl;
}

//DYNAMIK Definitionen ENDE-----------------------------------------------------------------------

// Feature Definitionen Ende ---------------


void NLTransferSys::plot(double t, double dt){
  if(getPlotFeature(plotRecursive)==enabled) {
    plotVector.push_back(y(0));
    double eingang;
    eingang=(this->*Uin)(t)(0);
    plotVector.push_back(eingang);

    SPSys::plot(t,dt);
  }
}


