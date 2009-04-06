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
#include "mbsimControl/spsys.h"
#include "mbsim/object.h"
#include "mbsim/dynamic_system_solver.h"

using namespace fmatvec;
using namespace std;

SPSys::SPSys(const string &name) : OrderOneDynamics(name) {
  Uin=&SPSys::InputDummie;
  Testgesetzt=false;
  if (Single_Input){DGL_INPUT_DIMENSION=1;} else {DGL_INPUT_DIMENSION=0;} // wird in setIn... Routinen NumberofInputs nachgefuehrt werden. 
  talt=-100;
  modus=" ";
  Signal=new SignalSPSys;
  Signal->setMother(this);
  y=Vec(1,INIT,0.0);
  NumberofInputs=0;

}

Vec SPSys::operator()(double Zeit){
  if (talt!=Zeit){updateg(Zeit);
    talt=Zeit;
  }
  return y;
}

// ANFANG INPUTFUNKTIONEN
Vec SPSys::InputDummie(double t){
  cout<<"ERROR in Definition of Signal Processing Object "<<name<<" ! You have to set at least one System Input!"<<endl;
  throw 55;
}

Vec SPSys::NoInput(double t){
  return Vec(1,INIT,0);    
}


Vec SPSys::SinglePosition(double t){
  Vec D(1);
  D(0)=kPos(0)*(PositionInputs[0]->getPosition()(xyzPos[0]));
  return D;
}
Vec SPSys::SingleVelocity(double t){
  Vec D(1);
  D(0)=kVin(0)*(VInputs[0]->getVelocity()(xyzVin[0]));
  return D;
}
Vec SPSys::SingleUF(double t){
  Vec D;
  D=kTF(0)*(*SigInputs[0])(t);
  return D;
}

Vec SPSys::MultiInputs_MI(double t){
  Vec D(kTF.rows()+kPos.rows()+kVin.rows());
  for (int k=0;k<kTF.rows();k++)
  {	Vec R;
    R=kTF(k)*((*SigInputs[k])(t));
    D(k)=R(0); 
  }
  for (int k=0;k<kPos.rows();k++)
  {
    D(k+kTF.rows())=kPos(k)*(PositionInputs[k]->getPosition()(xyzPos[k]));
  }
  for (int k=0;k<kVin.rows();k++)
  {
    D(k+kTF.rows()+kPos.rows())=kVin(k)*(VInputs[k]->getVelocity()(xyzVin[k]));
  }
  return D;
}

Vec SPSys::MultiInputs_SI(double t){
  Vec D(1);
  for(int k=0;k<kTF.rows();k++)
  {
    D=D+kTF(k)*((*SigInputs[k])(t));
  }
  for(int k=0;k<kPos.rows();k++)
  {
    D(0)=D(0)+kPos(k)*(PositionInputs[k]->getPosition()(xyzPos[k]));
  }
  for(int k=0;k<kVin.rows();k++)
  {
    D(0)=D(0)+kVin(k)*(VInputs[k]->getVelocity()(xyzVin[k]));
  }
  return D;
}
Vec SPSys::Step(double t){
  if (t>1){return Vec(DGL_INPUT_DIMENSION,INIT,1); } else {return Vec(1,INIT,0);}
}
Vec SPSys::Sine(double t){
  return Vec(DGL_INPUT_DIMENSION,INIT,sin(2*3.141592654*Hz*t));
}   
Vec SPSys::RampDXDT(double t){
  return Vec(DGL_INPUT_DIMENSION,INIT,dxdt*t);
}
Vec SPSys::Parabel(double t){
  return Vec(DGL_INPUT_DIMENSION,INIT,t*t);
}
Vec SPSys::Puls(double t){
  double sout=0;
  if (t>1) {sout=1;}
  if (t>2) {sout=0;}
  return Vec(DGL_INPUT_DIMENSION,INIT,sout);
}
//INPUTFORMEN ENDE


void SPSys::setSumInputs(){
  cout<<"SPSys "<<name<<" has been setup for using SumInputs!"<<endl;
  Single_Input=true;
  Uin=&SPSys::MultiInputs_SI;
}


// ENDE INPUTFUNKTIONEN

// ANFANG TESTMODES
void SPSys::TestStepMode(){
  modus="step response testmode";
  Testgesetzt=true;
  DGL_INPUT_DIMENSION=1;
  cout<<"Hint: object "<<name<<" has been set up by user for step response testmode!"<<endl;
  Uin=&SPSys::Step;
}
void SPSys::TestSineMode(double fHertz){
  modus="sine input testmode";
  Testgesetzt=true;
  DGL_INPUT_DIMENSION=1;
  cout<<"Hint: object "<<name<<" is set up for sine input testmode. "<<endl;
  Hz=fHertz;
  Uin=&SPSys::Sine;
}
void SPSys::TestRampDXDT(double mDXDT){
  modus="ramp input mode";
  Testgesetzt=true;
  DGL_INPUT_DIMENSION=1;
  cout<<"Hint: Oject "<<name<<" is set up for ramp input mode"<<endl;
  dxdt=mDXDT;
  Uin=&SPSys::RampDXDT;
}
void SPSys::TestParabel(){
  modus="Parabel input mode";
  Testgesetzt=true;
  DGL_INPUT_DIMENSION=1;
  cout<<"Hint: Object "<<name<<" is set up for parabel input mode"<<endl;
  Uin=&SPSys::Parabel;
}
void SPSys::TestPulsMode(){
  modus="Puls testmode";
  Testgesetzt=true;
  DGL_INPUT_DIMENSION=1;
  cout<<"Hint: Object "<<name<<" is set up for puls input mode"<<endl;
  Uin=&SPSys::Puls;
}
//ENDE TESTMODES
//DEFINE INPUTS ANFANG**************************************************************************************
//
void SPSys::setNoInput(){
  if ((Testgesetzt)||(NumberofInputs>0)){cout<<"Warning - Inputs are set but now overwritten by Method setNoImput"<<endl;}
  Uin=&SPSys::NoInput;
}

void SPSys::setInSignalnWeight(SPSys *In_,double wichtung){
  setInSignalnWeight(In_->SigOut(),wichtung);
}

void SPSys::setInSignalnWeight(DataInterfaceBase *In_,double wichtung){
  if (Testgesetzt){
    cout<<"Hint: Object "<<name<<" is set up for a testmode by user, inputsignals disconnected!"<<endl;
  }
  else
  {
    SigInputs.push_back(In_);

    Vec k_(kTF.rows()+1);
    k_(0,kTF.rows()-1)=kTF;
    k_(kTF.rows())=wichtung;
    kTF.resize()=k_; 

    NumberofInputs++;

    if (NumberofInputs>1) 
    {if (Single_Input){Uin=&SPSys::MultiInputs_SI;}
      else {Uin=&SPSys::MultiInputs_MI;
        DGL_INPUT_DIMENSION=NumberofInputs;}
    } 
    else 
    {Uin=&SPSys::SingleUF;}
  }
}



void SPSys::setInPositionnWeight(Frame *Inport,char XYZ,double wichtung){
  if (Testgesetzt){
    cout<<"Hint: Object "<<name<<" is set up for testmode, input signals disconnected!"<<endl;
  }
  else
  {
    PositionInputs.push_back(Inport);

    switch (XYZ)
    {
      case 'x':xyzPos.push_back(0);break;
      case 'y':xyzPos.push_back(1);break;
      case 'z':xyzPos.push_back(2);break;
      default:cout<<"ERROR in Object "<<name<<" are the letters x, y or z expected but not provided as Positiondirections! "<<endl;
              throw 55;
    }


    Vec k_(kPos.rows()+1);
    k_(0,kPos.rows()-1)=kPos;
    k_(kPos.rows())=wichtung;
    kPos.resize()=k_;

    NumberofInputs++;

    if (NumberofInputs>1) 
    {if (Single_Input){Uin=&SPSys::MultiInputs_SI;} 
      else 
      {Uin=&SPSys::MultiInputs_MI;
        DGL_INPUT_DIMENSION=NumberofInputs;}
    }
    else 
    {Uin=&SPSys::SinglePosition;}
  }
}
void SPSys::setInVelocitynWeight(Frame *Inport, char XYZ, double wichtung)
{
  if (Testgesetzt){
    cout<<"Hint: Object "<<name<<" is set up for testmode, regular input signals are replaced by testinput!"<<endl;
  }
  else
  {

    VInputs.push_back(Inport);

    switch (XYZ)
    {
      case 'x':xyzVin.push_back(0);break;
      case 'y':xyzVin.push_back(1);break;
      case 'z':xyzVin.push_back(2);break;
      default:cout<<"ERROR in Object "<<name<<" are the letters x, y or z expected but not provided as Positiondirections! "<<endl;
              throw 55;
    }

    Vec k_(kVin.rows()+1);
    k_(0,kVin.rows()-1)=kVin;
    k_(kVin.rows())=wichtung;
    kVin.resize()=k_;


    NumberofInputs++;

    if (NumberofInputs>1) 
    {if (Single_Input){Uin=&SPSys::MultiInputs_SI;}
      else {Uin=&SPSys::MultiInputs_MI;
        DGL_INPUT_DIMENSION=NumberofInputs; }
    }
    else 
    {Uin=&SPSys::SingleVelocity;}
  }
}

