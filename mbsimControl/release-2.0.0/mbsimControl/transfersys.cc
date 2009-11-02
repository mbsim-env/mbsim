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
#include "mbsimControl/transfersys.h"

using namespace std;
using namespace fmatvec;

TransferSys::TransferSys(const string& name) : SPSys(name) {
  ABCDdefined=false;
  R1=0.002;
  R2=1;
  c=1;
  modus=" ";
  Single_Input=true;
}

void TransferSys::init(InitStage stage) {
  if (stage==MBSim::plot) {
    updatePlotFeatures(parent);
    if(getPlotFeature(plotRecursive)==enabled) {
      plotColumns.push_back("Sigout");
      plotColumns.push_back("Sigin");
      SPSys::init(stage);
    }
  }
  else
    SPSys::init(stage);
}

void TransferSys::updatedx(double t, double dt){
  xd=(A*x+B*(this->*Uin)(t))*dt;
}
void TransferSys::updatexd(double t){
  xd=A*x+B*(this->*Uin)(t);
}

void TransferSys::updateg(double t){
  y=(this->*OutForm)(t);
}
//OUTCD SYS DEFINITIONEN ***************************************************
Vec TransferSys::OutC(double t){
  return C*x;
}
Vec TransferSys::OutD(double t){
  return D*(this->*Uin)(t);
}

Vec TransferSys::OutCD(double t){
  return C*x+D*(this->*Uin)(t);
}
//OUTCDFORMEN ENDE-----------------------------------------------------------------------

// SETABCDAREA ANFANG
void TransferSys::showABCD(){
  cout<<"State space modell of Object \""<<name<<"\":"<<endl;
  cout<<"A Matrix:"<<A<<endl;
  cout<<"B Matrix:"<<B<<endl;
  cout<<"C Matrix:"<<C<<endl;
  cout<<"D Matrix:"<<D<<endl;
  cout<<"D Factor:"<<DFactor<<endl;  
  cout<<"Weightvectors:"<<endl;
  cout<<"ktF"<<kTF<<endl;
  cout<<"kPos"<<kPos<<endl;
  cout<<"kVin"<<kVin<<endl;
  cout<<"Number of systeminputs:"<<NumberofInputs<<endl;
  cout<<"Elements stored in signalinputvector:"<<SigInputs.size()<<endl;
  cout<<"Elements stored in positioninputvector:"<<PositionInputs.size()<<endl;
  cout<<"Elements stored in velocityinputvector:"<<VInputs.size()<<endl;
}

void TransferSys::setABCD(Mat A_,Mat B_,Mat C_,Mat D_){
  modus="ABCD definition mode (setABCD)";
  if (ABCDdefined) 
  {
    cout<<"ERROR!!!! State space matrices ABCD are going to be redefined by setABCD!"<<endl;
    cout<<"Redefinition happens in Object: "<<name<<endl;
    throw 45;
  }

  ABCDdefined=true;

  A=A_;
  B=B_;
  C=C_;
  D=D_;

  xSize=A.rows();
  x=Vec(xSize,INIT,0);
  if (A.rows()!=B.rows()||C.rows()!=D.rows()||B.cols()!=D.cols()||A.cols()!=C.cols())
  { 
    showABCD();
    cout<<"ERROR: In Object with name \""<<name<<"\" is a conflict in Matrix dimensions!!!!! "<<endl;
    cout<<"Please provide Matrices with Dimensions for a standard StateSpace Model!"<<endl;
    throw 4;
  }
  if (A.rows()!=A.cols())
  {
    cout<<"ERROR: A Matrix in Object \""<<name<<"\" must be a square Matrix!"<<endl;
    throw 5; 
  }

  OutForm=&TransferSys::OutCD;
}

void TransferSys::setPID(double P_, double I_, double D_){
  modus+="PID Regler Modus (setPID)";

  //   OutForm=&TransferSys::OutCD;
  if (D_!=0){OutForm=&TransferSys::OutCD;} else {if (I_!=0) {OutForm=&TransferSys::OutCD;} else {OutForm=&TransferSys::OutD;}}


  if (ABCDdefined) {
    cout<<"ERROR!!!! Zustandsraummatrizen ABCD werden von setPID redefiniert!"<<endl;
    cout<<"Redefinition in Objekt: "<<name<<endl;
    throw 34;
  }

  ABCDdefined=true;
  if (D_==0)
  {
    Mat MA(1,1);
    Mat MB(1,1);
    Mat MC(1,1);
    Mat MD(1,1);
    MA(0,0)=0;
    MB(0,0)=1;
    MC(0,0)=I_;
    MD(0,0)=P_;
    A=MA;
    B=MB;
    C=MC;
    D=MD;
  }
  else
  {  

    Mat MA(2,2);
    Mat MB(2,1);
    Mat MC(1,2);
    Mat MD(1,1);
    MA(1,1)=-1/(R1*c);
    MB(0,0)=1;
    MB(1,0)= 1/(R1*c);
    MC(0,0)=I_;
    MC(0,1)=-D_*R2*c/(R1*c);
    MD(0,0)=P_+D_*R2*c/(R1*c);
    A=MA;
    B=MB;
    C=MC;
    D=MD;

  }   
  xSize=A.rows();
  x=Vec(xSize,INIT,0); 
}



void TransferSys::setBandwidth(double Hz_fg)
{
  double omegag;
  if (Hz_fg>0){
    omegag=2*3.141592654*Hz_fg;
    R1=1/(omegag*c);
    R2=sqrt(R1*R1+1/(c*c));
  }
  else
  { 
    cout<<"Error! As internal Derive Bandwith is a positive nun zero frequency required!"<<endl;
    cout<<"Non valid Argument of Method setBandwidth: "<<Hz_fg<<endl;
    cout<<"Message Source: Object "<<name<<endl;
    throw 55;
  }
}
void TransferSys::setIntegrator(double OutputGain){
  modus+="Integrator mode";
  if (ABCDdefined) {
    cout<<"ERROR!!!! State space matrices ABCD are going to be redefined by setIntegrator!"<<endl;cout<<"Redefinition in Objekt: "<<name<<endl;
    throw 34;
  }
  Mat A_(1,1),B_(1,1),C_(1,1),D_(1,1);
  A_(0,0)=0;
  B_(0,0)=1;
  C_(0,0)=OutputGain;

  setABCD(A_,B_,C_,D_);
}

void TransferSys::setI2(double OutputGain){
  modus+="Double integrator mode";
  if (ABCDdefined) {
    cout<<"ERROR!!!! State space matrices ABCD are going to be redefined by setI2!"<<endl;cout<<"Redefinition in Object: "<<name<<endl;
    throw 34;
  }

  Mat A_(2,2),B_(2,1),C_(1,2),D_(1,1);
  A_(0,1)=1;
  B_(1,0)=1;
  C_(0,0)=OutputGain;
  setABCD(A_,B_,C_,D_);
}

void TransferSys::setPT1(double P,double T){
  modus+="PT1 Modus";
  if (ABCDdefined) {
    cout<<"ERROR!!!! State space matrices ABCD are going to be redefined by setPT1!"<<endl;cout<<"Redefinition in Object: "<<name<<endl;
    throw 34;
  }

  Mat A_(1,1), B_(1,1), C_(1,1), D_(1,1);
  A_(0,0)=-1/T;
  B_(0,0)=1;
  C_(0,0)=P/T;
  setABCD(A_,B_,C_,D_);

}

void TransferSys::setGain(double P){
  modus+="Gain Mode";
  if (ABCDdefined) {
    cout<<"ERROR!!!! State space matrices ABCD are going to be redefined by setGain!"<<endl;cout<<"Redefinition in Object: "<<name<<endl;
    throw 34;
  }

  setPID(P,0,0);

}

void TransferSys::plot(double t, double dt) {
  if(getPlotFeature(plotRecursive)==enabled) {
    plotVector.push_back(y(0));
    plotVector.push_back((this->*Uin)(t)(0));

    SPSys::plot(t,dt);
  }
}
