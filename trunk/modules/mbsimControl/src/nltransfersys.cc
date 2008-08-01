/* Copyright (C) 2006 Mathias Bachmayer
 *
 * Institute of Applied Mechanics
 * Technical University of Munich

 *
 * Contact:
 *   bachmayer@amm.mw.tum.de
 *
 */ 
#include <config.h>
#include "nltransfersys.h"


NLTransferSys::NLTransferSys(const string& name) : SPSys(name) {
    OutForm=&NLTransferSys::SystemOutput;
    xSize=0;
    Single_Input=false;
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


//Plot Def Anfang************************************************************
void NLTransferSys::plot(double t, double dt){
  double eingang;
  eingang=(this->*Uin)(t)(0);
  Element::plot(t,dt);
 //Vec Zwerg=y;
  //for (int k=0;k<Zwerg.rows();k++)
  //{
  plotfile<<" "<<y(0);
  //}
  plotfile<<" "<<eingang;

}

void NLTransferSys::initPlotFiles() {
  Element::initPlotFiles();
 //Vec TestVec;
 //Vec Dussl(100);
 // TestVec=(this->*OutForm)(1,Dussl,(this->*Uin)(1));
 // for (int k=0;k<TestVec.rows();k++){
plotfile <<"# "<< plotNr++ << ": OutputSignal " << endl;
 // }
plotfile <<"# "<< plotNr++ << ": Inputsignal        " << endl;
plotfile <<"#  Objekt "<<name<<" befindet sich im "<<modus<<endl;
 // plotfile<<"# System hat "<<TestVec.rows()<<" Ausgangssignale."<<endl;
}
//Plot Defs Ende--------------------------------------------------------------------
//
//

Multiplier::Multiplier(const string& name): NLTransferSys(name){
    Single_Input=false;
    cout<<"You are using Multiplier, it should actually be removed in the near future! Report it, to stop it! " <<endl;
}

Vec Multiplier::SystemOutput(double t, Vec U){
Vec Out(1,INIT,1);
    for(int k=0;k<U.size();k++){
	Out(0)=Out(0)*U(k);
    }
}

