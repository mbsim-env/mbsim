/* Copyright (C) 2006 Mathias Bachmayer
 *
 * Institute of Applied Mechanics
 * Technical University of Munich

 *
 * Contact:
 *   bachmayer@amm.mw.tum.de
 *
 */ 
#ifndef _NLTRANSFERSYS_
#define _NLTRANSFERSYS_

#include "spsys.h"
#include "port.h"
#include "polynome.h"

class NLTransferSys : public SPSys {
  protected:
    //OutputSignal *TFOutputSignal; 
    Vec xNull;
    Vec (NLTransferSys::*OutForm)(double,Vec); // Zeiger zum Umschalten bzgl der Ausgangsfunktion
    Vec Saturation(double t, Vec U);
    virtual Vec DE(double t,  Vec U);
    virtual Vec SystemOutput(double t,Vec U);
    double MaxLimit,MinLimit;
  public:   
    NLTransferSys(const string& name);
    void updatedx(double t, double dt);
    void updatexd(double t);
    void updateStage1(double t);
    void initPlotFiles();
    void plot(double t,double dt);
    
    void setMinMaxOut(double MinOut,double MaxOut);
    void activateDynamics();
    void setxNull(Vec xNull_){xNull=xNull_;}
    
    // UserFunction* SigU(){return TFOutputSignal;} 
    
};

class Multiplier :public NLTransferSys{

    protected:
	Vec SystemOutput(double t, Vec U);
    public:
	Multiplier(const string& name);
	void setSumInputs(){cout<<"Sum Inputs not allowed for a Multiplier!"<<endl;}
};

#endif
