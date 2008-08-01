/* Copyright (C) 2006 Mathias Bachmayer
 *
 * Institute of Applied Mechanics
 * Technical University of Munich

 *
 * Contact:
 *   bachmayer@amm.mw.tum.de
 *
 */ 

#ifndef _TRANSFERSYS_
#define _TRANSFERSYS_

#include "spsys.h"
#include "port.h"
#include "polynome.h"

#ifdef MBSIMXML
#include <xercesc/util/XercesDefs.hpp>
namespace XERCES_CPP_NAMESPACE { class DOMNode; }
#endif


class TransferSys : public SPSys {
  protected:
    Mat A,B,C,D;// Sysmatrizen   
    
    //OutputSignal *TFOutputSignal; 

    Vec DFactor; 
    double R1,R2,c; // Vars fuer interne Differenzierer Bandbreite

    Vec dz;
    double Tt;
    Vec Dd;
    bool ABCDdefined;
    Vec (TransferSys::*OutForm)(double); // Zeiger zum Umschalten bzgl der Ausgangsfunktion

    // Anfang - Deklaration möglicher Systemausgänge 
    Vec OutC(double t);
    Vec OutD(double t);
    Vec OutCD(double t);
    Vec OutCDD(double t);
    // Ende - Deklaration möglicher Systemausgänge
    
  public:   
    TransferSys(const string& name);
    void updatedx(double t, double dt);
    void updatexd(double t);
    void updateStage1(double t);
    void initPlotFiles();
    void plot(double t,double dt);
    void showABCD();
    
   
    //UserFunction* SigU(){return TFOutputSignal;} 
    void setPID(double P_, double I_, double D_);
    void setABCD(Mat A_,Mat B_,Mat C_,Mat D_);
    void setDECoeffs(Vec Numerator,Vec Denominator);
    void setPolesnZerosnGain(Vec Poles,Vec Zeros,double Gain);
    void setPIDKausal(double P_, double I_, double D_);
    void setBandwidth(double Hz_fg);
    void setIntegrator(double OutputGain);
    void setI2(double OutputGain);
    void setPT1(double P, double T);
    void setGain(double P);
#ifdef MBSIMXML
    XERCES_CPP_NAMESPACE::DOMNode *DOMInput(XERCES_CPP_NAMESPACE::DOMNode *n, MultiBodySystem *parentmbs);
#endif  
};

#endif
