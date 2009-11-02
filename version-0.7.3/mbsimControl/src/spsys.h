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
 *   mbachmayer@users.berlios.de
 *
 */ 
#ifndef _SPSYS_H_
#define _SPSYS_H_

#include <string>
#include "port.h"
#include "signalspsys.h"
#include "data_interface_base.h"
#include "extra_dynamic_interface.h"
#include <vector>

using namespace MBSim;

class SPSys : public ExtraDynamicInterface {

  protected:
    SignalSPSys *Signal;
    double talt; // Variable um letzten Update Zeitpunkt zu merken  
    string modus; //String Variable, fuer Betriebsarteintrag in Plot File des Objekts 
    // Container & Vars fuer Inputfunktionen Anfang
    vector<DataInterfaceBase*> SigInputs;
    vector<Port*> PositionInputs;
    vector<Port*> VInputs;
    Vec kTF,kPos,kVin;
    vector<int> xyzVin;
    vector<int> xyzPos;	
    int NumberofInputs;
    int DGL_INPUT_DIMENSION;
    Vec (SPSys::*Uin)(double); //Zeiger auf Inputfunktionen
    virtual Vec SingleUF(double t);
    virtual Vec SinglePosition(double t);
    virtual Vec SingleVelocity(double t);
    virtual Vec MultiInputs_SI(double t); // SI = Single Input
    virtual Vec MultiInputs_MI(double t); // MI = Many Inputs
    Vec InputDummie(double t);
    Vec NoInput(double t);
    bool Single_Input; // Bool entscheidet ob MultiInputs_SI oder MultiInputs_MI verwendet wird
 
    // Container & Vars fuer Inputfunktionen Ende
    // Test Funktionen
    bool Testgesetzt;
    Vec Step(double t);
    Vec Sine(double t);
    Vec RampDXDT(double t);
    Vec Parabel(double t);
    Vec Puls(double t);
    // Ende Test Funktionsgeneratoren
    double Hz;
    double dxdt;

  public:

    SPSys(const string &name);
    void setInSignalnWeight(SPSys *In_,double wichtung);
    void setInSignalnWeight(DataInterfaceBase *In_,double wichtung);
    void setInPositionnWeight(Port *Inport,char XYZ,double wichtung);
    void setInVelocitynWeight(Port *Inport,char XYZ,double wichtung);
    void setNoInput();
    
    virtual void setSumInputs();	

    void TestStepMode();
    void TestSineMode(double fHertz);
    void TestRampDXDT(double DXDT);
    void TestParabel();
    void TestPulsMode();

    Vec operator()(double Zeit);
    SignalSPSys* SigOut(){return Signal;} 
};


#endif
