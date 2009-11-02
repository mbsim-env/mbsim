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
#ifndef _SPSYS_H_
#define _SPSYS_H_

#include "mbsim/frame.h"
#include "mbsimControl/signalspsys.h"
#include "mbsim/data_interface_base.h"
#include "mbsim/extra_dynamic.h"
#include <vector>
#include <string>

using namespace MBSim;

class SPSys : public MBSim::ExtraDynamic {
  public:
    SPSys(const std::string &name);
    void setInSignalnWeight(SPSys *In_,double wichtung);
    void setInSignalnWeight(DataInterfaceBase *In_,double wichtung);
    void setInPositionnWeight(Frame *Inframe,char XYZ,double wichtung);
    void setInVelocitynWeight(Frame *Inframe,char XYZ,double wichtung);
    void setNoInput();

    virtual void setSumInputs();	

    void TestStepMode();
    void TestSineMode(double fHertz);
    void TestRampDXDT(double DXDT);
    void TestParabel();
    void TestPulsMode();

    fmatvec::Vec operator()(double Zeit);
    SignalSPSys* SigOut(){return Signal;} 

  protected:
    SignalSPSys *Signal;
    double talt; // Variable um letzten Update Zeitpunkt zu merken  
    std::string modus; //String Variable, fuer Betriebsarteintrag in Plot File des Objekts 
    // Container & Vars fuer Inputfunktionen Anfang
    std::vector<DataInterfaceBase*> SigInputs;
    std::vector<Frame*> PositionInputs;
    std::vector<Frame*> VInputs;
    fmatvec::Vec kTF,kPos,kVin;
    std::vector<int> xyzVin;
    std::vector<int> xyzPos;	
    int NumberofInputs;
    int DGL_INPUT_DIMENSION;
    fmatvec::Vec (SPSys::*Uin)(double); //Zeiger auf Inputfunktionen
    virtual fmatvec::Vec SingleUF(double t);
    virtual fmatvec::Vec SinglePosition(double t);
    virtual fmatvec::Vec SingleVelocity(double t);
    virtual fmatvec::Vec MultiInputs_SI(double t); // SI = Single Input
    virtual fmatvec::Vec MultiInputs_MI(double t); // MI = Many Inputs
    fmatvec::Vec InputDummie(double t);
    fmatvec::Vec NoInput(double t);
    bool Single_Input; // Bool entscheidet ob MultiInputs_SI oder MultiInputs_MI verwendet wird

    // Container & Vars fuer Inputfunktionen Ende
    // Test Funktionen
    bool Testgesetzt;
    fmatvec::Vec Step(double t);
    fmatvec::Vec Sine(double t);
    fmatvec::Vec RampDXDT(double t);
    fmatvec::Vec Parabel(double t);
    fmatvec::Vec Puls(double t);
    // Ende Test Funktionsgeneratoren
    double Hz;
    double dxdt;
};

#endif

