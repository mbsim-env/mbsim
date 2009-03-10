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
#ifndef _NLTRANSFERSYS_
#define _NLTRANSFERSYS_

#include "spsys.h"
#include "coordinate_system.h"

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
    void initPlot(bool top=true);
    void plot(double t,double dt, bool top=true);
    
    void setMinMaxOut(double MinOut,double MaxOut);
    void activateDynamics();
    void setxNull(Vec xNull_){xNull=xNull_;}

    virtual string getType() const {return "NLTransferSys";}
    
    // UserFunction* SigU(){return TFOutputSignal;} 
    
};

#endif
