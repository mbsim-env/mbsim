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
#ifndef _SIGNALSPSYS_H_
#define _SIGNALSPSYS_H_

#include "fmatvec.h"
#include "data_interface_base.h"
#include <vector>

using namespace fmatvec;
using namespace MBSim;
/*! 
 *  A controlsignal is the medium required for data transfer where SPSYS
 *  objects are involved 
 * */
class SPSys;

class SignalSPSys : public DataInterfaceBase {
    protected:
      SPSys *Mother;
    public:
        SignalSPSys() {}  // default value 
        ~SignalSPSys() {}
	void setMother(SPSys *Mother_){Mother=Mother_;}
	Vec operator()(double t);
};

           
#endif
