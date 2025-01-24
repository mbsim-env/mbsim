/*
   MBSimGUI - A fronted for MBSim.
   Copyright (C) 2021 MBSim-Env

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
   */

#include <config.h>
#include "signal_.h"
#include "objectfactory.h"

using namespace std;

namespace MBSimGUI {

  MBSIMGUI_REGOBJECTFACTORY(Demultiplexer);
  MBSIMGUI_REGOBJECTFACTORY(ExternSignalSink);
  MBSIMGUI_REGOBJECTFACTORY(ExternSignalSource);
  MBSIMGUI_REGOBJECTFACTORY(LinearTransferSystem);
  MBSIMGUI_REGOBJECTFACTORY(Multiplexer);
  MBSIMGUI_REGOBJECTFACTORY(NonlinearTransferSystem);
  MBSIMGUI_REGOBJECTFACTORY(SignalOperation);
  MBSIMGUI_REGOBJECTFACTORY(Switch);
  MBSIMGUI_REGOBJECTFACTORY(Stop);
  MBSIMGUI_REGOBJECTFACTORY(Duration);
  MBSIMGUI_REGOBJECTFACTORY(StateMachine);

}
