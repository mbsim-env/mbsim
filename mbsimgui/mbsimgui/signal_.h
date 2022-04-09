/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2013 Martin Förg

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

#ifndef _SIGNAL__H_
#define _SIGNAL__H_

#include "link_.h"

namespace MBSimGUI {

  class Signal : public Link {
    MBSIMGUI_OBJECTFACTORY_CLASS(Signal, Link, MBSIMCONTROL%"Signal", "Signal");
  };

  class Multiplexer : public Signal {
    MBSIMGUI_OBJECTFACTORY_CLASS(Multiplexer, Signal, MBSIMCONTROL%"Multiplexer", "Multiplexer");
    public:
      PropertyDialog* createPropertyDialog() override { return new MultiplexerPropertyDialog(this); }
  };

  class Demultiplexer : public Signal {
    MBSIMGUI_OBJECTFACTORY_CLASS(Demultiplexer, Signal, MBSIMCONTROL%"Demultiplexer", "Demultiplexer");
    public:
      PropertyDialog* createPropertyDialog() override { return new DemultiplexerPropertyDialog(this); }
  };

  class LinearTransferSystem : public Signal {
    MBSIMGUI_OBJECTFACTORY_CLASS(LinearTransferSystem, Signal, MBSIMCONTROL%"LinearTransferSystem", "Linear transfer system");
    public:
      PropertyDialog* createPropertyDialog() override { return new LinearTransferSystemPropertyDialog(this); }
  };

  class NonlinearTransferSystem : public Signal {
    MBSIMGUI_OBJECTFACTORY_CLASS(NonlinearTransferSystem, Signal, MBSIMCONTROL%"NonlinearTransferSystem", "Nonlinear transfer system");
    public:
      PropertyDialog* createPropertyDialog() override { return new NonlinearTransferSystemPropertyDialog(this); }
  };

  class SignalOperation : public Signal {
    MBSIMGUI_OBJECTFACTORY_CLASS(SignalOperation, Signal, MBSIMCONTROL%"SignalOperation", "Signal operation");
    public:
      PropertyDialog* createPropertyDialog() override { return new SignalOperationPropertyDialog(this); }
  };

  class ExternSignalSource : public Signal {
    MBSIMGUI_OBJECTFACTORY_CLASS(ExternSignalSource, Signal, MBSIMCONTROL%"ExternSignalSource", "Extern signal source");
    public:
      PropertyDialog* createPropertyDialog() override { return new ExternSignalSourcePropertyDialog(this); }
  };

  class ExternSignalSink : public Signal {
    MBSIMGUI_OBJECTFACTORY_CLASS(ExternSignalSink, Signal, MBSIMCONTROL%"ExternSignalSink", "Extern signal sink");
    public:
      PropertyDialog* createPropertyDialog() override { return new ExternSignalSinkPropertyDialog(this); }
  };

  class Switch : public Signal {
    MBSIMGUI_OBJECTFACTORY_CLASS(Switch, Signal, MBSIMCONTROL%"Switch", "Switch");
    public:
      PropertyDialog* createPropertyDialog() override { return new SwitchPropertyDialog(this); }
  };

  class Duration : public Signal {
    MBSIMGUI_OBJECTFACTORY_CLASS(Duration, Signal, MBSIMCONTROL%"Duration", "Duration");
    public:
      PropertyDialog* createPropertyDialog() override { return new DurationPropertyDialog(this); }
  };

  class StateMachine : public Signal {
    MBSIMGUI_OBJECTFACTORY_CLASS(StateMachine, Signal, MBSIMCONTROL%"StateMachine", "State machine");
    public:
      PropertyDialog* createPropertyDialog() override { return new StateMachinePropertyDialog(this); }
  };

}

#endif
