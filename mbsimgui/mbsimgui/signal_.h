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
  };

  class Multiplexer : public Signal {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"Multiplexer"; }
      QString getType() const override { return "Multiplexer"; }
      PropertyDialog* createPropertyDialog() override { return new MultiplexerPropertyDialog(this); }
  };

  class Demultiplexer : public Signal {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"Demultiplexer"; }
      QString getType() const override { return "Demultiplexer"; }
      PropertyDialog* createPropertyDialog() override { return new DemultiplexerPropertyDialog(this); }
  };

  class LinearTransferSystem : public Signal {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"LinearTransferSystem"; }
      QString getType() const override { return "Linear transfer system"; }
      PropertyDialog* createPropertyDialog() override { return new LinearTransferSystemPropertyDialog(this); }
  };

  class NonlinearTransferSystem : public Signal {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"NonlinearTransferSystem"; }
      QString getType() const override { return "Nonlinear transfer system"; }
      PropertyDialog* createPropertyDialog() override { return new NonlinearTransferSystemPropertyDialog(this); }
  };

  class SignalOperation : public Signal {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"SignalOperation"; }
      QString getType() const override { return "Signal operation"; }
      PropertyDialog* createPropertyDialog() override { return new SignalOperationPropertyDialog(this); }
  };

  class ExternSignalSource : public Signal {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"ExternSignalSource"; }
      QString getType() const override { return "Extern signal source"; }
      PropertyDialog* createPropertyDialog() override { return new ExternSignalSourcePropertyDialog(this); }
  };

  class ExternSignalSink : public Signal {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"ExternSignalSink"; }
      QString getType() const override { return "Extern signal sink"; }
      PropertyDialog* createPropertyDialog() override { return new ExternSignalSinkPropertyDialog(this); }
  };

  class Switch : public Signal {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"Switch"; }
      QString getType() const override { return "Switch"; }
      PropertyDialog* createPropertyDialog() override { return new SwitchPropertyDialog(this); }
  };

  class Duration : public Signal {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"Duration"; }
      QString getType() const override { return "Duration"; }
      PropertyDialog* createPropertyDialog() override { return new DurationPropertyDialog(this); }
  };

  class StateMachine : public Signal {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"StateMachine"; }
      QString getType() const override { return "StateMachine"; }
      PropertyDialog* createPropertyDialog() override { return new StateMachinePropertyDialog(this); }
  };

}

#endif
