/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2013 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef _SIGNAL__H_
#define _SIGNAL__H_

#include "link_.h"

namespace MBSimGUI {

  class Signal : public Link {
    public:
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMCONTROL; }
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

}

#endif
