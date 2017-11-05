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

#include "link.h"

namespace MBSimGUI {

  class Signal : public Link {
    public:
      virtual MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIMCONTROL; }
      void addPlotFeatures() { addPlotFeature(MBSIMCONTROL%"signal"); }
  };

  class Multiplexer : public Signal {
    public:
      QString getType() const { return "Multiplexer"; }
      ElementPropertyDialog* createPropertyDialog() {return new MultiplexerPropertyDialog(this);}
  };

  class Demultiplexer : public Signal {
    public:
      QString getType() const { return "Demultiplexer"; }
      ElementPropertyDialog* createPropertyDialog() {return new DemultiplexerPropertyDialog(this);}
  };

  class LinearTransferSystem : public Signal {
    public:
      QString getType() const { return "LinearTransferSystem"; }
      ElementPropertyDialog* createPropertyDialog() {return new LinearTransferSystemPropertyDialog(this);}
  };

  class SignalOperation : public Signal {
    public:
      QString getType() const { return "SignalOperation"; }
      ElementPropertyDialog* createPropertyDialog() {return new SignalOperationPropertyDialog(this);}
  };

  class ExternSignalSource : public Signal {
    public:
      QString getType() const { return "ExternSignalSource"; }
      ElementPropertyDialog* createPropertyDialog() {return new ExternSignalSourcePropertyDialog(this);}
  };

  class ExternSignalSink : public Signal {
    public:
      QString getType() const { return "ExternSignalSink"; }
      ElementPropertyDialog* createPropertyDialog() {return new ExternSignalSinkPropertyDialog(this);}
  };

}

#endif
