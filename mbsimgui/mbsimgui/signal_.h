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
      Signal(const QString &str="");
      virtual MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIMCONTROL; }
      ~Signal(); 
  };

  class PIDController : public Signal {
    public:
    PIDController(const QString &str="");
    QString getType() const { return "PIDController"; }
    ElementPropertyDialog* createPropertyDialog() {return new PIDControllerPropertyDialog(this);}
  };

  class UnarySignalOperation : public Signal {
    public:
    UnarySignalOperation(const QString &str="");
    QString getType() const { return "UnarySignalOperation"; }
    ElementPropertyDialog* createPropertyDialog() {return new UnarySignalOperationPropertyDialog(this);}
  };

  class BinarySignalOperation : public Signal {
    public:
    BinarySignalOperation(const QString &str="");
    QString getType() const { return "BinarySignalOperation"; }
    ElementPropertyDialog* createPropertyDialog() {return new BinarySignalOperationPropertyDialog(this);}
  };

  class ExternSignalSource : public Signal {
    public:
      ExternSignalSource(const QString &str="");
      QString getType() const { return "ExternSignalSource"; }
      ElementPropertyDialog* createPropertyDialog() {return new ExternSignalSourcePropertyDialog(this);}
  };

  class ExternSignalSink : public Signal {
    public:
      ExternSignalSink(const QString &str="");
      QString getType() const { return "ExternSignalSink"; }
      ElementPropertyDialog* createPropertyDialog() {return new ExternSignalSinkPropertyDialog(this);}
  };

}

#endif
