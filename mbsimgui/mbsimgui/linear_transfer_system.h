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

#ifndef _LINEAR_TRANSFER_SYSTEM__H_
#define _LINEAR_TRANSFER_SYSTEM__H_

#include "signal_processing_system.h"
#include "widget.h"

namespace MBSimGUI {

  class LinearTransferSystemPropertyDialog : public SignalProcessingSystemPropertyDialog {

    public:
      LinearTransferSystemPropertyDialog(LinearTransferSystem *lts, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *choice;
  };

  class LinearTransferSystem : public SignalProcessingSystem {
    friend class LinearTransferSystemPropertyDialog;
    public:
    LinearTransferSystem(const std::string &str="");
    std::string getType() const { return "LinearTransferSystem"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new LinearTransferSystemPropertyDialog(this);}
    protected:
    ExtProperty choice;
  };

  class LinearTransferSystemWidgetFactory : public WidgetFactory {
    public:
      LinearTransferSystemWidgetFactory();
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
  };

  class LinearTransferSystemPropertyFactory: public PropertyFactory {
    public:
      LinearTransferSystemPropertyFactory();
      Property* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<MBXMLUtils::FQN> name;
  };

}

#endif
