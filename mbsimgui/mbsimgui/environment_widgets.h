/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2020 Martin Förg

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

#ifndef _ENVIRONMENT_WIDGETS_H_
#define _ENVIRONMENT_WIDGETS_H_

#include "widget.h"
#include "namespace.h"

namespace MBSimGUI {

  class ExtWidget;

  class EnvironmentWidget : public Widget {

    public:
      EnvironmentWidget() = default;
      virtual MBXMLUtils::FQN getXMLType() const { return MBSIM%"Environment"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) { return element; }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr);
  };

  class MBSimEnvironmentWidget : public EnvironmentWidget {

    public:
      MBSimEnvironmentWidget();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"MBSimEnvironment"; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr);
    private:
      ExtWidget *accelerationOfGravity, *openMBVObject;
  };

  class EnvironmentWidgetFactory : public WidgetFactory {
    public:
      EnvironmentWidgetFactory();
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      Widget* createWidget(int i=0) override;
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

}

#endif
