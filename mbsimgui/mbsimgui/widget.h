/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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

#ifndef _XML_WIDGETS_H_
#define _XML_WIDGETS_H_

#include <QWidget>
#include <xercesc/util/XercesDefs.hpp>
#include <mbxmlutilshelper/dom.h>

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

namespace MBSimGUI {

  class WidgetInterface {
    public:
      virtual void updateWidget() { }
      virtual void resize_(int m, int n) { }
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) { return nullptr; }
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) { return nullptr; }
  };

  class Widget : public QWidget, public WidgetInterface {
    Q_OBJECT
    public:
      Widget() = default;
    signals:
      void widgetChanged();
    public slots:
      void updateWidget() override { }
  };

  class WidgetFactory {
    public:
      virtual ~WidgetFactory() = default;
      virtual QWidget* createWidget(int i=0) = 0;
      virtual QString getName(int i=0) const { return ""; }
      virtual int getSize() const { return 0; }
      virtual MBXMLUtils::FQN getXMLName(int i=0) const { return ""; }
  };

}

#endif
