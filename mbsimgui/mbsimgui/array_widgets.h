/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2025 Martin Förg

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

#ifndef _ARRAY_WIDGETS_H_
#define _ARRAY_WIDGETS_H_

#include "widget.h"
#include <optional>
#include <QLabel>
#include <QComboBox>
#include <QBoxLayout>

class QSpinBox;
class QStackedWidget;
class QTableWidget;
class QPushButton;

namespace MBSimGUI {

  class ArrayWidget : public Widget {

    public:
      ArrayWidget(WidgetFactory *factory_, const QString &name_="Element", int m=1, bool fixedSize_=false, bool n1Disabled=false, bool n2Disabled=false, bool n3Disabled=false, MBXMLUtils::NamespaceURI uri_=MBSIM);
      ~ArrayWidget() override;
      void resize_(int n1, int n2, int n3);
      void resize_(int m, int n) override;
      int getSize1() const;
      int getSize2() const;
      int getSize3() const;
      void updateSize2();
      void updateSize3();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      QString getXMLComment(xercesc::DOMElement *element) override;
      void setXMLComment(const QString &comment, xercesc::DOMNode *element) override;

    protected:
      void addElements(int n=1, bool emitSignals=true);
      void removeElements(int n=1);
      void updateTable();
      void changeCurrent(int currentRow, int currentColumn, int previousRow, int previousColumn);
      void changeSize(int size);
      QStackedWidget *stackedWidget;
      QSpinBox *n1, *n2, *n3;
      QTableWidget *table;
      WidgetFactory *factory;
      QString name;
      bool fixedSize;
      MBXMLUtils::NamespaceURI uri;

    private:
      Widget* getWidgetVirtual(int i) const override;
  };

  class TwoDimensionalArrayWidget : public Widget {

    public:
      TwoDimensionalArrayWidget(WidgetFactory *factory_, const QString &name_="Element", int m=1, int n=1, bool fixedSize_=false, bool n1Disabled=false, bool n2Disabled=false, bool n3Disabled=false, bool n4Disabled=false, int square=false, int symmetric=false, MBXMLUtils::NamespaceURI uri_=MBSIM);
      ~TwoDimensionalArrayWidget() override;
      void resize_(int n1, int n2, int n3, int n4);
      void resize_(int m, int n) override;
      int getSize1() const;
      int getSize2() const;
      int getSize3() const;
      int getSize4() const;
      void updateSize3();
      void updateSize4();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      QString getXMLComment(xercesc::DOMElement *element) override;
      void setXMLComment(const QString &comment, xercesc::DOMNode *element) override;

    protected:
      void addElements(int n=1, bool emitSignals=true);
      void removeElements(int n=1);
      void updateTable();
      void changeCurrent(int currentRow, int currentColumn, int previousRow, int previousColumn);
      void changeSize(int r, int c);
      void forceSymmetrie();
      WidgetFactory *factory;
      QString name;
      int m{0};
      int n{0};
      bool fixedSize, square, symmetric;
      QStackedWidget *stackedWidget;
      QSpinBox *n1, *n2, *n3, *n4;
      QTableWidget *table;
      MBXMLUtils::NamespaceURI uri;

    private:
      Widget* getWidgetVirtual(int i) const override;
  };

}

#endif
