/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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

#ifndef _PROPERTY_DIALOG_H_
#define _PROPERTY_DIALOG_H_

#include <QScrollArea>
#include <QTabWidget>
#include <QDialog>
#include <map>
#include <xercesc/util/XercesDefs.hpp>
#include <mbxmlutilshelper/dom.h>
#include "extended_widgets.h"
#include "mbxmlutils/eval.h"
#include <boost/tti/has_member_function.hpp>

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

class QVBoxLayout;
class QDialogButtonBox;
class QAbstractButton;

BOOST_TTI_HAS_MEMBER_FUNCTION(getStretchHint);

namespace MBSimGUI {

  class EmbedItemData;
  class ExtWidget;

  // this class is duplicted in the scripting language for the evaluator -> keep it simple and synchronized
  class BasicPropertyDialog : public QDialog {
    Q_OBJECT

    public:
      BasicPropertyDialog();
    protected:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
  };

  class PropertyDialog : public BasicPropertyDialog {
    Q_OBJECT

    public:
      PropertyDialog(const QString &title);
      void setParentObject(QObject *obj);

      template<class W>
      void addToTab(const QString &name, W* widget_) {
        int stretchHint=0;
        if constexpr (has_member_function_getStretchHint<int(W::*)() const>::value)
          stretchHint=widget_->getStretchHint();
        int idx = layout[name]->count()-1;
        layout[name]->insertWidget(idx,widget_, stretchHint);

        if constexpr (has_member_function_getStretchHint<int(W::*)() const>::value)
        {
          connect(widget_,&Widget::widgetChanged,this,[this, name, idx, widget_](){
            layout[name]->setStretch(idx, widget_->getStretchHint());
          });
        }
      }

      void addTab(const QString &name, int i=-1);
      void setCancel(bool on);
      bool getCancel() const;
      virtual void toWidget() { }
      virtual void fromWidget() { }
    protected:
      void showEvent(QShowEvent *event) override;
      void hideEvent(QHideEvent *event) override;
      void closeEvent(QCloseEvent *event) override;
      void clicked(QAbstractButton *button);
      virtual void updateWidget() { }
      std::map<QString,QVBoxLayout*> layout;
      QTabWidget *tabWidget;
      QDialogButtonBox *buttonBox;
      QPushButton *buttonResize;
    signals:
      void apply();
      void showXMLHelp();
  };

  class EmbedItemPropertyDialog : public PropertyDialog {

    public:
      EmbedItemPropertyDialog(const QString &title, EmbedItemData *item_);
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) { return nullptr; }
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) { return nullptr; }
      EmbedItemData* getItem() const { return item; }
      void toWidget() override;
      void fromWidget() override;
    protected:
      EmbedItemData *item;
    private:
      MBXMLUtils::NewParamLevel npl;
  };

  class UnknownItemPropertyDialog : public EmbedItemPropertyDialog {

    public:
      UnknownItemPropertyDialog(EmbedItemData *item);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *editor;
  };

}

#endif
