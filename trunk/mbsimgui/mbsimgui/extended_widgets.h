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

#ifndef _EXTENDED_WIDGETS_H_
#define _EXTENDED_WIDGETS_H_

#include "xml_widget.h"
#include <QComboBox>
#include <QGroupBox>

class QStackedWidget;
class StringWidget;
class PhysicalStringWidget;
class EvalDialog;
class QVBoxLayout;

class ExtPhysicalVarWidget : public XMLWidget {
  Q_OBJECT

  public:
    ExtPhysicalVarWidget(std::vector<PhysicalStringWidget*> inputWidget);

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    PhysicalStringWidget* getPhysicalStringWidget(int i) {return inputWidget[i];}
    PhysicalStringWidget* getCurrentPhysicalStringWidget() {return inputWidget[inputCombo->currentIndex()];}
    int getNumberOfInputs() const {return inputWidget.size();}
    virtual std::string getValue() const;
    void setValue(const std::string &str);

  protected:
    std::vector<PhysicalStringWidget*> inputWidget;
    QComboBox *inputCombo;
    EvalDialog *evalDialog;
    QStackedWidget *stackedWidget;
    int evalInput;
  protected slots:
    void openEvalDialog();
    void updateInput();
    void changeCurrent(int idx);
  signals:
    void inputDialogChanged(int);
};

class XMLWidgetChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:
    XMLWidgetChoiceWidget(const std::vector<std::string> &name, const std::vector<QWidget*> &widget);
    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual void initialize();
    virtual void update();
  protected slots:
    void changeCurrent(int idx);
  protected:
    QComboBox *choice;
    QStackedWidget *stackedWidget;
};

class ExtXMLWidget : public QGroupBox, public XMLInterface {
  Q_OBJECT

  public:
    ExtXMLWidget(const QString &name, XMLWidget *widget, bool disable=false);
    void setXMLName(const std::string &name, bool flag=true) {xmlName = name; alwaysWriteXMLName=flag;}

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    XMLWidget* getWidget() {return widget;}
    virtual void initialize() {widget->initialize();}
    virtual void update() {widget->update();}
    virtual void resizeVariables() {widget->resizeVariables();}
    bool isActive() const {return (isCheckable() && !isChecked())?0:1;}

  protected:
    XMLWidget *widget;
    std::string xmlName;
    bool alwaysWriteXMLName;
  signals:
    void resize();
};

class XMLWidgetContainer : public XMLWidget {
  public:
    XMLWidgetContainer();

    virtual bool initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void addWidget(QWidget *widget_);

  protected:
    QVBoxLayout *layout;
    std::vector<QWidget*> widget;
};

#endif
