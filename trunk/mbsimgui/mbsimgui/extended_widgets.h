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

#include "widget.h"
#include <QComboBox>
#include <QGroupBox>
#include <QBoxLayout>

class QStackedWidget;
class VariableWidget;
class PhysicalVariableWidget;
class EvalDialog;
class QVBoxLayout;

class ExtPhysicalVarWidget : public Widget {
  Q_OBJECT

  public:
    ExtPhysicalVarWidget(std::vector<PhysicalVariableWidget*> inputWidget, int evalIndex=0);
    ~ExtPhysicalVarWidget();

    PhysicalVariableWidget* getPhysicalVariableWidget(int i) {return inputWidget[i];}
    PhysicalVariableWidget* getCurrentPhysicalVariableWidget() {return inputWidget[inputCombo->currentIndex()];}
    int getNumberOfInputs() const {return inputWidget.size();}
    int getCurrentInput() const {return inputCombo->currentIndex();}
    void setCurrentInput(int i) {inputCombo->setCurrentIndex(i);}
    virtual QString getValue() const;
    void setValue(const QString &str);

  protected:
    std::vector<PhysicalVariableWidget*> inputWidget;
    QComboBox *inputCombo;
    EvalDialog *evalDialog;
    QStackedWidget *stackedWidget;
    int evalInput;
  protected slots:
    void openEvalDialog();
    void changeCurrent(int idx);
  signals:
    void inputDialogChanged(int);
};

class ExtWidget : public QGroupBox, public WidgetInterface {
  Q_OBJECT

  friend class ExtProperty;

  public:
    ExtWidget(const QString &name, Widget *widget, bool deactivatable=false, bool active=false);
    Widget* getWidget() {return widget;}
    virtual void updateWidget() {widget->updateWidget();}
    virtual void resizeVariables() {widget->resizeVariables();}
    bool isActive() const {return (isCheckable() && !isChecked())?0:1;}
    void setActive(bool flag) {if(isCheckable()) setChecked(flag);}
    void setWidgetVisible(bool flag) {if(isCheckable()) widget->setVisible(flag);}

  protected:
    Widget *widget;
  signals:
    void resize_();
};

class ChoiceWidget : public Widget {
  Q_OBJECT

  friend class ChoiceProperty;

  public:
    ChoiceWidget(const std::vector<QWidget*> &widget, const std::vector<QString> &name, QBoxLayout::Direction dir=QBoxLayout::TopToBottom);

    void resize_(int m, int n);
    QWidget* getWidget(int i) const;
    QWidget* getWidget() const;
    QString getName(int i) const;
    QString getName() const;
    void updateWidget();

  protected slots:
    void defineWidget(int);

  protected:
    QComboBox *comboBox;
    QStackedWidget *stackedWidget;

  signals:
    void resize_();
    void widgetChanged();
};

class ContainerWidget : public Widget {

  friend class ContainerProperty;

  public:
    ContainerWidget();

    void addWidget(QWidget *widget_);
    QWidget* getWidget(int i) const {return widget[i];}

  protected:
    QVBoxLayout *layout;
    std::vector<QWidget*> widget;
};

#endif
