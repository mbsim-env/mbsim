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

#ifndef _PROPERTY_DIALOG_H_
#define _PROPERTY_DIALOG_H_

#include <QScrollArea>
#include <QTabWidget>
#include <QDialog>
#include <map>

class QVBoxLayout;
class QDialogButtonBox;
class QAbstractButton;

class PropertyDialog : public QDialog {
  Q_OBJECT

  public:
    PropertyDialog(QWidget * parent = 0, Qt::WindowFlags f = 0);
    ~PropertyDialog();
    void setParentObject(QObject *obj);
    void addToTab(const QString &name, QWidget* widget_);
    void addTab(const QString &name, int i=-1);
    void addStretch();
    void updateWidget();
  protected:
    std::map<QString,QVBoxLayout*> layout;
    std::vector<QWidget*> widget;
    QTabWidget *tabWidget;
    QDialogButtonBox *buttonBox;
    QPushButton *buttonResize;
  public slots:
    void clicked(QAbstractButton *button);
    virtual void toWidget() {}
    virtual void fromWidget() {}
  signals:
    void apply();
};

#endif
