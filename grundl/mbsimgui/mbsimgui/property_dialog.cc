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

#include <config.h>
#include "property_dialog.h"
#include "widget.h"
#include <QGridLayout>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QStyle>

#include <iostream>
using namespace std;

PropertyDialog::PropertyDialog(QWidget *parent, Qt::WindowFlags f) : QDialog(parent,f) {

  QGridLayout *layout = new QGridLayout;
  setLayout(layout);
  tabWidget = new QTabWidget(this);
  layout->addWidget(tabWidget,0,0,1,2);
  buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Apply | QDialogButtonBox::Cancel);
  connect(buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(clicked(QAbstractButton*)));
  layout->addWidget(buttonBox,1,1);
  //buttonResize = new QPushButton(Utils::QIconCached(QString::fromStdString(MBXMLUtils::getInstallPath())+"/share/mbsimgui/icons/resize.svg"), "Resize");
  buttonResize = new QPushButton(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DialogResetButton)), "Resize");
  layout->addWidget(buttonResize,1,0);
  setWindowTitle(QString("Properties"));
}

PropertyDialog::~PropertyDialog() {
  cout << "delete" << endl;
}

void PropertyDialog::clicked(QAbstractButton *button) {
  if(button == buttonBox->button(QDialogButtonBox::Ok)) {
    fromWidget();
    accept();
  }
  else if(button == buttonBox->button(QDialogButtonBox::Apply)) {
    fromWidget();
    emit apply();
  }
  else if(button == buttonBox->button(QDialogButtonBox::Cancel))
    reject();
}

void PropertyDialog::addToTab(const QString &name, QWidget* widget_) {
  layout[name]->addWidget(widget_);
  widget.push_back(widget_);
}

void PropertyDialog::addStretch() {
  for ( std::map<QString,QVBoxLayout*>::iterator it=layout.begin() ; it != layout.end(); it++ )
    (*it).second->addStretch(1);
}

void PropertyDialog::updateWidget() {
  for(unsigned int i=0; i<widget.size(); i++)
    dynamic_cast<WidgetInterface*>(widget[i])->updateWidget();
}

void PropertyDialog::addTab(const QString &name, int i) {  
  QScrollArea *tab = new QScrollArea;
  tab->setWidgetResizable(true);

  QWidget *box = new QWidget;
  QVBoxLayout *layout_ = new QVBoxLayout;
  box->setLayout(layout_);
  layout[name] = layout_;

  tab->setWidget(box);
  if(i==-1)
    tabWidget->addTab(tab, name);
  else 
    tabWidget->insertTab(i,tab,name);
}
