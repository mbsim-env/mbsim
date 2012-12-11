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

#include "property_dialog.h"
#include "mainwindow.h"
#include <QGridLayout>
#include <QPushButton>
#include <QMessageBox>

PropertyDialog *PropertyDialog::instance=NULL;

PropertyDialog::PropertyDialog(MainWindow *mw_,
                               QWidget *widget_,
                               boost::function<TiXmlElement* (TiXmlNode*)> writeXMLFile,
                               boost::function<void (TiXmlElement*)> initializeUsingXML_) {
  mw=mw_;
  widget=widget_;
  initializeUsingXML=initializeUsingXML_;

  // save dialog settings
  savedSettings=new TiXmlElement("dummy");
  writeXMLFile(savedSettings);

  // create dialog
  resize(550, 650);
  QGridLayout *layout=new QGridLayout(this);
  setLayout(layout);
  layout->addWidget(widget, 0, 0, 1, 4);
  QPushButton *button;
  button=new QPushButton("Cancel");
  button->setDisabled(true); // MISSING: remove if Cancel works
  connect(button, SIGNAL(clicked(bool)), this, SLOT(cancelPressed()));
  layout->addWidget(button, 1, 0);
  button=new QPushButton("Apply");
  connect(button, SIGNAL(clicked(bool)), this, SLOT(applyPressed()));
  layout->addWidget(button, 1, 2);
  button=new QPushButton("OK");
  connect(button, SIGNAL(clicked(bool)), this, SLOT(okPressed()));
  layout->addWidget(button, 1, 3);

  connect(this, SIGNAL(accepted()), this, SLOT(okPressed()));
  connect(this, SIGNAL(rejected()), this, SLOT(cancelPressed()));
}

PropertyDialog::~PropertyDialog() {
}

void PropertyDialog::create(MainWindow *mw,
                            QWidget *widget,
                            boost::function<TiXmlElement* (TiXmlNode*)> writeXMLFile,
                            boost::function<void (TiXmlElement*)> initializeUsingXML) {
  // singelton
  if(instance) {
    QMessageBox::information(NULL, "Dialog Information",
                             "Only one property dialog can be opened at the same time.");
    return;
  }

  instance=new PropertyDialog(mw, widget, writeXMLFile, initializeUsingXML);
  instance->show();
}

void PropertyDialog::applyPressed() {
  mw->inlineOpenMBV();
}

void PropertyDialog::cancelPressed() {
  // restore dialog settings
  //MISSING not working: initializeUsingXML(savedSettings->FirstChildElement());

  close();
  instance=NULL;
  layout()->removeWidget(widget);
  widget->setParent(NULL);
  delete savedSettings;
  deleteLater();
}

void PropertyDialog::okPressed() {
  blockSignals(true);
  close();
  blockSignals(false);
  instance=NULL;
  layout()->removeWidget(widget);
  widget->setParent(NULL);
  delete savedSettings;
  deleteLater();

  mw->inlineOpenMBV();
}
