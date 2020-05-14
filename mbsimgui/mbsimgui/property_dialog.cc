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
#include "embeditemdata.h"
#include "widget.h"
#include "mainwindow.h"
#include <QGridLayout>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QStyle>
#include <QSettings>

using namespace std;

namespace MBSimGUI {

  extern MainWindow *mw;

  PropertyDialog::PropertyDialog() : QDialog(mw) {

    auto *layout = new QGridLayout;
    setLayout(layout);
    tabWidget = new QTabWidget(this);
    layout->addWidget(tabWidget,0,0,1,3);
    buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Apply | QDialogButtonBox::Cancel | QDialogButtonBox::Help);
    connect(buttonBox, &QDialogButtonBox::clicked, this, &PropertyDialog::clicked);
    layout->addWidget(buttonBox,1,2);
    buttonResize = new QPushButton(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DialogResetButton)), "Resize");
    layout->addWidget(buttonResize,1,1);
    setWindowTitle(QString("Properties"));

    connect(buttonResize, &QPushButton::clicked, this, &PropertyDialog::updateWidget);
  }

  void PropertyDialog::clicked(QAbstractButton *button) {
    if(button == buttonBox->button(QDialogButtonBox::Ok))
      accept();
    else if(button == buttonBox->button(QDialogButtonBox::Apply))
      emit apply();
    else if(button == buttonBox->button(QDialogButtonBox::Cancel))
      reject();
    else if(button == buttonBox->button(QDialogButtonBox::Help)) {
      emit showXMLHelp();
    }
  }

  void PropertyDialog::addToTab(const QString &name, QWidget* widget_) {
    layout[name]->insertWidget(layout[name]->count()-1,widget_);
  }

  void PropertyDialog::addStretch(int s) {
    for (auto & it : layout)
      it.second->addStretch(s);
  }

  void PropertyDialog::addTab(const QString &name, int i) {  
    auto *tab = new QScrollArea;
    tab->setWidgetResizable(true);

    QWidget *box = new QWidget;
    auto *layout_ = new QVBoxLayout;
    layout_->setSpacing(15);
    box->setLayout(layout_);
    layout[name] = layout_;

    tab->setWidget(box);
    if(i==-1)
      tabWidget->addTab(tab, name);
    else 
      tabWidget->insertTab(i,tab,name);

    layout[name]->addStretch(1);
  }

  void PropertyDialog::setCancel(bool on) {
    buttonBox->button(QDialogButtonBox::Cancel)->setEnabled(on);
  }

  bool PropertyDialog::getCancel() const {
    return buttonBox->button(QDialogButtonBox::Cancel)->isEnabled();
  }
  
  void PropertyDialog::showEvent(QShowEvent *event) {
    QSettings settings;
    restoreGeometry(settings.value("propertydialog/geometry").toByteArray());
    QDialog::showEvent(event);
  }

  void PropertyDialog::hideEvent(QHideEvent *event) {
    QSettings settings;
    settings.setValue("propertydialog/geometry", saveGeometry());
    QDialog::hideEvent(event);
  }

  void PropertyDialog::closeEvent(QCloseEvent *event) {
    if(getCancel())
      QDialog::closeEvent(event);
    else
      event->ignore();
  }

  EmbedItemPropertyDialog::EmbedItemPropertyDialog(EmbedItemData *item_) : item(item_) {
    if(item->getEmbeded()) {
      buttonBox->button(QDialogButtonBox::Apply)->setDisabled(true);
      buttonBox->button(QDialogButtonBox::Ok)->setDisabled(true);
    }
  }

  void EmbedItemPropertyDialog::toWidget() {
    initializeUsingXML(item->getXMLElement());
  }

  void EmbedItemPropertyDialog::fromWidget() {
    writeXMLFile(item->getXMLElement());
  }

}
