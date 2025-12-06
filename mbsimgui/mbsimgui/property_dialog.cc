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

#include <config.h>
#include "property_dialog.h"
#include "embeditemdata.h"
#include "basic_widgets.h"
#include "mainwindow.h"
#include <QGridLayout>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QStyle>
#include <QSettings>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  BasicPropertyDialog::BasicPropertyDialog() : QDialog(mw) {
    assert(mw);
  }

  void BasicPropertyDialog::showEvent(QShowEvent *event) {
    mw->prepareForPropertyDialogOpen();
    QDialog::showEvent(event);
  }

  void BasicPropertyDialog::hideEvent(QHideEvent *event) {
    mw->prepareForPropertyDialogClose();
    QDialog::hideEvent(event);
  }

  PropertyDialog::PropertyDialog(const QString& title) : BasicPropertyDialog() {

    auto *layout = new QGridLayout;
    setLayout(layout);
    tabWidget = new QTabWidget(this);
    layout->addWidget(tabWidget,0,0,1,3);
    buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Apply | QDialogButtonBox::Cancel | QDialogButtonBox::Help);
    connect(buttonBox, &QDialogButtonBox::clicked, this, &PropertyDialog::clicked);
    layout->addWidget(buttonBox,1,2);
    buttonResize = new QPushButton(style()->standardIcon(QStyle::StandardPixmap(QStyle::SP_DialogResetButton)), "Resize");
    layout->addWidget(buttonResize,1,1);
    setWindowTitle(title);

    connect(buttonResize, &QPushButton::clicked, this, &PropertyDialog::updateWidget);
  }

  void PropertyDialog::keyPressEvent(QKeyEvent *e) {
    if(e->key() == Qt::Key_Escape)
      return; // block Esc: a silent dialog close is using ESC is very unconvinient
    BasicPropertyDialog::keyPressEvent(e);
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

    layout[name]->addStretch(0);
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
    BasicPropertyDialog::showEvent(event);
  }

  void PropertyDialog::hideEvent(QHideEvent *event) {
    QSettings settings;
    settings.setValue("propertydialog/geometry", saveGeometry());
    BasicPropertyDialog::hideEvent(event);
  }

  void PropertyDialog::closeEvent(QCloseEvent *event) {
    if(getCancel())
      QDialog::closeEvent(event);
    else
      event->ignore();
  }

  EmbedItemPropertyDialog::EmbedItemPropertyDialog(const QString &title, EmbedItemData *item_) : PropertyDialog(title), item(item_), npl(mw->eval) {
    mw->setCurrentlyEditedItem(item);
    mw->updateParameters(item);
  }

  void EmbedItemPropertyDialog::toWidget() {
    initializeUsingXML(item->getXMLElement());
  }

  void EmbedItemPropertyDialog::fromWidget() {
    writeXMLFile(item->getXMLElement());
  }

  void EmbedItemPropertyDialog::showEvent(QShowEvent *ev) {
    PropertyDialog::showEvent(ev);
  }

  void EmbedItemPropertyDialog::hideEvent(QHideEvent *ev) {
    mw->setCurrentlyEditedItem(nullptr);
    PropertyDialog::hideEvent(ev);
  }

  UnknownItemPropertyDialog::UnknownItemPropertyDialog(EmbedItemData *item) : EmbedItemPropertyDialog("Element Properties", item) {
    addTab("General");
    editor = new ExtWidget("XML Editor",new XMLEditorWidget);
    addToTab("General", editor);
  }

  DOMElement* UnknownItemPropertyDialog::initializeUsingXML(DOMElement *parent) {
    editor->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* UnknownItemPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    item->setXMLElement(editor->writeXMLFile(item->getXMLElement(),ref));
    item->updateName();
    return nullptr;
  }

}
