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
#include "property_widget.h"
#include "extended_widgets.h"
#include <QtGui>

using namespace std;

PropertyWidget::PropertyWidget(QObject *parentObject_) : parentObject(parentObject_) {

  setWindowTitle("Properties");
}

PropertyWidget::~PropertyWidget() {
}

void PropertyWidget::addToTab(const QString &name, ExtXMLWidget* widget_) {
  layout[name]->addWidget(widget_);
  widget.push_back(widget_);
}

void PropertyWidget::addStretch() {
  for ( std::map<QString,QVBoxLayout*>::iterator it=layout.begin() ; it != layout.end(); it++ )
    (*it).second->addStretch(1);
}

void PropertyWidget::update() {
  for(unsigned int i=0; i<widget.size(); i++)
    widget[i]->update();
}

void PropertyWidget::initialize() {
  for(unsigned int i=0; i<widget.size(); i++)
    widget[i]->initialize();
}

void PropertyWidget::resizeVariables() {
  for(unsigned int i=0; i<widget.size(); i++)
    widget[i]->resizeVariables();
}

void PropertyWidget::addTab(const QString &name) {  
  QScrollArea *tab = new QScrollArea;
  tab->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  tab->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  tab->setWidgetResizable(true);
  QWidget *widget = new QWidget;
  QHBoxLayout *hlo = new QHBoxLayout;

  QWidget *box = new QWidget;
  QVBoxLayout *layout_ = new QVBoxLayout;
  box->setLayout(layout_);
  layout[name] = layout_;
  hlo->addWidget(box);

  widget->setLayout(hlo);
  tab->setWidget(widget);
  addTab(tab, name);
}

void PropertyWidget::setParentObject(QObject *parentObject_) {
  parentObject=parentObject_;
}

void PropertyWidget::initializeUsingXML(TiXmlElement *element) {
  for(unsigned int i=0; i<widget.size(); i++)
    widget[i]->initializeUsingXML(element);
}

TiXmlElement* PropertyWidget::writeXMLFile(TiXmlNode *parent) {
  for(unsigned int i=0; i<widget.size(); i++)
    widget[i]->writeXMLFile(parent);
}
