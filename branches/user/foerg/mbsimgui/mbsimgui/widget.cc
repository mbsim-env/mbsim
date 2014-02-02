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
#include "widget.h"
#include "property.h"
#include <QHBoxLayout>
#include <QComboBox>
#include <QGroupBox>
#include <iostream>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

Widget::Widget(const Units &units, int defaultUnit) : unit(0) {
  QHBoxLayout *mainlayout = new QHBoxLayout;
  setLayout(mainlayout);
  mainlayout->setMargin(0);
  QGroupBox *box = new QGroupBox("Variable");
  varlayout = new QGridLayout;
  varlayout->setMargin(0);
  box->setLayout(varlayout);
  mainlayout->addWidget(box);
  if(units.getNumberOfUnits()) {
    QGroupBox *box = new QGroupBox("Unit");
    QHBoxLayout *layout = new QHBoxLayout;
    mainlayout->addWidget(box);
    layout->setMargin(0);
    box->setLayout(layout);
    unit = new QComboBox;
    for(int i=0; i<units.getNumberOfUnits(); i++)
      unit->addItem(QString::fromStdString(units.getUnit(i)));
    unit->setCurrentIndex((defaultUnit==-1)?units.getDefaultUnit():defaultUnit);
    layout->addWidget(unit);
  }
}

void Widget::fromProperty(Property *property) {
  if(unit)
    unit->setCurrentIndex(property->getCurrentUnit());
}

void Widget::toProperty(Property *property) {
  if(unit)
    property->setCurrentUnit(unit->currentIndex());
}


