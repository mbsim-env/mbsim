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
#include "contour.h"
#include <QMenu>

using namespace std;

Contour::Contour(const QString &str, QTreeWidgetItem *parentItem, int ind) : Element(str,parentItem,ind) {
  setText(1,getType());
}

Contour::~Contour() {
}

//virtual void Contour::initializeUsingXML(TiXmlElement *element) {
//  Element::initializeUsingXML(element);
//}
//
//TiXmlElement* Contour::writeXMLFile(TiXmlNode *parent) {
//  return Element::writeXMLFile(parent);
//}

Point::Point(const QString &str, QTreeWidgetItem *parentItem, int ind) : Contour(str,parentItem,ind) {
  properties->addStretch();
}

Point::~Point() {
}

Line::Line(const QString &str, QTreeWidgetItem *parentItem, int ind) : Contour(str,parentItem,ind) {
  properties->addStretch();
}

Line::~Line() {
}
