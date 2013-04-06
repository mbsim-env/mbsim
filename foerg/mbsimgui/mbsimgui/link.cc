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
#include "link.h"
#include <QtGui/QMenu>
#include "mainwindow.h"

using namespace std;


Link::Link(const QString &str, QTreeWidgetItem *parentItem, int ind) : Element(str, parentItem, ind) {

  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Remove", this);
  connect(action,SIGNAL(triggered()),this,SLOT(remove()));
  contextMenu->addAction(action);
}

Link::~Link() {
}

Element * Link::getByPathSearch(QString path) {
  if (path.mid(0, 3)=="../") // relative path
    return getParentElement()->getByPathSearch(path.mid(3));
  else // absolut path
    if(getParentElement())
      return getParentElement()->getByPathSearch(path);
    else
      return getByPathSearch(path.mid(1));
}

//void Link::initializeUsingXML(TiXmlElement *element) {
//}

//TiXmlElement* Link::writeXMLFile(TiXmlNode *parent) {    
//}
