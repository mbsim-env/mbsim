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
#include "body.h"
#include "frame.h"
#include "contour.h"

using namespace std;

Body::Body(const QString &str, QTreeWidgetItem *parentItem, int ind) : Object(str, parentItem, ind) {
}

Body::~Body() {
}

Element * Body::getByPathSearch(QString path) {
  if (path.mid(0, 1)=="/") // absolut path
    if(getParentElement())
      return getParentElement()->getByPathSearch(path);
    else
      return getByPathSearch(path.mid(1));
  else if (path.mid(0, 3)=="../") // relative path
    return getParentElement()->getByPathSearch(path.mid(3));
  else { // local path
    size_t pos0=path.indexOf("[", 0);
    QString container=path.mid(0, pos0);
    size_t pos1=path.indexOf("]", pos0);
    QString searched_name=path.mid(pos0+1, pos1-pos0-1);
    if (container=="Frame")
      return getFrame(searched_name);
    else if (container=="Contour")
      return getContour(searched_name);
    else {
      cout << "ERROR in "+getName().toStdString()+" (Body::getByPathSearch): Unknown name of container!" << endl;
      throw;
    }
  }
}


