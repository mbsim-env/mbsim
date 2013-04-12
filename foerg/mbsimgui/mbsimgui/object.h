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

#ifndef _OBJECT__H_
#define _OBJECT__H_

#include "element.h"
#include "extended_properties.h"

class QAction;
class VecWidget;

class Object : public Element {
  friend class ObjectPropertyDialog;
  public:
    Object(const std::string &str, Element *parent);
    ~Object();
    virtual int getqSize() {return 0;}
    virtual int getuSize() {return 0;}
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual Element* getByPathSearch(std::string path);
    PropertyDialog* createPropertyDialog() {return new ObjectPropertyDialog(this);}
  protected:
    QAction *actionSaveAs;
    ExtProperty q0, u0;

};

#endif
