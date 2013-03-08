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

#ifndef _FRAME__H_
#define _FRAME__H_

#include "element.h"
#include "extended_properties.h"

class ExtWidget;

class Frame : public Element {
  public:
    Frame(const QString &str, QTreeWidgetItem *parentItem, int ind, bool grey=false);
    ~Frame();
    QString getType() const { return "Frame"; }
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual void initializeUsingXML2(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile2(TiXmlNode *element);
    bool openMBVFrame() const {return visuProperty.isActive();}
    void setOpenMBVFrame(bool flag) {visuProperty.setActive(flag);}
    virtual Element * getByPathSearch(QString path);
    virtual void initializeDialog();
    virtual void fromWidget();
    virtual void toWidget();
  protected:
    ExtWidget *visuWidget;
    ExtProperty visuProperty;
};

class FixedRelativeFrame : public Frame {
  public:
    FixedRelativeFrame(const QString &str, QTreeWidgetItem *parentItem, int ind);
    ~FixedRelativeFrame();
    QString getType() const { return "FixedRelativeFrame"; }
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual void initializeUsingXML2(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual void initialize();
    virtual void initializeDialog();
    virtual void fromWidget();
    virtual void toWidget();
  protected:
    ExtWidget *refFrameWidget, *positionWidget, *orientationWidget;
    ExtProperty refFrameProperty, positionProperty, orientationProperty;
};

#endif
