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
#include <QtGui/QActionGroup>
#include "utils.h"
#include <editors.h>

class Frame : public Element {
  Q_OBJECT
  private:
  public:
    Frame(const QString &str, QTreeWidgetItem *parentItem, int ind, bool grey=false);
    ~Frame();
    void resetAnimRange(int numOfRows, double dt);
    static int counter;
    QString getType() const { return "Frame"; }
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual void initializeUsingXML2(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile2(TiXmlNode *element);
    bool openMBVFrame() const {return ((OMBVObjectChoiceWidget*)visu->getXMLWidget())->openMBVObject();}
  public slots:
  protected:
    XMLEditor *visu;
    friend class MainWindow;
};

#endif
