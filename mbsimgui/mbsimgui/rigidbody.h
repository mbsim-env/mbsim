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

#ifndef _RIGIDBODY__H_
#define _RIGIDBODY__H_

#include "body.h"

class RigidBody : public Body {
  Q_OBJECT
  public:
    RigidBody(const QString &str, QTreeWidgetItem *parentItem, int ind);
    ~RigidBody();
    virtual int getqSize() {return getSize();}
    virtual int getuSize() {return getSize();}
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    QString getType() const { return "RigidBody"; }
    void setConstrained(bool b) {constrained = b;}
    int getSize() const {return constrained ? 0 : getUnconstrainedSize();}
    int getUnconstrainedSize() const; 
    void resizeGeneralizedPosition();
    void resizeGeneralizedVelocity();
  public slots:
    void addFrame();
    void addPoint();
    void addLine();
    void addPlane();
    void addSphere();
  protected:
    ExtXMLWidget *frameForKinematics, *mass, *inertia, *translation, *rotation, *frameOfReference, *framePos, *contourPos, *ombvEditor, *weightArrow, *jointForceArrow, *jointMomentArrow;
    bool constrained;
};

#endif
