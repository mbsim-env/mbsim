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

#ifndef _BODY__H_
#define _BODY__H_

#include "object.h"

class Body : public Object {
  friend class BodyPropertyDialog;
  public:
    Body(const std::string &str, Element *parent);
    ~Body();
    virtual Element* getByPathSearch(std::string path);

    int getNumberOfFrames() {return frame.size();}
    int getNumberOfContours() {return contour.size();}

    Frame* getFrame(int i) {return frame[i];}
    Contour* getContour(int i) {return contour[i];}

    Frame* getFrame(const std::string &name, bool check=true);
    Contour* getContour(const std::string &name, bool check=true);

    void addFrame(Frame *frame);
    void addContour(Contour *contour);
    void removeElement(Element* element);

    PropertyDialog* createPropertyDialog() {return new BodyPropertyDialog(this);}
  protected:
    std::vector<Frame*> frame;
    std::vector<Contour*> contour;
};

#endif
