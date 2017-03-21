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

namespace MBSimGUI {

  class Body : public Object {
    public:
      Body(const std::string &str, Element *parent);
      Body(const Body &b);
      ~Body();
      Body& operator=(const Body &b);
      virtual Element * getChildByContainerAndName(const std::string &container, const std::string &name) const;
      int getNumberOfFrames() {return frame.size();}
      int getNumberOfContours() {return contour.size();}
      Frame* getFrame(int i) const {return frame[i];}
      Contour* getContour(int i) const {return contour[i];}
      Frame* getFrame(const std::string &name) const;
      Contour* getContour(const std::string &name) const;
      void addFrame(Frame *frame);
      void addContour(Contour *contour);
      void removeElement(Element* element);
    protected:
      std::vector<Frame*> frame;
      std::vector<Contour*> contour;
      std::vector<Element*> removedElement;
  };

}

#endif
