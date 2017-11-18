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
      ~Body() override;
      Element * getChildByContainerAndName(const QString &container, const QString &name) const override;
      int getNumberOfFrames() override {return frame.size();}
      int getNumberOfContours() override {return contour.size();}
      int getIndexOfFrame(Frame *frame) override;
      int getIndexOfContour(Contour *contour) override;
      Frame* getFrame(int i) const override {return frame[i];}
      Contour* getContour(int i) const override {return contour[i];}
      Frame* getFrame(const QString &name) const override;
      Contour* getContour(const QString &name) const;
      void setFrame(Frame *frame_, int i) override { frame[i] = frame_; }
      void setContour(Contour *contour_, int i) override { contour[i] = contour_; }
      void addFrame(Frame *frame) override;
      void addContour(Contour *contour) override;
      void removeElement(Element* element) override;
    protected:
      std::vector<Frame*> frame;
      std::vector<Contour*> contour;
      std::vector<Element*> removedElement;
  };

}

#endif
