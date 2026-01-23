/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#ifndef _BODY__H_
#define _BODY__H_

#include "object.h"

namespace MBSimGUI {

  class Body : public Object {
    MBSIMGUI_OBJECTFACTORY_CLASS(Body, Object, MBSIM%"Body", "Body");
    public:
      Body();
      ~Body() override;
      Element * getChildByContainerAndName(const QString &container, const QString &name) const override;
      int getNumberOfFrames() override { return frame.size(); }
      int getNumberOfContours() override { return contour.size(); }
      int getIndexOfFrame(Frame *frame_) override;
      int getIndexOfContour(Contour *contour_) override;
      Frame* getFrame(int i) const override { return frame[i]; }
      Contour* getContour(int i) const override { return contour[i]; }
      Frame* getFrame(const QString &name) const override;
      Contour* getContour(const QString &name) const;
      void setFrame(Frame *frame_, int i) override { frame[i] = frame_; }
      void setContour(Contour *contour_, int i) override { contour[i] = contour_; }
      void addFrame(Frame *frame_) override;
      void addContour(Contour *contour_) override;
      void removeElement(Element* element) override;
      void clear() override;
      void updateStatus() override;
      void createDiagramItem() override;
      void createDiagramArrows() override;
    protected:
      std::vector<Frame*> frame;
      std::vector<Contour*> contour;
  };

}

#endif
