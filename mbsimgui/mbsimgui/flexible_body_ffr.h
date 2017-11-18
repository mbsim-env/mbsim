/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2016 Martin Förg

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

#ifndef _FLEXIBLE_BODY_FFR__H_
#define _FLEXIBLE_BODY_FFR__H_

#include "body.h"

namespace MBSimGUI {

  class FlexibleBodyFFR : public Body {
    friend class FlexibleBodyFFRPropertyDialog;
    public:
      FlexibleBodyFFR();
      QString getType() const override { return "FlexibleBodyFFR"; }
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMFLEX; }
      xercesc::DOMElement* getXMLFrames() override { return frames; }
      xercesc::DOMElement* getXMLContours() override { return contours; }
      void removeXMLElements() override;
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent) override;
      xercesc::DOMElement* processFileID(xercesc::DOMElement* element) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      ElementPropertyDialog* createPropertyDialog() override {return new FlexibleBodyFFRPropertyDialog(this);}
      QMenu* createFrameContextMenu() override {return new NodeFramesContextMenu(this);}
    protected:
      xercesc::DOMElement *frames, *contours;
  };

}

#endif
