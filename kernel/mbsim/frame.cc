/* Copyright (C) 2004-2009 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/frame.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/rigid_body.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/mbsim_event.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/group.h>
#endif

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  Frame::Frame(const string &name) : Element(name), AWP(EYE) {

    hSize[0] = 0;
    hSize[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;

#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVFrame=0;
#endif
  }

  void Frame::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(globalPosition)==enabled) {
        for(int i=0; i<3; i++)
          plotVector.push_back(WrOP(i));
        Vec3 cardan=AIK2Cardan(AWP);
        for(int i=0; i<3; i++)
          plotVector.push_back(cardan(i));
      }
      if(getPlotFeature(globalVelocity)==enabled) {
        for(int i=0; i<3; i++)
          plotVector.push_back(WvP(i));
        for(int i=0; i<3; i++)
          plotVector.push_back(WomegaP(i));
      }
      if(getPlotFeature(globalAcceleration)==enabled) {
        for(int i=0; i<3; i++)
          plotVector.push_back(WaP(i));
        for(int i=0; i<3; i++)
          plotVector.push_back(WpsiP(i));
      }
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVFrame && !openMBVFrame->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(WrOP(0));
          data.push_back(WrOP(1));
          data.push_back(WrOP(2));
          Vec3 cardan=AIK2Cardan(AWP);
          data.push_back(cardan(0));
          data.push_back(cardan(1));
          data.push_back(cardan(2));
          data.push_back(0);
          openMBVFrame->append(data);
        }
      }
#endif
      Element::plot(t,dt);
    }
  }

  void Frame::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::closePlot();
    }
  }

  void Frame::init(InitStage stage) {
    if(stage==resize) {
      WJP[0].resize(hSize[0]);
      WJR[0].resize(hSize[0]);
      WJP[1].resize(hSize[1]);
      WJR[1].resize(hSize[1]);
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();
  
      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(globalPosition)==enabled) {
          for(int i=0; i<3; i++)
            plotColumns.push_back("WrOP("+numtostr(i)+")");
          plotColumns.push_back("alpha");
          plotColumns.push_back("beta");
          plotColumns.push_back("gamma");
        }
        if(getPlotFeature(globalVelocity)==enabled) {
          for(int i=0; i<3; i++)
            plotColumns.push_back("WvP("+numtostr(i)+")");
          for(int i=0; i<3; i++)
            plotColumns.push_back("WomegaP("+numtostr(i)+")");
        }
        if(getPlotFeature(globalAcceleration)==enabled) {
          for(int i=0; i<3; i++)
            plotColumns.push_back("WaP("+numtostr(i)+")");
          for(int i=0; i<3; i++)
            plotColumns.push_back("WpsiP("+numtostr(i)+")");
        }
  
  #ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVFrame) {
            openMBVFrame->setName(name);
            parent->getOpenMBVGrp()->addObject(openMBVFrame);
          }
        }
  #endif
        Element::init(stage);
      }
    }
    else
      Element::init(stage);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void Frame::enableOpenMBV(double size, double offset) {
    if(size>=0) {
      openMBVFrame=new OpenMBV::Frame;
      openMBVFrame->setSize(size);
      openMBVFrame->setOffset(offset);
    }
    else {
      openMBVFrame=0;
    }
  }
#endif

  void Frame::initializeUsingXML(TiXmlElement *element) {
    Element::initializeUsingXML(element);

#ifdef HAVE_OPENMBVCPPINTERFACE
    TiXmlElement *ee;
    if((ee=element->FirstChildElement(MBSIMNS"enableOpenMBV"))) {
      enableOpenMBV(getDouble(ee->FirstChildElement(MBSIMNS"size")),
          getDouble(ee->FirstChildElement(MBSIMNS"offset")));

      // pass a OPENMBV_ID processing instruction to the OpenMBV Frame object
      for(TiXmlNode *child=ee->FirstChild(); child; child=child->NextSibling()) {
        TiXmlUnknown *unknown=child->ToUnknown();
        const size_t length=strlen("?OPENMBV_ID ");
        if(unknown && unknown->ValueStr().substr(0, length)=="?OPENMBV_ID ")
          openMBVFrame->setID(unknown->ValueStr().substr(length, unknown->ValueStr().length()-length-1));
      }
    }
    if((ee=element->FirstChildElement(MBSIMNS"openMBVFrame"))) {
      OpenMBV::Frame *f=new OpenMBV::Frame;
      setOpenMBVFrame(f);
      f->initializeUsingXML(ee->FirstChildElement());
    }
#endif
  }

  TiXmlElement* Frame::writeXMLFile(TiXmlNode *parent) {

    TiXmlElement *ele0 = Element::writeXMLFile(parent);

#ifdef HAVE_OPENMBVCPPINTERFACE
    if(openMBVFrame) {
      TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"enableOpenMBV" );
      addElementText(ele1,MBSIMNS"size",openMBVFrame->getSize());
      addElementText(ele1,MBSIMNS"offset",openMBVFrame->getOffset());
      ele0->LinkEndChild(ele1);
    }
#endif
    return ele0;
  }

  Element *Frame::getByPathSearch(string path) {
    if (path.substr(0, 1)=="/") // absolut path
      if(parent)
        return parent->getByPathSearch(path);
      else
        return getByPathSearch(path.substr(1));
    else if (path.substr(0, 3)=="../") // relative path
      return parent->getByPathSearch(path.substr(3));
    else { // local path
      throw;
    }
  }

  void FixedRelativeFrame::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      Frame::init(stage);
    }
    else
      Frame::init(stage);
  }

  void FixedRelativeFrame::initializeUsingXML(TiXmlElement *element) {
    Frame::initializeUsingXML(element);
    TiXmlElement *ec=element->FirstChildElement();
    ec=element->FirstChildElement(MBSIMNS"frameOfReference");
    if(ec) setFrameOfReference(ec->Attribute("ref"));
    ec=element->FirstChildElement(MBSIMNS"relativePosition");
    if(ec) setRelativePosition(getVec3(ec));
    ec=element->FirstChildElement(MBSIMNS"relativeOrientation");
    if(ec) setRelativeOrientation(getSqrMat3(ec));
  }

  TiXmlElement* FixedRelativeFrame::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Frame::writeXMLFile(parent);
    return ele0;
  }

}

