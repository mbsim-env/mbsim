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
#include "mbsim/body.h"
#include "mbsim/frame.h"
#include "mbsim/contour.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/dynamic_system_solver.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#include <openmbvcppinterface/body.h>
#endif

using namespace std;

namespace MBSim {

  Body::Body(const string &name) : Object(name), R(0),
#ifdef HAVE_OPENMBVCPPINTERFACE
                                     openMBVBody(0), openMBVGrp(0), 
#endif
                                   saved_frameOfReference("") {
                                   } 

  Body::~Body() {
    for(vector<Frame*>::iterator i = frame.begin(); i != frame.end(); ++i) 
      delete *i;
    for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i) 
      delete *i;
  }

  void Body::sethSize(int hSize_, int j) {
    Object::sethSize(hSize_, j);

    for(vector<Frame*>::iterator i=frame.begin(); i!=frame.end(); i++)
      (*i)->sethSize(hSize[j],j);
    for(vector<Contour*>::iterator i=contour.begin(); i!=contour.end(); i++) 
      (*i)->sethSize(hSize[j],j);
  }

  void Body::sethInd(int hInd_, int j) {
    Object::sethInd(hInd_, j);

    for(vector<Frame*>::iterator i=frame.begin(); i!=frame.end(); i++) 
      (*i)->sethInd(hInd[j],j);
    for(vector<Contour*>::iterator i=contour.begin(); i!=contour.end(); i++) 
      (*i)->sethInd(hInd[j],j);
  }  

  void Body::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      Object::plot(t,dt);

      for(unsigned int j=0; j<frame.size(); j++)
        frame[j]->plot(t,dt);
      for(unsigned int j=0; j<contour.size(); j++)
        contour[j]->plot(t,dt);
    }
  }

  void Body::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Object::closePlot();

      for(unsigned int j=0; j<frame.size(); j++)
        frame[j]->closePlot();
      for(unsigned int j=0; j<contour.size(); j++)
        contour[j]->closePlot();
    }
  }

  void Body::setDynamicSystemSolver(DynamicSystemSolver* sys) {
    Object::setDynamicSystemSolver(sys);

    for(unsigned i=0; i<frame.size(); i++)
      frame[i]->setDynamicSystemSolver(sys);
    for(unsigned i=0; i<contour.size(); i++)
      contour[i]->setDynamicSystemSolver(sys);
  }

  void Body::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      Object::init(stage);
    }
    if(stage==preInit) {
      Object::init(stage);
      if(!R)
        R = dynamic_cast<DynamicSystem*>(parent)->getFrameI();
      Body* obj = dynamic_cast<Body*>(R->getParent());
      if(obj)
        dependency.push_back(obj);
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          openMBVGrp=new OpenMBV::Group();
          openMBVGrp->setName(name+"_Group");
          openMBVGrp->setExpand(false);
          parent->getOpenMBVGrp()->addObject(openMBVGrp);
          if(getPlotFeature(openMBV)==enabled && openMBVBody) {
            openMBVBody->setName(name);
            openMBVGrp->addObject(openMBVBody);
          }
        }
#endif
        Object::init(stage);
      }
    }
    else
      Object::init(stage);

    for(vector<Frame*>::iterator i=frame.begin(); i!=frame.end(); i++) 
      (*i)->init(stage);
    for(vector<Contour*>::iterator i=contour.begin(); i!=contour.end(); i++) 
      (*i)->init(stage);
  }

  void Body::addContour(Contour* contour_) {
    if(getContour(contour_->getName(),false)) { //Contourname exists already
      throw MBSimError("ERROR in "+getName()+" (Body::addContour): The body can only comprise one contour by the name \""+contour_->getName()+"\"!");
      assert(getContour(contour_->getName(),false)==NULL);
    }
    contour.push_back(contour_);
    contour_->setParent(this);
  }

  void Body::addFrame(Frame* frame_) {
    if(getFrame(frame_->getName(),false)) { //Contourname exists already
      throw MBSimError("ERROR in "+getName()+" (Body::addFrame): The body can only comprise one frame by the name \""+frame_->getName()+"\"!");
      assert(getFrame(frame_->getName(),false)==NULL);
    }
    frame.push_back(frame_);
    frame_->setParent(this);
  }

  Contour* Body::getContour(const string &name_, bool check) {
    unsigned int i;
    for(i=0; i<contour.size(); i++) {
      if(contour[i]->getName() == name_)
        return contour[i];
    }
    if(check) {
      if(!(i<contour.size()))
        throw MBSimError("ERROR in "+getName()+" (Body::getContour): The body comprises no contour \""+name_+"\"!"); 
      assert(i<contour.size());
    }
    return NULL;
  }

  Frame* Body::getFrame(const string &name_, bool check) {
    unsigned int i;
    for(i=0; i<frame.size(); i++) {
      if(frame[i]->getName() == name_)
        return frame[i];
    }             
    if(check) {
      if(!(i<frame.size()))
        throw MBSimError("ERROR in "+getName()+": (Body::getFrame): The body comprises no frame \""+name_+"\"!"); 
      assert(i<frame.size());
    }
    return NULL;
  }

  int Body::frameIndex(const Frame *frame_) const {
    for(unsigned int i=0; i<frame.size(); i++) {
      if(frame_==frame[i])
        return i;
    }
    return -1;
  }

  int Body::contourIndex(const Contour *contour_) const {
    for(unsigned int i=0; i<contour.size(); i++) {
      if(contour_==contour[i])
        return i;
    }
    return -1;
  }

  void Body::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    Object::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMNS"frameOfReference");
    if(e) saved_frameOfReference=e->Attribute("ref");
  }

  TiXmlElement* Body::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Object::writeXMLFile(parent);
    TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"frameOfReference" );
    // ele1->SetAttribute("ref", getFrameOfReference()->getXMLPath()); // absolute path
    ele1->SetAttribute("ref", R->getXMLPath(this,true)); // relative path
    ele0->LinkEndChild(ele1);
    return ele0;
  }

  Element * Body::getByPathSearch(string path) {
    if (path.substr(0, 1)=="/") // absolut path
      if(parent)
        return parent->getByPathSearch(path);
      else
        return getByPathSearch(path.substr(1));
    else if (path.substr(0, 3)=="../") // relative path
      return parent->getByPathSearch(path.substr(3));
    else { // local path
      size_t pos0=path.find_first_of("[", 0);
      string container=path.substr(0, pos0);
      size_t pos1=path.find_first_of("]", pos0);
      string searched_name=path.substr(pos0+1, pos1-pos0-1);
      if (container=="Frame")
        return getFrame(searched_name);
      else if (container=="Contour")
        return getContour(searched_name);
      else
        throw MBSimError("ERROR in "+getName()+" (Body::getByPathSearch): Unknown name of container!");
    }
  }

}

