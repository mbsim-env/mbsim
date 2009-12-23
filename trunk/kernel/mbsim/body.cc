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
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h>
#include "mbsim/body.h"
#include "mbsim/frame.h"
#include "mbsim/contour.h"
#include "mbsim/dynamic_system.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#endif

using namespace std;

namespace MBSim {

  Body::Body(const string &name) : Object(name) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVBody=0;
    openMBVGrp=0;
#endif
  } 

  Body::~Body() {
    for(vector<Frame*>::iterator i = frame.begin(); i != frame.end(); ++i) 
      delete *i;
    for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i) 
      delete *i;
#ifdef HAVE_OPENMBVCPPINTERFACE
    if(openMBVBody) { delete openMBVBody; openMBVBody=0; }
#endif
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
#ifdef HAVE_OPENMBVCPPINTERFACE
    if(openMBVGrp) { delete openMBVGrp; openMBVGrp=0; }
    if(openMBVBody) { delete openMBVBody; openMBVBody=0; }
#endif
  }

  void Body::setDynamicSystemSolver(DynamicSystemSolver* sys) {
    Object::setDynamicSystemSolver(sys);

    for(unsigned i=0; i<frame.size(); i++)
      frame[i]->setDynamicSystemSolver(sys);
    for(unsigned i=0; i<contour.size(); i++)
      contour[i]->setDynamicSystemSolver(sys);
  }

  void Body::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures(parent);

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          openMBVGrp=new OpenMBV::Group();
          openMBVGrp->setName(name+"#Group");
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
      cout << "Error: The body \"" << name << "\" can only comprise one contour by the name \"" << contour_->getName() << "\"!" << endl;
      assert(getContour(contour_->getName(),false)==NULL);
    }
    contour.push_back(contour_);
    contour_->setParent(this);
  }

  void Body::addFrame(Frame* frame_) {
    if(getFrame(frame_->getName(),false)) { //Contourname exists already
      cout << "Error: The body \"" << name << "\" can only comprise one frame by the name \"" << frame_->getName() << "\"!" << endl;
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
      if(!(i<contour.size())) cout << "Error: The body \"" << name << "\" comprises no contour \"" << name_ << "\"!" << endl; 
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
      if(!(i<frame.size())) cout << "Error: The body \"" << name << "\" comprises no frame \"" << name_ << "\"!" << endl; 
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

  Frame *Body::getFrameByPath(string path) {
    if(path[path.length()-1]!='/') path=path+"/";
    size_t i=path.find('/');
    // absolut path
    if(i==0) {
      if(parent)
        return parent->getFrameByPath(path);
      else
        return getFrameByPath(path.substr(1));
    }
    // relative path
    string firstPart=path.substr(0, i);
    string restPart=path.substr(i+1);
    if(firstPart=="..")
      return parent->getFrameByPath(restPart);
    else if(firstPart.substr(0,6)=="Frame[")
      return getFrame(firstPart.substr(6,firstPart.find(']')-6));
    else
      return 0;
  }

  Contour *Body::getContourByPath(string path) {
    if(path[path.length()-1]!='/') path=path+"/";
    size_t i=path.find('/');
    // absolut path
    if(i==0) {
      if(parent)
        return parent->getContourByPath(path);
      else
        return getContourByPath(path.substr(1));
    }
    // relative path
    string firstPart=path.substr(0, i);
    string restPart=path.substr(i+1);
    if(firstPart=="..")
      return parent->getContourByPath(restPart);
    else if(firstPart.substr(0,8)=="Contour[")
      return getContour(firstPart.substr(8,firstPart.find(']')-8));
    else
      return 0;
  }

}

