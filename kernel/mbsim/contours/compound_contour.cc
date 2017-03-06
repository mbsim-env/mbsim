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

#include<config.h>
#include "mbsim/contours/compound_contour.h"
#include "mbsim/frames/fixed_relative_frame.h"

#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;

namespace MBSim {

  CompoundContour::CompoundContour(const string &name, Frame *R) :
      RigidContour(name,R)
  {
  }

  CompoundContour::~CompoundContour() {
    for(unsigned int i=0; i<element.size(); i++)
      delete element[i];
    for(unsigned int i=0; i<frame.size(); i++)
      delete frame[i];
  }

  void CompoundContour::addContour(RigidContour* c) {
    element.push_back(c);
    c->setParent(this);
  }

  void CompoundContour::plot() {
    for (size_t i = 0; i < element.size(); i++) {
      element[i]->plot();
    }
  }

  void CompoundContour::addFrame(FixedRelativeFrame* f) {
    frame.push_back(f);
    f->setParent(this);
  }

  void CompoundContour::init(InitStage stage) {
    if (stage == unknownStage) {
      Contour::init(stage);
      for (unsigned int i = 0; i < element.size(); i++)
        element[i]->sethSize(hSize[0]);
    }
    else if (stage == plotting) {
      if (parent)
        updatePlotFeatures();

      if (plotFeature[11334901831169464975ULL] == enabled) {
        if (openMBVGroup == 0) {
          openMBVGroup = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
          openMBVGroup->setName(name + "Group");
          //if(parent) parent->openMBVGrp->addObject(openMBVGrp);
          if (parent)
            parent->getOpenMBVGrp()->addObject(openMBVGroup);
          if (plotFeature[18269718848207088804ULL] == enabled)
            openMBVGroup->setSeparateFile(true);
        }
      }
    }
    Contour::init(stage);

    for (unsigned int i = 0; i < element.size(); i++) {
      //element[i]->setParent(parent); // PARENT
      element[i]->init(stage);
    }
  }

  void CompoundContour::resetUpToDate() {
    for (unsigned int i = 0; i < frame.size(); i++)
      frame[i]->resetUpToDate();
  }

}
