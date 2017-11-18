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

  CompoundContour::CompoundContour(const string &name, Frame *R) : RigidContour(name,R) {
  }

  CompoundContour::~CompoundContour() {
    for(auto & i : element)
      delete i;
    for(auto & i : frame)
      delete i;
  }

  void CompoundContour::addContour(RigidContour* c) {
    element.push_back(c);
    c->setParent(this);
  }

  void CompoundContour::plot() {
    for (auto & i : element)
      i->plot();
  }

  void CompoundContour::addFrame(FixedRelativeFrame* f) {
    frame.push_back(f);
    f->setParent(this);
  }

  void CompoundContour::init(InitStage stage, const InitConfigSet &config) {
    if (stage == unknownStage) {
      for (auto & i : element)
        i->sethSize(hSize[0]);
    }
    else if (stage == plotting) {
      if (plotFeature[openMBV] and openMBVGroup == nullptr) {
        openMBVGroup = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
        openMBVGroup->setName(name + "Group");
        if (parent)
          parent->getOpenMBVGrp()->addObject(openMBVGroup);
        if (plotFeature[separateFilePerGroup])
          openMBVGroup->setSeparateFile(true);
      }
    }
    RigidContour::init(stage, config);

    for (auto & i : element)
      i->init(stage, config);
  }

  void CompoundContour::resetUpToDate() {
    for (auto & i : frame)
      i->resetUpToDate();
  }

}
