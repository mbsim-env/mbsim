/* Copyright (C) 2004-2016 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/links/mechanical_link.h"
#include "mbsim/frames/frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  const PlotFeatureEnum force;
  const PlotFeatureEnum moment;

  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, force)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, moment)

  MechanicalLink::MechanicalLink(const std::string &name) : Link(name), updF(true), updM(true), updRMV(true), updlaF(true), updlaM(true) {
  }

  void MechanicalLink::resetUpToDate() {
    Link::resetUpToDate(); 
    updF = true; 
    updM = true; 
    updRMV = true;
    updlaF = true;
    updlaM = true;
  }

  void MechanicalLink::updateGeneralizedForces() {
    lambda.set(iF, evallaF());
    lambda.set(iM, evallaM());
    updla = false;
  }

  void MechanicalLink::plot() {
    if(plotFeature[plotRecursive]) {
      if(plotFeature[force]) {
        for(size_t i=0; i<F.size(); i++)
	  Element::plot(evalForce(i));
      }
      if(plotFeature[moment]) {
        for(size_t i=0; i<M.size(); i++)
	  Element::plot(evalMoment(i));
      }
    }
    Link::plot();
  }

  void MechanicalLink::init(InitStage stage, const InitConfigSet &config) {
    if(stage==plotting) {
      if(plotFeature[plotRecursive]) {
        if(plotFeature[force]) {
          for(size_t i=0; i<F.size(); i++)
	    addToPlot("force "+to_string(convertIndex(i)),{"x","y","z"});
        }
        if(plotFeature[moment]) {
          for(size_t i=0; i<M.size(); i++)
	    addToPlot("moment "+to_string(convertIndex(i)),{"x","y","z"});
        }
      }
    }
    Link::init(stage, config);
  }

}
