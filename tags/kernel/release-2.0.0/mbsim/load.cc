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
#include "mbsim/load.h"
#include "mbsim/data_interface_base.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Load::Load(const string &name) : LinkMechanics(name), func(0), KOSYID(0) {
    fprintf(stderr,"The class Load in deprecated. It must be moved to mbsimControl! Use class Excitation for kinetic time dependent excitations.\n");
  }

  Load::~Load() {
    delete func;
  }

  void Load::updateh(double t) {
    la = (*func)(t);
    if(KOSYID) {
      Wf = frame[0]->getOrientation()*forceDir;
      Wm = frame[0]->getOrientation()*momentDir;
    }
    WF[0] = Wf*la(IT);
    WM[0] = Wm*la(IR);
    h[0] += trans(frame[0]->getJacobianOfTranslation())*WF[0] + trans(frame[0]->getJacobianOfRotation())*WM[0];
    hLink[0] += trans(frame[0]->getJacobianOfTranslation())*WF[0] + trans(frame[0]->getJacobianOfRotation())*WM[0];
  } 

  void Load::init(InitStage stage) {
    if(stage==unknownStage) {
      LinkMechanics::init(stage);
      IT = Index(0,forceDir.cols()-1);
      IR = Index(forceDir.cols(),forceDir.cols()+momentDir.cols()-1);
      if(forceDir.cols()) 
        Wf = forceDir;
      else {
        forceDir.resize(3,0);
        Wf.resize(3,0);
      }
      if(momentDir.cols())
        Wm = momentDir;
      else {
        momentDir.resize(3,0);
        Wm.resize(3,0);
      }
    }
    else
      LinkMechanics::init(stage);
  }

  void Load::calclaSize() {
    LinkMechanics::calclaSize();
    laSize = forceDir.cols()+momentDir.cols();
  }

  void Load::connect(Frame *frame_) {
    LinkMechanics::connect(frame_);
  }

  void Load::setForceDirection(const Mat &fd) {
    assert(fd.rows() == 3);

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.col(i) = forceDir.col(i)/nrm2(fd.col(i));
  }

  void Load::setMomentDirection(const Mat &md) {
    assert(md.rows() == 3);

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.col(i) = momentDir.col(i)/nrm2(md.col(i));
  }

  void Load::initDataInterfaceBase(DynamicSystemSolver *parentds) {
    if(DIBRefs.size()==1){
      DataInterfaceBase* in_=parentds->getDataInterfaceBase(DIBRefs[0]); 
      setSignal(in_); 
    }
  }

}

