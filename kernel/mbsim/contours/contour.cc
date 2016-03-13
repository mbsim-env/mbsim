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
#include "mbsim/contours/contour.h"
#include "mbsim/frames/contour_frame.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  Contour::Contour(const string &name) : Element(name), thickness(0.01) {
    hSize[0] = 0;
    hSize[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;
  }

  void Contour::init(InitStage stage) {
    if(stage==resize) {
      for(vector<ContourFrame*>::iterator i=frame.begin(); i!=frame.end(); i++) {
        (*i)->sethSize(hSize[0],0);
        (*i)->sethInd(hInd[0],0);
        (*i)->sethSize(hSize[1],1);
        (*i)->sethInd(hInd[1],1);
      }
      Element::init(stage);
    }
    else
      Element::init(stage);

    for(vector<ContourFrame*>::iterator i=frame.begin(); i!=frame.end(); i++)
      (*i)->init(stage);
  }

  Vec3 Contour::getPosition(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getPosition): Not implemented.");
    return 0;
  }

  Vec3 Contour::getKrPS(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getKrPS): Not implemented.");
    return 0;
  }

  Vec3 Contour::getKs(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getKs): Not implemented.");
    return 0;
  }

  Vec3 Contour::getKt(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getKt): Not implemented.");
    return 0;
  }
  Vec3 Contour::getParDer1Ks(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Ks): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Ks(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Ks): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer1Kt(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Kt): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Kt(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Kt): Not implemented.");
    return 0;
  }

  Vec3 Contour::getWu(double t, const Vec2 &zeta) {
    Vec3 Ws=getWs(t,zeta);
    return Ws/nrm2(Ws);
  }

  Vec3 Contour::getWv(double t, const Vec2 &zeta) {
    Vec3 Wt=getWt(t,zeta);
    return Wt/nrm2(Wt);
  }

  Vec3 Contour::getWn(double t, const Vec2 &zeta) {
    Vec3 N=crossProduct(getWs(t,zeta),getWt(t,zeta));
    return N/nrm2(N);
  }

  Vec3 Contour::getParDer1Kn(const fmatvec::Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Kn): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Kn(const fmatvec::Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Kn): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer1Ku(const Vec2 &zeta) {
    Vec3 Ks = getKs(zeta);
    Vec3 parDer1Ks = getParDer1Ks(zeta);
    return parDer1Ks/nrm2(Ks) - Ks*((Ks.T()*parDer1Ks)/pow(nrm2(Ks),3));
  }

  Vec3 Contour::getParDer2Ku(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Ku): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer1Kv(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Kv): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Kv(const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Kv): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer1Wn(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Wn): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Wn(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Wn): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer1Wu(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Wu): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Wu(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Wu): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer1Wv(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer1Wv): Not implemented.");
    return 0;
  }

  Vec3 Contour::getParDer2Wv(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getParDer2Wv): Not implemented.");
    return 0;
  }

  Vec3 Contour::getWrPS(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getWrPS): Not implemented.");
    return 0;
  }

  Vec3 Contour::getWs(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getWs): Not implemented.");
    return 0;
  }

  Vec3 Contour::getWt(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour::getWt): Not implemented.");
    return 0;
  }

  Mat3x2 Contour::getWN(double t, const Vec2 &zeta) {
    Mat3x2 WN(NONINIT);
    WN.set(0,getParDer1Wn(t,zeta));
    WN.set(1,getParDer2Wn(t,zeta));
    return WN;
  }

  Mat3x2 Contour::getWR(double t, const Vec2 &zeta) {
    Mat3x2 WR(NONINIT);
    WR.set(0,getWs(t,zeta));
    WR.set(1,getWt(t,zeta));
    return WR;
  }

  Mat3x2 Contour::getWU(double t, const Vec2 &zeta) {
    Mat3x2 WU(NONINIT);
    WU.set(0,getParDer1Wu(t,zeta));
    WU.set(1,getParDer2Wu(t,zeta));
    return WU;
  }

  Mat3x2 Contour::getWV(double t, const Vec2 &zeta) {
    Mat3x2 WV(NONINIT);
    WV.set(0,getParDer1Wv(t,zeta));
    WV.set(1,getParDer2Wv(t,zeta));
    return WV;
  }

  Vec2 Contour::getZeta(double t, const Vec3 &WrPS) {
    THROW_MBSIMERROR("(Contour::getZeta): Not implemented.");
    return 0;
  }

  ContourFrame* Contour::getFrame(const string &name_, bool check) const {
    unsigned int i;
    for(i=0; i<frame.size(); i++) {
      if(frame[i]->getName() == name_)
        return frame[i];
    }
    if(check) {
      if(!(i<frame.size()))
        THROW_MBSIMERROR("(Contour::getFrame): The body comprises no frame \""+name_+"\"!");
      assert(i<frame.size());
    }
    return NULL;
  }

  void Contour::addFrame(ContourFrame* frame_) {
    if(getFrame(frame_->getName(),false)) { //Contourname exists already
      THROW_MBSIMERROR("(Body::addFrame): The body can only comprise one frame by the name \""+frame_->getName()+"\"!");
      assert(getFrame(frame_->getName(),false)==NULL);
    }
    frame.push_back(frame_);
    frame_->setParent(this);
  }

  void Contour::resetUpToDate() {
    for(unsigned int i=0; i<frame.size(); i++)
      frame[i]->resetUpToDate();
  }

  void Contour::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      for(unsigned int j=0; j<frame.size(); j++)
        frame[j]->plot(t,dt);
#endif
    }
    Element::plot(t, dt);
  }

}
