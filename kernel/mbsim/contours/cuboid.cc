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
#include "mbsim/contours/cuboid.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plate.h"
#include "mbsim/contours/edge.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/utils/rotarymatrices.h"

#include <openmbvcppinterface/cuboid.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  Cuboid::Cuboid(const string &name, Frame *R) : CompoundContour(name,R) { }

  Cuboid::Cuboid(const string &name, double lx_, double ly_, double lz_, Frame *R) : CompoundContour(name,R), lx(lx_), ly(ly_), lz(lz_) { }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Cuboid)

  void Cuboid::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {

      Vec3 r;

      r(0) = lx/2.0;
      r(1) = ly/2.0;
      r(2) = lz/2.0;
      FixedRelativeFrame *frame = new FixedRelativeFrame("P1",r,SqrMat3(EYE),R);
      addFrame(frame);
      Point *point = new Point("Point1");
      point->setFrameOfReference(frame);
      addContour(point);

      r(0) = -lx/2.0;
      r(1) = ly/2.0;
      r(2) = lz/2.0;
      frame = new FixedRelativeFrame("P2",r,SqrMat3(EYE),R);
      addFrame(frame);
      point = new Point("Point2");
      point->setFrameOfReference(frame);
      addContour(point);
  
      r(0) = -lx/2.0;
      r(1) = -ly/2.0;
      r(2) = lz/2.0;
      frame = new FixedRelativeFrame("P3",r,SqrMat3(EYE),R);
      addFrame(frame);
      point = new Point("Point3");
      point->setFrameOfReference(frame);
      addContour(point);
  
      r(0) = lx/2.0;
      r(1) = -ly/2.0;
      r(2) = lz/2.0;
      frame = new FixedRelativeFrame("P4",r,SqrMat3(EYE),R);
      addFrame(frame);
      point = new Point("Point4");
      point->setFrameOfReference(frame);
      addContour(point);
  
      r(0) = lx/2.0;
      r(1) = ly/2.0;
      r(2) = -lz/2.0;
      frame = new FixedRelativeFrame("P5",r,SqrMat3(EYE),R);
      addFrame(frame);
      point = new Point("Point5");
      point->setFrameOfReference(frame);
      addContour(point);
  
      r(0) = -lx/2.0;
      r(1) = ly/2.0;
      r(2) = -lz/2.0;
      frame = new FixedRelativeFrame("P6",r,SqrMat3(EYE),R);
      addFrame(frame);
      point = new Point("Point6");
      point->setFrameOfReference(frame);
      addContour(point);
  
      r(0) = -lx/2.0;
      r(1) = -ly/2.0;
      r(2) = -lz/2.0;
      frame = new FixedRelativeFrame("P7",r,SqrMat3(EYE),R);
      addFrame(frame);
      point = new Point("Point7");
      point->setFrameOfReference(frame);
      addContour(point);
  
      r(0) = lx/2.0;
      r(1) = -ly/2.0;
      r(2) = -lz/2.0;
      frame = new FixedRelativeFrame("P8",r,SqrMat3(EYE),R);
      addFrame(frame);
      point = new Point("Point8");
      point->setFrameOfReference(frame);
      addContour(point);

      double thicknessFactor = 0.5;

      r(0) = -lx/2.0;
      r(1) = 0;
      r(2) = 0;
      frame = new FixedRelativeFrame("F1",r,BasicRotAIKz(M_PI),R);
      addFrame(frame);
      Plate *plate = new Plate("Face1");
      plate->setYLength(ly);
      plate->setZLength(lz);
      plate->setThickness(lx*thicknessFactor);
      plate->setFrameOfReference(frame);
      addContour(plate);

      r(0) = lx/2.0;
      r(1) = 0;
      r(2) = 0;
      frame = new FixedRelativeFrame("F2",r,BasicRotAIKz(0),R);
      addFrame(frame);
      plate = new Plate("Face2");
      plate->setYLength(ly);
      plate->setZLength(lz);
      plate->setThickness(lx*thicknessFactor);
      plate->setFrameOfReference(frame);
      addContour(plate);

      r(0) = 0;
      r(1) = 0;
      r(2) = -lz/2.0;
      frame = new FixedRelativeFrame("F3",r,BasicRotAIKy(M_PI/2),R);
      addFrame(frame);
      plate = new Plate("Face3");
      plate->setYLength(ly);
      plate->setZLength(lx);
      plate->setThickness(lz*thicknessFactor);
      plate->setFrameOfReference(frame);
      addContour(plate);

      r(0) = 0;
      r(1) = 0;
      r(2) = lz/2.0;
      frame = new FixedRelativeFrame("F4",r,BasicRotAIKy(-M_PI/2),R);
      addFrame(frame);
      plate = new Plate("Face4");
      plate->setYLength(ly);
      plate->setZLength(lx);
      plate->setThickness(lz*thicknessFactor);
      plate->setFrameOfReference(frame);
      addContour(plate);

      r(0) = 0;
      r(1) = -ly/2.0;
      r(2) = 0;
      frame = new FixedRelativeFrame("F5",r,BasicRotAIKz(-M_PI/2),R);
      addFrame(frame);
      plate = new Plate("Face5");
      plate->setYLength(lx);
      plate->setZLength(lz);
      plate->setThickness(ly*thicknessFactor);
      plate->setFrameOfReference(frame);
      addContour(plate);

      r(0) = 0;
      r(1) = ly/2.0;
      r(2) = 0;
      frame = new FixedRelativeFrame("F6",r,BasicRotAIKz(M_PI/2),R);
      addFrame(frame);
      plate = new Plate("Face6");
      plate->setYLength(lx);
      plate->setZLength(lz);
      plate->setThickness(ly*thicknessFactor);
      plate->setFrameOfReference(frame);
      addContour(plate);

      r(0) = 0;
      r(1) = +ly/2.0;
      r(2) = -lz/2.0;
      frame = new FixedRelativeFrame("E1",r,BasicRotAIKz(-M_PI/2)*BasicRotAIKy(3*M_PI/4),R);
      addFrame(frame);
      Edge *edge = new Edge("Edge1");
      edge->setLength(lx);
      edge->setThickness(min(ly, lz)*thicknessFactor);
      edge->setFrameOfReference(frame);
      addContour(edge);

      r(0) = +lx/2.0;
      r(1) = -ly/2.0;
      r(2) = 0;
      frame = new FixedRelativeFrame("E2",r,BasicRotAIKx(M_PI/2)*BasicRotAIKy(-M_PI/4),R);
      addFrame(frame);
      edge = new Edge("Edge2");
      edge->setLength(lz);
      edge->setThickness(min(lx, ly)*thicknessFactor);
      edge->setFrameOfReference(frame);
      addContour(edge);

      r(0) = 0;
      r(1) = -ly/2.0;
      r(2) = +lz/2.0;
      frame = new FixedRelativeFrame("E3",r,BasicRotAIKz(M_PI/2)*BasicRotAIKy(5*M_PI/4),R);
      addFrame(frame);
      edge = new Edge("Edge3");
      edge->setLength(lx);
      edge->setThickness(min(ly, lz)*thicknessFactor);
      edge->setFrameOfReference(frame);
      addContour(edge);

      r(0) = -lx/2.0;
      r(1) = 0;
      r(2) = +lz/2.0;
      frame = new FixedRelativeFrame("E4",r,BasicRotAIKy(5*M_PI/4),R);
      addFrame(frame);
      edge = new Edge("Edge4");
      edge->setLength(ly);
      edge->setThickness(min(lx, lz)*thicknessFactor);
      edge->setFrameOfReference(frame);
      addContour(edge);

      r(0) = 0;
      r(1) = +ly/2.0;
      r(2) = +lz/2.0;
      frame = new FixedRelativeFrame("E5",r,BasicRotAIKz(-M_PI/2)*BasicRotAIKy(5*M_PI/4),R);
      addFrame(frame);
      edge = new Edge("Edge5");
      edge->setLength(lx);
      edge->setThickness(min(ly, lz)*thicknessFactor);
      edge->setFrameOfReference(frame);
      addContour(edge);

      r(0) = -lx/2.0;
      r(1) = +ly/2.0;
      r(2) = 0;
      frame = new FixedRelativeFrame("E6",r,BasicRotAIKx(-M_PI/2)*BasicRotAIKy(5*M_PI/4),R);
      addFrame(frame);
      edge = new Edge("Edge6");
      edge->setLength(lz);
      edge->setThickness(min(lx, ly)*thicknessFactor);
      edge->setFrameOfReference(frame);
      addContour(edge);

      r(0) = +lx/2.0;
      r(1) = +ly/2.0;
      r(2) = 0;
      frame = new FixedRelativeFrame("E7",r,BasicRotAIKx(M_PI/2)*BasicRotAIKy(M_PI/4),R);
      addFrame(frame);
      edge = new Edge("Edge7");
      edge->setLength(lz);
      edge->setThickness(min(lx, ly)*thicknessFactor);
      edge->setFrameOfReference(frame);
      addContour(edge);

      r(0) = +lx/2.0;
      r(1) = 0;
      r(2) = +lz/2.0;
      frame = new FixedRelativeFrame("E8",r,BasicRotAIKy(-M_PI/4),R);
      addFrame(frame);
      edge = new Edge("Edge8");
      edge->setLength(ly);
      edge->setThickness(min(lx, lz)*thicknessFactor);
      edge->setFrameOfReference(frame);
      addContour(edge);

      r(0) = +lx/2.0;
      r(1) = 0;
      r(2) = -lz/2.0;
      frame = new FixedRelativeFrame("E9",r,BasicRotAIKy(M_PI/4),R);
      addFrame(frame);
      edge = new Edge("Edge9");
      edge->setLength(ly);
      edge->setThickness(min(lx, lz)*thicknessFactor);
      edge->setFrameOfReference(frame);
      addContour(edge);

      r(0) = 0;
      r(1) = -ly/2.0;
      r(2) = -lz/2.0;
      frame = new FixedRelativeFrame("E10",r,BasicRotAIKz(M_PI/2)*BasicRotAIKy(3*M_PI/4),R);
      addFrame(frame);
      edge = new Edge("Edge10");
      edge->setLength(lx);
      edge->setThickness(min(ly, lz)*thicknessFactor);
      edge->setFrameOfReference(frame);
      addContour(edge);

      r(0) = -lx/2.0;
      r(1) = -ly/2.0;
      r(2) = 0;
      frame = new FixedRelativeFrame("E11",r,BasicRotAIKx(-M_PI/2)*BasicRotAIKy(3*M_PI/4),R);
      addFrame(frame);
      edge = new Edge("Edge11");
      edge->setLength(lz);
      edge->setThickness(min(lx, ly)*thicknessFactor);
      edge->setFrameOfReference(frame);
      addContour(edge);

      r(0) = -lx/2.0;
      r(1) = 0;
      r(2) = -lz/2.0;
      frame = new FixedRelativeFrame("E12",r,BasicRotAIKy(3*M_PI/4),R);
      addFrame(frame);
      edge = new Edge("Edge12");
      edge->setLength(ly);
      edge->setThickness(min(lx, lz)*thicknessFactor);
      edge->setFrameOfReference(frame);
      addContour(edge);
    }
    else if (stage == plotting) {
      if(openMBVRigidBody)
        static_pointer_cast<OpenMBV::Cuboid>(openMBVRigidBody)->setLength(lx,ly,lz);
    }
    CompoundContour::init(stage, config);
  }

  void Cuboid::initializeUsingXML(DOMElement *element) {
    CompoundContour::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"length");
    setLength(E(e)->getText<Vec3>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      DOMElement *d, *t;
      d=E(e)->getFirstElementChildNamed(MBSIM%"diffuseColor");
      t=E(e)->getFirstElementChildNamed(MBSIM%"transparency");
      if( d &&  t) enableOpenMBV(_diffuseColor=E(d)->getText<Vec3>(), _transparency=E(e)->getText<double>());
      if(!d &&  t) enableOpenMBV(                          _transparency=E(e)->getText<double>());
      if( d && !t) enableOpenMBV(_diffuseColor=E(d)->getText<Vec3>()                                       );
      if(!d && !t) enableOpenMBV(                                                                          );
    }
  }

}
