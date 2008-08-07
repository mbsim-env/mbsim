/* Copyright (C) 2004-2008  Martin Förg
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _SUBSYSTEM_H_
#define _SUBSYSTEM_H_

#include "object.h"
#include "hitsphere_link.h"

namespace MBSim {
  class CoordinateSystem;
  class ExtraDynamicInterface;
  class DataInterfaceBase;

  class Subsystem : public Object {

    friend class HitSphereLink;

    protected:
      vector<Object*> object;
      vector<Link*> link;
      vector<ExtraDynamicInterface*> EDI;
      vector<DataInterfaceBase*> DIB;
      vector<Subsystem*> subsystem;
      vector<Link*> linkSingleValued;
      vector<Link*> linkSetValued;
      vector<Link*> linkSetValuedActive;
      vector<HitSphereLink*> HSLink;

      int gSize;
      int gInd;
      Vec la;
      Vec dla;
      Vec s;
      Vec g, gd;
      int laSize, laInd;
      Vec res;
      Vec rFactor;
      int rFactorSize, rFactorInd;

      int svSize;
      int svInd;
      Vec sv;
      Vector<int> jsv;

      int nHSLinkSetValuedFixed;
      int nHSLinkSingleValuedFixed;
      Index Ig, Ila;
 
   public:
      /*! Constructor */
      Subsystem(const string &name);
      /*! Destructor */
      ~Subsystem();

      void init();
      void initz();
      void calcSize();
      void calchSize();
      void calclaSize();
      void checkActiveConstraints();

      Vec& getsv() {return sv;}
      const Vec& getsv() const {return sv;}
      Vector<int>& getjsv() {return jsv;}
      const Vector<int>& getjsv() const {return jsv;}
      const Vec& getla() const {return la;}
      Vec& getla() {return la;}
      const Vec& gets() const {return s;}
      Vec& gets() {return s;}
      const Vec& getres() const {return res;}
      Vec& getres() {return res;}
      const Vec& getg() const {return g;}
      Vec& getg() {return g;}
      const Vec& getgd() const {return gd;}
      Vec& getgd() {return gd;}
      const Vec& getrFactor() const {return rFactor;}
      Vec& getrFactor() {return rFactor;}

      void setgInd(int gInd_) {gInd = gInd_;Ig=Index(gInd,gInd+gSize-1);} 
      void setlaInd(int laInd_) {laInd = laInd_;Ila=Index(laInd,laInd+laSize-1); } 
      void setrFactorInd(int rFactorInd_) {rFactorInd = rFactorInd_; } 
      void setsvInd(int svInd_) {svInd = svInd_;};
      int getgSize() const {return gSize;} 
      int getlaSize() const {return laSize;} 
      int getlaInd() const {return laInd;} 
      int getrFactorSize() const {return rFactorSize;} 
      int getxSize() const {return xSize;}
      int getsvSize() const {return svSize;}

      void initPlotFiles();
      void plot(double t, double dt=1);
      void plotParameters();
      void closePlotFiles();

      void updateKinematics(double t);
      void updateLinksStage1(double t);
      void updateLinksStage2(double t);
      void updateT(double t); 
      void updateh(double t); 
      void updateM(double t); 
      void updateW(double t); 
      void updateG(double t); 
      void updateb(double t); 
      void updater(double t); 
      void updateStopVector(double t); 
      void updatedq(double t, double dt); 
      void updatedx(double t, double dt); 
      void updateqRef(); 
      void updateqdRef(); 
      void updateuRef(); 
      void updateudRef(); 
      void updatexRef(); 
      void updatexdRef(); 
      void updatezRef(); 
      void updatezdRef(); 
      void updatehRef();
      void updatefRef();
      void updaterRef();
      void updateTRef();
      void updateMRef();
      void updateLLMRef();
      void updatesvRef();
      void updatejsvRef();
      void updategRef();
      void updateRef();
      void updateWRef();
      void updatelaRef();
      void updategdRef();
      void updatebRef();
      void updatesRef();
      void updateresRef();
      void updaterFactorRef();

      using Object::addCoordinateSystem;
      using Object::addContour;

      //void addContour(Contour* contour);
      //void addCoordinateSystem(CoordinateSystem* port);
      //CoordinateSystem* getCoordinateSystem(const string &name, bool check=true);
      //Contour* getContour(const string &name, bool check);

      void addCoordinateSystem(CoordinateSystem *port_, const Vec &RrRK, const SqrMat &ARK, const CoordinateSystem* refCoordinateSystem=0); 

      void addCoordinateSystem(const string &str, const Vec &SrSK, const SqrMat &ASK, const CoordinateSystem* refCoordinateSystem=0);

      void addContour(Contour* contour, const Vec &RrRC, const SqrMat &ARC, const CoordinateSystem* refCoordinateSystem=0);

      void addSubsystem(Subsystem *subsystem, const Vec &RrRC, const SqrMat &ARC, const CoordinateSystem* refCoordinateSystem=0);

      void addObject(Object *object);
      void addLink(Link *connection);
      Object* getObject(const string &name,bool check=true);
      Link* getLink(const string &name,bool check=true);
      /*! Returns an extra dynamic interface */
      ExtraDynamicInterface* getEDI(const string &name,bool check=true);
      DataInterfaceBase* getDataInterfaceBase(const string &name, bool check=true);
      /*! Adds an \param object to multibody system */
      void addEDI(ExtraDynamicInterface *edi_);
      /* Add a data_interface_base \param dib_ to the DataInterfaceBase-vector */
      void addDataInterfaceBase(DataInterfaceBase* dib_);

      virtual const Vec& getAccelerationOfGravity() const {return parent->getAccelerationOfGravity();}

      virtual const SymMat& getG() const {return parent->getG();}
      virtual SymMat& getG() {return parent->getG();}
      virtual const Vec& getlaMBS() const {return parent->getlaMBS();}
      virtual Vec& getlaMBS() {return parent->getlaMBS();}
      virtual const Matrix<Sparse, double>& getGs() const {return parent->getGs();}
      virtual Matrix<Sparse, double>& getGs() {return parent->getGs();}
      virtual const SqrMat& getJprox() const {return parent->getJprox();}
      virtual SqrMat& getJprox() {return parent->getJprox();}
      virtual void setTermination(bool term) {parent->setTermination(term);}

      virtual HitSphereLink* getHitSphereLink(Object* obj0, Object* obj1);
      virtual void setActiveConstraintsChanged(bool b) {parent->setActiveConstraintsChanged(b);}


      virtual int solveFixpointSingle(double dt);
      virtual void checkForTermination(double dt);
      virtual void updaterFactors();

      virtual int getlaIndMBS() const {return parent->getlaIndMBS() + laInd;}

  };
}

#endif

