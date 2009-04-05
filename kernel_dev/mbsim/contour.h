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
 *          rzander@users.berlios.de
 */

#ifndef _CONTOUR_H_
#define _CONTOUR_H_

#include <mbsim/flexible_body.h>
#include <mbsim/element.h>
#include <mbsim/userfunction_contour.h>
#include <mbsim/contour_pdata.h>
#include <mbsim/frame.h>

#ifdef HAVE_AMVIS
namespace AMVis {class CBody; class CRigidBody;}
#endif
#ifdef HAVE_AMVISCPPINTERFACE
#include <amviscppinterface/rigidbody.h>
#endif

namespace MBSim {

  class Object;

  // perhaps helpfull when debugging
  //                0    , 1   , 2          , 3           , 4        , 5    , 6     , 7      , 8   , 9   , 10       , 11       , 12              , 13
  enum ContourType { point, line, circlesolid, circlehollow, frustum2D, plane, sphere, frustum, area, edge, contour1s, contour2d, cylinderflexible, interpolation };

  /** 
   * \brief basic class for contour definition for rigid (which do not know about their shape) and flexible (they know how they look like) bodies
   * \author Martin Foerg
   * \date 2009-03-23 some comments (Thorsten Schindler)
   *
   * kinematics is stored in coordinate system class and is individually evaluated in specific contact kinematics 
   */
  class Contour : public Element { 
    public:
      /** 
       * \brief constructor
       * \param name of contour
       */
      Contour(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Contour();	

      virtual void preinit() {}
      virtual void init();

      virtual void initPlot();
      virtual void plot(double t, double dt = 1); 

      int gethSize(int i=0) const { return hSize[i]; }
      int gethInd(int i=0) const { return hInd[i]; }
      void sethSize(int size, int i=0) { hSize[i] = size; }
      void sethInd(int ind, int i=0) { hInd[i] = ind; }

      ObjectInterface* getParent() { return parent; }
      void setParent(ObjectInterface* parent_) { parent = parent_; }

      /**
       * \return whole kinematics of contour
       */
      Frame* getFrame() { return &R; }

      /**
       * \param position of contour in inertial frame
       */
      virtual void setWrOP(const fmatvec::Vec &WrOP) { R.setPosition(WrOP); }

      /**
       * \return position of contour in inertial frame
       */
      const fmatvec::Vec& getWrOP() const { return R.getPosition(); }

      /**
       * \param velocity of contour in inertial frame
       */
      virtual void setWvP(const fmatvec::Vec &WvP) { R.setVelocity(WvP); }

      /**
       * \return velocity of contour in inertial frame
       */
      const fmatvec::Vec& getWvP() const { return R.getVelocity(); }

      /**
       * \param angular velocity of contour in inertial frame
       */
      virtual void setWomegaC(const fmatvec::Vec &WomegaC) { R.setAngularVelocity(WomegaC); }

      /**
       * \return angular velocity of contour in inertial frame
       */
      const fmatvec::Vec& getWomegaC() const { return R.getAngularVelocity(); }

      /**
       * \param orientation of contour to inertial frame
       */
      virtual void setAWC(const fmatvec::SqrMat &AWC) { R.setOrientation(AWC); }

      /**
       * \return orientation of contour to inertial frame
       */
      const fmatvec::SqrMat& getAWC() const { return R.getOrientation(); }

      virtual void setWJP(const fmatvec::Mat &WJP) { R.setJacobianOfTranslation(WJP); }
      virtual void setWjP(const fmatvec::Vec &WjP) { R.setGyroscopicAccelerationOfTranslation(WjP); }
      virtual void setWJR(const fmatvec::Mat &WJR) { R.setJacobianOfRotation(WJR); }
      virtual void setWjR(const fmatvec::Vec &WjR) { R.setGyroscopicAccelerationOfRotation(WjR); }
      const fmatvec::Mat& getWJP() const { return R.getJacobianOfTranslation(); }
      const fmatvec::Mat& getWJR() const { return R.getJacobianOfRotation(); }
      fmatvec::Mat& getWJP() { return R.getJacobianOfTranslation(); }
      fmatvec::Mat& getWJR() { return R.getJacobianOfRotation(); }
      const fmatvec::Vec& getWjP() const { return R.getGyroscopicAccelerationOfTranslation(); }
      const fmatvec::Vec& getWjR() const { return R.getGyroscopicAccelerationOfRotation(); }
      fmatvec::Vec& getWjP() { return R.getGyroscopicAccelerationOfTranslation(); }
      fmatvec::Vec& getWjR() { return R.getGyroscopicAccelerationOfRotation(); }

      void resizeJacobians(int j);

      // /*! adjust HitSphere of parent body
      // */
      // virtual void adjustParentHitSphere(const fmatvec::Vec &CrC);

      /**
       * return std::string representation
       */
      std::string getType() const { return "Contour"; }

#ifdef HAVE_AMVIS
      /**
       * \brief activate output for AMVis
       * \param binary or ASCII data format in pos-file
       */
      void createAMVisBody(bool binary_=false) {boolAMVis = true; boolAMVisBinary = binary_;}

      void setAMVisBody(AMVis::CRigidBody *AMVisBody, DataInterfaceBase *funcColor=NULL);
#endif

    protected:
      ObjectInterface* parent;

      int hSize[2], hInd[2];

      /**
       * coordinate system of contour
       */
      Frame R;


#ifdef HAVE_AMVIS
      /**
       * \brief body for AMVis
       */
      AMVis::CBody *bodyAMVis;

      DataInterfaceBase* bodyAMVisUserFunctionColor;

      /**
       * \brief flag to allow for activation of AMVis output during init-routines
       */
      bool boolAMVis, boolAMVisBinary;
#endif

#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::RigidBody *amvisRigidBody;
#endif
  };

  /**
   * \brief most primitive contour: the point (no extention)
   * \author Martin Foerg
   * \date 2009-03-19 comments (Thorsten Schindler)
   */
  class Point : public Contour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Point(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Point();

      /* INHERITED INTERFACE */
      std::string getType() const { return "Point"; }
  };

  /*! \brief unbounded line with constant normal */
  class Line : public Contour {

    protected:
    public:
      Line(const std::string &name);
      ~Line();

      std::string getType() const {return "Line";}

      void load(const std::string& path, std::ifstream &inputfile); 
      void save(const std::string &path, std::ofstream &outputfile);
  };

  /*! \brief Circular Contour with material included inside */
  class CircleSolid : public Contour {

    private:
      double r;
      fmatvec::Vec Cb;

    public:
      /*! Constructor */
      CircleSolid(const std::string &name);
      /*! Constructor */
      CircleSolid(const std::string &name, double r);
      /*! Set radius */
      void setRadius(double r_) {r = r_;}
      /*! Get radius */
      double getRadius() const {return r;}
      /*! Set binormal in local FR */ 
      void setCb(const fmatvec::Vec& Cb_);
      /*! Compute binormal in inertial FR */
      fmatvec::Vec computeWb() {return R.getOrientation()*Cb;}

#ifdef HAVE_AMVISCPPINTERFACE
      void enableAMVis(bool enable=true);
#endif
  };


  /*! \brief circular Contour describing hole in material */
  class CircleHollow : public Contour {

    protected:
      double r;
      fmatvec::Vec Cb;

    public:
      CircleHollow(const std::string &name);
      void setRadius(double r_) {r = r_;}
      double getRadius() const {return r;}

      void setCb(const fmatvec::Vec& Cb_);

      fmatvec::Vec computeWb() {return R.getOrientation()*Cb;}
  };

  /*! \brief Planar slice of a Frustum
   *
   * */
  class Frustum2D : public Contour {
    protected:
      /** Normalenvektor der Kontur **/
      fmatvec::Vec Ka;
      fmatvec::Vec r;
      double h;
      fmatvec::Vec Cb;
      /** Haltepunkt der Kontur **/
    public:
      Frustum2D(const std::string &name);
      void setRadii(const fmatvec::Vec &r_) {r = r_;}
      const fmatvec::Vec& getRadii() const {return r;} 
      void setAxis(const fmatvec::Vec &a);
      const fmatvec::Vec& getAxis() const {return Ka;} 
      void setHeight(double h_) {h = h_;}
      double getHeight() const {return h;} 

      const fmatvec::Vec& getCb() const {return Cb;} 
      void setCb(const fmatvec::Vec& Cb_);

      fmatvec::Vec computeWb() {return R.getOrientation()*Cb;}

  };

  /*! \brief Parent for contours described by one contour parameter \f$s\f$ */
  class Contour1s : public Contour {
    protected:
      double as, ae;
      double width;
      fmatvec::Vec nodes;
      fmatvec::Vec Cb;
    public:
      Contour1s(const std::string &name);

      virtual fmatvec::Vec computeWn(double alpha) = 0;
      virtual fmatvec::Mat computeWt(double alpha) = 0;
      virtual fmatvec::Vec computeWb(double alpha) = 0;
      virtual fmatvec::Vec computeWrOC(double alpha) = 0;
      virtual fmatvec::Vec computeWvC(double alpha) = 0;
      virtual fmatvec::Vec computeWomega(double alpha) = 0;
      virtual double computeRadius(double alpha) = 0;

      fmatvec::Mat computeWt  (const ContourPointData &cp)   ;// {return computeWt  (cp.alpha(0));}
      fmatvec::Vec computeWn  (const ContourPointData &cp)   ;// {return computeWn  (cp.alpha(0));}
      fmatvec::Vec computeWb  (const ContourPointData &cp)   ;// {return computeWb  (cp.alpha(0));}
      fmatvec::Vec computeWrOC(const ContourPointData &cp)   ;// {return computeWrOC(cp.alpha(0));}
      fmatvec::Vec computeWvC (const ContourPointData &cp)   ;// {return computeWvC (cp.alpha(0));}
      fmatvec::Vec computeWomega(const ContourPointData &cp) ;// {return computeWomega(cp.alpha(0));}

      void setCb(const fmatvec::Vec& Cb);

      void   setWidth(double width_u) {width= width_u;}
      double getWidth() {return width;}

      void setAlphaStart(double as_) {as = as_;}
      void setAlphaEnd(double ae_) {ae = ae_;}
      double getAlphaStart() const {return as;}
      double getAlphaEnd() const {return ae;}
      void setNodes(const fmatvec::Vec &nodes_) {nodes = nodes_;}
      const fmatvec::Vec& getNodes() const {return nodes;}
  };

  /*! \brief analytical description of contours */
  class Contour1sAnalytical : public Contour1s {
    protected:
      UserFunctionContour1s  *funcCrPC;
    public:
      Contour1sAnalytical(const std::string &name);
      virtual ~Contour1sAnalytical();

      /*! \todo Cache !!!
      */
      fmatvec::Mat computeWt(double alpha) {
        Cb= funcCrPC->computeB(alpha);
        fmatvec::Mat Ctb(3,2);
        Ctb.col(0)= funcCrPC->computeT(alpha);
        Ctb.col(1)= Cb; 
        return R.getOrientation()*Ctb;
      }
      fmatvec::Vec computeWn(double alpha) {return R.getOrientation()*(funcCrPC->computeN(alpha));}
      fmatvec::Vec computeWb(double alpha) {Cb= funcCrPC->computeB(alpha); return R.getOrientation()*Cb;}
      fmatvec::Vec computeWrOC(double alpha) {return R.getPosition() + R.getOrientation()*(*funcCrPC)(alpha);}
      fmatvec::Vec computeWvC(double alpha) {return R.getVelocity() + crossProduct(R.getAngularVelocity(),R.getOrientation()*(*funcCrPC)(alpha));}
      fmatvec::Vec computeWomega(double alpha) {return fmatvec::Vec(3);} // dummy

      double computeRadius(double alpha) {
        return funcCrPC->computeR(alpha);
      }
      void setUserFunction(UserFunctionContour1s* f) {funcCrPC = f;}
      UserFunctionContour1s* getUserFunction() {return funcCrPC;}

      virtual void init();
  };

  /** 
   * \brief flexible neutral fiber for one dimensional flexible bodies
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-03-18 initial comment (Thorsten Schindler)
   * \date 2009-04-05 adapted to non-template FlexibleBody (Schindler / Zander)
   */
  class Contour1sFlexible : public Contour1s {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour1sFlexible(const std::string &name) : Contour1s(name) {}

      /**
       * \brief destructor
       */
      virtual ~Contour1sFlexible() {}

      /* INHERITED INTERFACE */
      fmatvec::Vec computeWb(double s) { throw(1); } // TODO
      fmatvec::Vec computeWn(double alpha) { throw(1); }
      fmatvec::Mat computeWt(double alpha) { throw(1); }
      fmatvec::Vec computeWrOC(double alpha) { throw(1); }
      fmatvec::Vec computeWvC(double alpha) { throw(1); }
      fmatvec::Vec computeWomega(double alpha) { throw(1); }
      double computeRadius(double alpha) { throw(1); }

      void updateKinematicsForFrame(ContourPointData &cp, Frame *frame=0) { parent->updateKinematicsForFrame(cp,frame); }

    protected:
      FlexibleBody* parent;
  };

  /** 
   * \brief flexible cylinder for one dimensional flexible bodies
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 18.03.09
   */
  class CylinderFlexible : public Contour1sFlexible {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      CylinderFlexible(const std::string &name) : Contour1sFlexible(name) {}

      /**
       * \brief destructor
       */
      virtual ~CylinderFlexible() {}

      /* GETTER / SETTER */
      void setRadius(double r_) { r = r_; }
      double getRadius() const  { return r; }

    protected:
      /**
       * \brief radius
       */
      double r;
  };

  /** 
   * \brief plane without borders
   * \author Martin Foerg
   * \date 09.03.09 some comments (Thorsten Schindler)
   *
   * normal equals first column in orientation matrix
   */
  class Plane : public Contour {
    public:
      /**
       * \brief constructor
       */
      Plane(const std::string &name);

      /** 
       * \return normal of the plane in local frame of reference
       * TODO obsolete
       */
      const fmatvec::Vec& getCn() const { return Cn; }

      /**
       * \param normal of the plane in local frame of reference
       * TODO obsolete
       */
      void setCn(const fmatvec::Vec& Cn_);

      /** 
       * \return normal of the plane in inertial frame of reference
       * TODO obsolete
       */
      fmatvec::Vec computeWn() {return R.getOrientation()*Cn;}

    private:
      /**
       * \brief normal of the plane in local frame of reference
       */
      fmatvec::Vec Cn;
  };

  /*! \brief Contour Area */
  class Area : public Contour {

    protected:
      double lim1, lim2;
      fmatvec::Vec Cn, Cd1, Cd2;

    public:
      Area(const std::string &name);

      void setLimit1(double l) {lim1 = l;}
      void setLimit2(double l) {lim2 = l;}
      void setCd1(const fmatvec::Vec& Cd);
      void setCd2(const fmatvec::Vec& Cd);
      virtual void init();
      double getLimit1() const {return lim1;}
      double getLimit2() const {return lim2;}

      fmatvec::Vec computeWn() {return R.getOrientation()*Cn;}
      fmatvec::Vec computeWd1() {return R.getOrientation()*Cd1;}
      fmatvec::Vec computeWd2() {return R.getOrientation()*Cd2;}
  };

  /*! \brief Contour Edge */
  class Edge : public Contour {

    protected:
      double lim;
      fmatvec::Vec Cn, Cd, Ce;

    public:
      Edge(const std::string &name);

      void setLimit(double l) {lim = l;}
      void setCd(const fmatvec::Vec& Cd);
      void setCe(const fmatvec::Vec& Ce);
      //void init();
      double getLimit() const {return lim;}

      // fmatvec::Vec computeWn() {return R.getOrientation()*Cn;}
      fmatvec::Vec computeWe() {return R.getOrientation()*Ce;}
      fmatvec::Vec computeWd() {return R.getOrientation()*Cd;}
  };

  /*! \brief Contour Sphere */
  class Sphere : public Contour {
    protected:
      /** radius */
      double r;

    public:
      Sphere(const std::string &name);
      void setRadius(double r_) {r = r_;}
      double getRadius() const {return r;}
      virtual void init();
#ifdef HAVE_AMVISCPPINTERFACE
      void enableAMVis(bool enable=true);
#endif
  };

  /*! \brief Frustum
   *
   *  Authors: Martin Foerg, Thorsten Schindler
   */
  class Frustum : public Contour {
    public:
      /*! Constructor with \default outCont=false */
      Frustum(const std::string &name);
      /*! Constructor */
      Frustum(const std::string &name, bool outCont_);

      /*! Deconstructor */
      ~Frustum();
      /*! Set Radii of the Frustum */
      void setRadii(const fmatvec::Vec &r_);
      /*! Get Radii of the Frustum */
      const fmatvec::Vec& getRadii() const;
      /*! Set Axis of the Frustum in contour FR */
      void setAxis(const fmatvec::Vec &a_);
      /*! Get Axis of the Frustum in contour FR */
      const fmatvec::Vec& getAxis() const; 
      /*! Set Height of the Frustum */
      void setHeight(double h_);
      /*! Get Height of the Frustum */
      double getHeight() const;
      /*! Set Contact on outer surface of the Frustum */
      void setOutCont(bool outCont_);
      /*! Get Contact outcont on outer surface of the Frustum */
      bool getOutCont() const;

    private:
      /** axis (contour FR) */
      fmatvec::Vec a;

      /** upper r(1) and lower radius r(0) in direction of the axis */
      fmatvec::Vec r;

      /** height */
      double h;

      /** contact on outer or inner surface? */
      bool outCont;
  };

  inline void Frustum::setRadii(const fmatvec::Vec &r_) {r = r_;}
  inline const fmatvec::Vec& Frustum::getRadii() const {return r;}
  inline void Frustum::setAxis(const fmatvec::Vec &a_) {a = a_/nrm2(a_);}
  inline const fmatvec::Vec& Frustum::getAxis() const {return a;}
  inline void Frustum::setHeight(double h_) {h = h_;}
  inline double Frustum::getHeight() const {return h;}
  inline void Frustum::setOutCont(bool outCont_) {outCont = outCont_;}
  inline bool Frustum::getOutCont() const {return outCont;}

  /*! \brief Parent for contours described by two contour parameters \f$\vs\f$
   *
   * */
  class Contour2s : public Contour {
    protected:
      fmatvec::Vec as, ae;
      fmatvec::Mat nodes;
      fmatvec::Vec Cb;

    public:
      Contour2s(const std::string &name);

      fmatvec::Vec computeWn(const fmatvec::Vec& alpha) {
        fmatvec::Mat Wt = computeWt(alpha);
        if(Cb(2) < 0.0 ) return   crossProduct(Wt.col(0),Wt.col(1));
        else             return - crossProduct(Wt.col(0),Wt.col(1));
      }
      virtual fmatvec::Mat computeWt  (const fmatvec::Vec& alpha) = 0;
      virtual fmatvec::Vec computeWrOC(const fmatvec::Vec& alpha) = 0;
      virtual fmatvec::Vec computeWvC (const fmatvec::Vec& alpha) = 0;

      //    void setCb(const fmatvec::Vec& Cb);

      void setAlphaStart(fmatvec::Vec as_) {as = as_;}
      void setAlphaEnd(fmatvec::Vec ae_) {ae = ae_;}
      fmatvec::Vec getAlphaStart() const {return as;}
      fmatvec::Vec getAlphaEnd() const {return ae;}

      void setNodes(const fmatvec::Mat &nodes_) {nodes = nodes_;}
      const fmatvec::Mat& getNodes() const {return nodes;}
  };

  /*! \brief Basis-Class for Contour interpolation between Point s, standard contact Point-ContourInterpolation is implemented
    special interpolations only need to provide (as derived class) the pure virtuals predefined here
    */
  class ContourInterpolation : public Contour {
    protected:
      /** list of Point s holding ContourInterpolation */
      std::vector<Point*> iPoints;
      /** number of Contour-parameters used by ContourInterpolation: 1 for lines, 2 for surfaces */
      int contourParameters;
      /** size of iPoints, number of Point s used for interpolation */
      int numberOfPoints;

    public:
      ContourInterpolation(const std::string &name, int parameters_, int nPoints_);

      //void plot(double t, double dt);

      //    Object* getObject() {return iPoints[0]->getObject();} // ACHTUNG: das kann in die Hose gehen, wenn iPoints noch nicht initzialisiert...

      /*! set Point for interpolation
        \param pointN Point to use
        \param position in iPoints, Point-number
        */
      void setPoint(Point *pointN, int n);
      /*! get list of Point s */
      std::vector<Point*> getPoints()   const {return iPoints;}
      Point* getPoint(const int n) const {return iPoints[n];}

      /*! get number of Point s used for interpolation */
      int getNPoints() const {return numberOfPoints;}
      /*! get number of Contour-parameters of Contour */
      int getNContourParameters() const {return contourParameters;}

      /*! prototype for test if Contour-point given is inside or outside defined contour area
        \param cp Contour-point
        \return true, if cp is inside boundaries, else false
        */ 
      virtual bool testInsideBounds(const ContourPointData &cp) = 0;

      /*! prototype of method giving weights of all Point s 
        \param s Contour-parameter(s)
        \param i Point number
        \return weight of Point i at s
        */
      virtual double computePointWeight(const fmatvec::Vec &s, int i) = 0;
      /*! prototype of method giving first derivatives with respect to the diff-th Contour-parameters of all Point s 
        \param s Contour-parameter(s)
        \param i Point number
        \param diff -th derivative
        \return weight/derivative of Point i at s
        */
      virtual double computePointWeight(const fmatvec::Vec &s, int i, int diff) = 0;

      /*! compute all weights for nodes */
      fmatvec::Vec computePointWeights(const fmatvec::Vec &s);

      fmatvec::Vec computeWrOC(const fmatvec::Vec& s);// {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWrOC(cp);};
      fmatvec::Vec computeWvC (const fmatvec::Vec& s);// {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWvC (cp);};
      fmatvec::Mat computeWt  (const fmatvec::Vec& s);// {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWt  (cp);};
      fmatvec::Vec computeWn  (const fmatvec::Vec& s);// {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWn  (cp);};

      fmatvec::Vec computeWrOC(const ContourPointData &cp);
      fmatvec::Vec computeWvC (const ContourPointData &cp);

      fmatvec::Mat computeWt  (const ContourPointData &cp);
      virtual fmatvec::Vec computeWn  (const ContourPointData &cp) = 0;
  };

  /*! \brief Quad for 3D interpolation  \see{OpenGL-documentation}
  */

  class ContourQuad : public ContourInterpolation {
    protected:
    public:
      ContourQuad(const std::string & name);

      virtual void init();

      bool testInsideBounds(const ContourPointData &cp);
      double computePointWeight(const fmatvec::Vec &s, int i);
      double computePointWeight(const fmatvec::Vec &s, int i, int diff);

      fmatvec::Vec computeWn(const ContourPointData &cp);
  };

  /*! \brief Contour consisting of primitive contour elements */
  class CompoundContour : public Contour {
    private:
      std::vector<Contour*> element;
      std::vector<fmatvec::Vec> Kr, Wr;
    public:
      /*! Constructor */
      CompoundContour(const std::string &name);
      void init();
      Contour* getContourElement(int i) {return element[i];}
      void addContourElement(Contour* ce, const fmatvec::Vec& re);
      unsigned int getNumberOfElements() {return element.size();}
      void setWrOP(const fmatvec::Vec &WrOP);
      void setWvP(const fmatvec::Vec &WvP);
      void setWomegaC(const fmatvec::Vec &WomegaC);
      void setAWC(const fmatvec::SqrMat &AWC);
      void setWJP(const fmatvec::Mat &WJP);
      void setWjP(const fmatvec::Vec &WjP);
      void setWJR(const fmatvec::Mat &WJR);
      void setWjR(const fmatvec::Vec &WjR);
  };


  /*! \brief Cuboid with 8 vertices, 12 edges and 6 faces */
  class Cuboid : public CompoundContour {
    private:
      double l,h,d;
    public:
      /*! Constructor */
      Cuboid(const std::string &name);
      void setLength(double l_) {l = l_;}
      void setHeight(double h_) {h = h_;}
      void setDepth(double d_) {d = d_;}
      void preinit();
  };

}

#endif /* _CONTOUR_H_ */
