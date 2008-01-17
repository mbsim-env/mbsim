/* Copyright (C) 2004-2006  Martin Förg, Roland Zander
 
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
 *   rzander@users.berlios.de
 *
 */

#ifndef _CONTOUR_H_
#define _CONTOUR_H_

#include "element.h"
#include "userfunction_contour.h"
#include "contour_pdata.h"

#ifdef HAVE_AMVIS
namespace AMVis {class CBody;}
#endif

namespace MBSim {

  class Object;
  struct ContourPointData;

  // perhaps helpfull when debugging
  //                0    , 1   , 2          , 3           , 4        , 5    , 6     , 7      , 8   , 9   , 10       , 11       , 12              , 13
  enum ContourType {point, line, circlesolid, circlehollow, frustum2D, plane, sphere, frustum, area, edge, contour1s, contour2d, cylinderflexible, interpolation};

  /*! 
   * \brief Basic class for contour definition for rigid (which do not know about their shape) 
   *  and flexible (they know how they look like) bodies
   */
  class Contour : public Element {
    protected:
      int id;
      Object* parent;
      Vec WrOP, WvP, WomegaC;
      SqrMat AWC;

#ifdef HAVE_AMVIS
      /* Body for AMVis */
      AMVis::CBody *bodyAMVis;
      /* Flag to allow for activation of AMVis output during Init-Routines */
      bool boolAMVis, boolAMVisBinary;
#endif

    public:
      //Contour(const string &name, ContourType type);
      Contour(const string &name);
      virtual ~Contour();	

      virtual void init();
      void initPlotFiles();

      /* geerbt */
      void plotParameters();
      /* geerbt */
      void plot(double t, double dt);

      Object* getObject() {return parent;}
      void setObject(Object* object) {parent = object;}
      void setID(int id_) {id = id_;}
      int getID() const {return id;}

      void setWrOP(const Vec &WrOP_) {WrOP = WrOP_;}
      const Vec& getWrOP() const {return WrOP;}
      void setWvP(const Vec &WvP_) {WvP = WvP_;}
      const Vec& getWvP() const {return WvP;}
      void setWomegaC(const Vec &WomegaC_) {WomegaC = WomegaC_;}
      const Vec& getWomegaC() const {return WomegaC;}
      void setAWC(const SqrMat &AWC_) {AWC = AWC_;}
      const SqrMat& getAWC() const {return AWC;}

      /*! adjust HitSphere of parent body
      */
      virtual void adjustParentHitSphere(const Vec &CrC);

#ifdef HAVE_AMVIS
      /*! activate output for AMVis
	\param binary_ for binary or ASCII data format in pos-file
	*/
      void createAMVisBody(bool binary_=false) {boolAMVis = true; boolAMVisBinary = binary_; plotLevel = 1;}
#endif
  };

  /*! \brief Most primitive Contour: the Point (no extention) */
  class Point : public Contour {
    public:
      /*! Constructor with \param name */
      Point(const string &name);
      /*! Deconstructor */
      ~Point();
      void init();
  };

  /*! \brief unbounded line with constant normal */
  class Line : public Contour {

    protected:
      Vec Cn, Cb;
    public:
      Line(const string &name);

      const Vec& getCb() const {return Cb;} 
      const Vec& getCn() const {return Cn;} 
      void setCn(const Vec& Cn_);
      void setCb(const Vec& Cb_);

      Vec computeWb() {return AWC*Cb;}
      Vec computeWn() {return AWC*Cn;}
      Vec computeWt() {return AWC*crossProduct(Cn,Cb);}
  };

  /*! \brief Circular Contour with material included inside */
  class CircleSolid : public Contour {

    private:
      double r;
      Vec Cb;

    public:
      /*! Constructor with \param name */
      CircleSolid(const string &name);
      /*! Set radius \param r_*/
      void setRadius(double r_) {r = r_;}
      /*! Get radius \result r */
      double getRadius() const {return r;}
	  /*! Set binormal \param Cb_ in local FR */ 
      void setCb(const Vec& Cb_);
	  /*! Compute binormal \return Wb in inertial FR */
      Vec computeWb() {return AWC*Cb;}
  };


  /*! \brief circular Contour describing hole in material */
  class CircleHollow : public Contour {

    protected:
      double r;
      Vec Cb;

    public:
      CircleHollow(const string &name);
      void setRadius(double r_) {r = r_;}
      double getRadius() const {return r;}

      void setCb(const Vec& Cb_);

      Vec computeWb() {return AWC*Cb;}
  };

  /*! \brief Planar slice of a Frustum
   *
   * */
  class Frustum2D : public Contour {
    protected:
      /** Normalenvektor der Kontur **/
      Vec Ka;
      Vec r;
      double h;
      Vec Cb;
      /** Haltepunkt der Kontur **/
    public:
      Frustum2D(const string &name);
      void setRadii(const Vec &r_) {r = r_;}
      const Vec& getRadii() const {return r;} 
      void setAxis(const Vec &a);
      const Vec& getAxis() const {return Ka;} 
      void setHeight(double h_) {h = h_;}
      double getHeight() const {return h;} 

      const Vec& getCb() const {return Cb;} 
      void setCb(const Vec& Cb_);

      Vec computeWb() {return AWC*Cb;}

  };

  /*! \brief Parent for contours described by one contour parameter \f$s\f$ */
  class Contour1s : public Contour {
    protected:
      double as, ae;
      double width;
      Vec nodes;
      Vec Cb;
    public:
      Contour1s(const string &name);

      virtual Vec computeWn(double alpha) = 0;
      virtual Mat computeWt(double alpha) = 0;
      virtual Vec computeWb(double alpha) = 0;
      virtual Vec computeWrOC(double alpha) = 0;
      virtual Vec computeWvC(double alpha) = 0;
      virtual Vec computeWomega(double alpha) = 0;
      virtual double computeRadius(double alpha) = 0;

      Mat computeWt  (const ContourPointData &cp)   ;// {return computeWt  (cp.alpha(0));}
      Vec computeWn  (const ContourPointData &cp)   ;// {return computeWn  (cp.alpha(0));}
      Vec computeWb  (const ContourPointData &cp)   ;// {return computeWb  (cp.alpha(0));}
      Vec computeWrOC(const ContourPointData &cp)   ;// {return computeWrOC(cp.alpha(0));}
      Vec computeWvC (const ContourPointData &cp)   ;// {return computeWvC (cp.alpha(0));}
      Vec computeWomega(const ContourPointData &cp) ;// {return computeWomega(cp.alpha(0));}

      void setCb(const Vec& Cb);

      void   setWidth(double width_u) {width= width_u;}
      double getWidth() {return width;}

      void setAlphaStart(double as_) {as = as_;}
      void setAlphaEnd(double ae_) {ae = ae_;}
      double getAlphaStart() const {return as;}
      double getAlphaEnd() const {return ae;}
      void setNodes(const Vec &nodes_) {nodes = nodes_;}
      const Vec& getNodes() const {return nodes;}
  };

  /*! \brief analytical description of contours */
  class Contour1sAnalytical : public Contour1s {
    protected:
      UserFunctionContour1s  *funcCrPC;
    public:
      Contour1sAnalytical(const string &name);
      virtual ~Contour1sAnalytical();

      /*! \todo Cache !!!
      */
      Mat computeWt(double alpha) {
	Cb= funcCrPC->computeB(alpha);
	Mat Ctb(3,2);
	Ctb.col(0)= funcCrPC->computeT(alpha);
	Ctb.col(1)= Cb; 
	return AWC*Ctb;
      }
      Vec computeWn(double alpha) {return AWC*(funcCrPC->computeN(alpha));}
      Vec computeWb(double alpha) {Cb= funcCrPC->computeB(alpha); return AWC*Cb;}
      Vec computeWrOC(double alpha) {return WrOP + AWC*(*funcCrPC)(alpha);}
      Vec computeWvC(double alpha) {return WvP + crossProduct(WomegaC,AWC*(*funcCrPC)(alpha));}
      Vec computeWomega(double alpha) {return Vec(3);} // dummy

      double computeRadius(double alpha) {
	return funcCrPC->computeR(alpha);
      }
      void setUserFunction(UserFunctionContour1s* f) {funcCrPC = f;}
      UserFunctionContour1s* getUserFunction() {return funcCrPC;}
      virtual void init();
  };

  /*! \brief 1s flexible */
  class Contour1sFlexible : public Contour1s {
    protected:

    public:
      Contour1sFlexible(const string &name);

      Vec computeWb  (double s) {return AWC*Cb;}

      Vec computeWn    (double alpha);
      Mat computeWt    (double alpha);
      Vec computeWrOC  (double alpha);
      Vec computeWvC   (double alpha);
      Vec computeWomega(double alpha);

      Mat computeWt  (const ContourPointData &cp);
      Vec computeWn  (const ContourPointData &cp);
      Vec computeWb  (const ContourPointData &cp);
      Vec computeWrOC(const ContourPointData &cp);
      Vec computeWvC (const ContourPointData &cp);

      Vec computeWomega(const ContourPointData &cp) {return Vec(3);}
      // TODO
      double computeRadius(double alpha) {return 1;}
  };

  /*! \brief Flexible cylinder for elastic 3D bending structure with one contour parameter
  */
  class CylinderFlexible : public Contour1sFlexible {
    protected:
      double r;

    public:
      CylinderFlexible(const string &name);
      void setRadius(double r_) {r = r_;}
      double getRadius() const  {return r;}

      /*     Mat computeWt    (double s); */
      Vec computeWn    (const ContourPointData& cp) {return Vec(3);}

      Vec computeWomega(const ContourPointData &cp);
  };

  /*! \brief Contour Plane without borders */
  class Plane : public Contour {
    private:
      Vec Cn;

    public:
      /*! Constructor with \param name */
      Plane(const string &name);
      /*! Get normal \return Cn of the Plane in local FR */
      const Vec& getCn() const {return Cn;}
      /*! Set normal \param Cn of the Plane in local FR */
      void setCn(const Vec& Cn_);
      /*! Compute normal \return AWC*Cn of the Plane in inertial FR */
      Vec computeWn() {return AWC*Cn;}
  };

  /*! \brief Contour Area */
  class Area : public Contour {

    protected:
      double lim1, lim2;
      Vec Cn, Cd1, Cd2;

    public:
      Area(const string &name);

      void setLimit1(double l) {lim1 = l;}
      void setLimit2(double l) {lim2 = l;}
      void setCd1(const Vec& Cd);
      void setCd2(const Vec& Cd);
      virtual void init();
      double getLimit1() const {return lim1;}
      double getLimit2() const {return lim2;}

      Vec computeWn() {return AWC*Cn;}
      Vec computeWd1() {return AWC*Cd1;}
      Vec computeWd2() {return AWC*Cd2;}
  };

  /*! \brief Contour Edge */
  class Edge : public Contour {

    protected:
      double lim;
      Vec Cn, Cd, Ce;

    public:
      Edge(const string &name);

      void setLimit(double l) {lim = l;}
      void setCd(const Vec& Cd);
      void setCe(const Vec& Ce);
      //void init();
      double getLimit() const {return lim;}

      // Vec computeWn() {return AWC*Cn;}
      Vec computeWe() {return AWC*Ce;}
      Vec computeWd() {return AWC*Cd;}
  };

  /*! \brief Contour Sphere */
  class Sphere : public Contour {
    protected:
      double r;
      /** Haltepunkt der Kontur **/
    public:
      Sphere(const string &name);
      void setRadius(double r_) {r = r_;}
      double getRadius() const {return r;}
      virtual void init();
  };

 /*! 
  * \brief Frustum of height h, axis a (contour FR) and in direction of the axis upper r(1) and lower radius r(0)
  *  Further it can be distinguished if the contact is on the outer or inner surface by outCont.
  *  Authors: Martin Foerg, Thorsten Schindler
  */
 class Frustum : public Contour {
	    private:
		      Vec a;
		      Vec r;
		      double h;
		      bool outCont;
		
	    public:
	    	  /*! Constructor with \param name and \default outCont=false */
		      Frustum(const string &name);
		      /*! Constructor with \param name and \param outCont_ */
		      Frustum(const string &name, bool outCont_);
		      /*! Deconstructor */
		      ~Frustum();
		      /*! Set Radii \param r_ of the Frustum */
		      void setRadii(const Vec &r_) {r = r_;}
		      /*! Get Radii \return r of the Frustum */
		      const Vec& getRadii() const {return r;}
		      /*! Set Axis \param a_ of the Frustum in contour FR */
		      void setAxis(const Vec &a_) {a = a_/nrm2(a_);}
		      /*! Get Axis \return a of the Frustum in contour FR */
		      const Vec& getAxis() const {return a;} 
		      /*! Set Height \param h_ of the Frustum */
		      void setHeight(double h_) {h = h_;}
		      /*! Get Height \return h of the Frustum */
		      double getHeight() const {return h;}
		      /*! Set Contact \param outcont_ on outer surface of the Frustum */
		      void setOutCont(bool outCont_) {outCont = outCont_;}
		      /*! Get Contact \return outcont on outer surface of the Frustum */
		      bool getOutCont() const {return outCont;}
  };

  /*! \brief Parent for contours described by two contour parameters \f$\vs\f$
   *
   * */
  class Contour2s : public Contour {
    protected:
      Vec as, ae;
      Mat nodes;
      Vec Cb;

    public:
      Contour2s(const string &name);

      Vec computeWn(const Vec& alpha) {
	Mat Wt = computeWt(alpha);
	if(Cb(2) < 0.0 ) return   crossProduct(Wt.col(0),Wt.col(1));
	else             return - crossProduct(Wt.col(0),Wt.col(1));
      }
      virtual Mat computeWt  (const Vec& alpha) = 0;
      virtual Vec computeWrOC(const Vec& alpha) = 0;
      virtual Vec computeWvC (const Vec& alpha) = 0;

      //    void setCb(const Vec& Cb);

      void setAlphaStart(Vec as_) {as = as_;}
      void setAlphaEnd(Vec ae_) {ae = ae_;}
      Vec getAlphaStart() const {return as;}
      Vec getAlphaEnd() const {return ae;}

      void setNodes(const Mat &nodes_) {nodes = nodes_;}
      const Mat& getNodes() const {return nodes;}
  };

  /*! \brief Basis-Class for Contour interpolation between Point s, standard contact Point-ContourInterpolation is implemented
    special interpolations only need to provide (as derived class) the pure virtuals predefined here
    */
  class ContourInterpolation : public Contour {
    protected:
      /** list of Point s holding ContourInterpolation */
      vector<Point*> iPoints;
      /** number of Contour-parameters used by ContourInterpolation: 1 for lines, 2 for surfaces */
      int contourParameters;
      /** size of iPoints, number of Point s used for interpolation */
      int numberOfPoints;

    public:
      ContourInterpolation(const string &name, int parameters_, int nPoints_);

      void plot(double t, double dt);

      //    Object* getObject() {return iPoints[0]->getObject();} // ACHTUNG: das kann in die Hose gehen, wenn iPoints noch nicht initzialisiert...

      /*! set Point for interpolation
	\param pointN Point to use
	\param position in iPoints, Point-number
	*/
      void setPoint(Point *pointN, int n);
      /*! get list of Point s */
      vector<Point*> getPoints()   const {return iPoints;}
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
      virtual double computePointWeight(const Vec &s, int i) = 0;
      /*! prototype of method giving first derivatives with respect to the diff-th Contour-parameters of all Point s 
	\param s Contour-parameter(s)
	\param i Point number
	\param diff -th derivative
	\return weight/derivative of Point i at s
	*/
      virtual double computePointWeight(const Vec &s, int i, int diff) = 0;

      /*! compute all weights for nodes */
      Vec computePointWeights(const Vec &s);

      Vec computeWrOC(const Vec& s);// {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWrOC(cp);};
      Vec computeWvC (const Vec& s);// {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWvC (cp);};
      Mat computeWt  (const Vec& s);// {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWt  (cp);};
      Vec computeWn  (const Vec& s);// {ContourPointData cp; cp.type=EXTINTERPOL;cp.alpha=s; return computeWn  (cp);};

      Vec computeWrOC(const ContourPointData &cp);
      Vec computeWvC (const ContourPointData &cp);

      Mat computeWt  (const ContourPointData &cp);
      virtual Vec computeWn  (const ContourPointData &cp) = 0;
  };

  /*! \brief Quad for 3D interpolation  \see{OpenGL-documentation}
  */

  class ContourQuad : public ContourInterpolation {
    protected:
    public:
      ContourQuad(const string & name);

      virtual void init();

      bool testInsideBounds(const ContourPointData &cp);
      double computePointWeight(const Vec &s, int i);
      double computePointWeight(const Vec &s, int i, int diff);

      Vec computeWn(const ContourPointData &cp);
  };

}

#endif /* _CONTOUR_H_ */
