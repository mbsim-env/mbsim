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

#include "mbsim/flexible_body.h"
#include "mbsim/element.h"
#include "mbsim/userfunction_contour.h"
#include "mbsim/contour_pdata.h"
#include "mbsim/frame.h"

#ifdef HAVE_OPONMBVCPPINTERFACE
#include <openmbvcppinterface/rigidbody.h>
#endif

namespace MBSim {

  class Object;

  // perhaps helpfull when debugging
  //                 0    , 1   , 2          , 3           , 4        , 5    , 6     , 7      , 8   , 9   , 10       , 11       , 12              , 13
  enum ContourType { point, line, circlesolid, circlehollow, frustum2D, plane, sphere, frustum, area, edge, contour1s, contour2d, cylinderflexible, interpolation };

  /** 
   * \brief basic class for contour definition for rigid (which do not know about their shape) and flexible (they know how they look like) bodies
   * \author Martin Foerg
   * \date 2009-03-23 some comments (Thorsten Schindler)
   * \date 2009-04-20 RigidContour added (Thorsten Schindler)
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

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "Contour"; }
      virtual void plot(double t, double dt = 1); 
      /***************************************************/

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \brief cartesian kinematic for contour (normal, tangent, binormal) is set by implementation class
       * \param contour parameter
       * \param selection of specific calculations for frames
       */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) = 0;

      /**
       * \brief JACOBIAN for contour (normal, tangent, binormal) is set by implementation class
       * \param contour parameter
       * \param selection of specific calculations for frames
       */
      virtual void updateJacobiansForFrame(ContourPointData &cp) = 0;

      /**
       * \return normal in world frame
       * \param contour position
       */
      virtual fmatvec::Vec computeNormal(ContourPointData &cp) { updateKinematicsForFrame(cp,normal); return cp.getFrameOfReference().getOrientation().col(0); }

      /**
       * \return position in world frame
       * \param contour position
       */
      virtual fmatvec::Vec computePosition(ContourPointData &cp) { updateKinematicsForFrame(cp,position); return cp.getFrameOfReference().getPosition(); }

      /**
       * \return velocity in world frame
       * \param contour position
       */
      virtual fmatvec::Vec computeVelocity(ContourPointData &cp) { updateKinematicsForFrame(cp,velocity); return cp.getFrameOfReference().getVelocity(); }

      /**
       * \return angular in world frame
       * \param contour position
       */
      virtual fmatvec::Vec computeAngularVelocity(ContourPointData &cp) { updateKinematicsForFrame(cp,angularVelocity); return cp.getFrameOfReference().getAngularVelocity(); } 
      
      /**
       * \param position of contour in inertial frame
       */
      virtual void setReferencePosition(const fmatvec::Vec &WrOP) { R.setPosition(WrOP); }

      /**
       * \param orientation of contour to inertial frame
       */
      virtual void setReferenceOrientation(const fmatvec::SqrMat &AWC) { R.setOrientation(AWC); }

      /**
       * \param velocity of contour in inertial frame
       */
      virtual void setReferenceVelocity(const fmatvec::Vec &WvP) { R.setVelocity(WvP); }

      /**
       * \param angular velocity of contour in inertial frame
       */
      virtual void setReferenceAngularVelocity(const fmatvec::Vec &WomegaC) { R.setAngularVelocity(WomegaC); }

      /**
       * \param JACOBIAN of translation of contour in inertial frame
       */
      virtual void setReferenceJacobianOfTranslation(const fmatvec::Mat &WJP) { R.setJacobianOfTranslation(WJP); }

      /**
       * \param gyroscopic acceleration of translation of contour in inertial frame
       */
      virtual void setReferenceGyroscopicAccelerationOfTranslation(const fmatvec::Vec &WjP) { R.setGyroscopicAccelerationOfTranslation(WjP); }

      /**
       * \param JACOBIAN of rotation of contour in inertial frame
       */
      virtual void setReferenceJacobianOfRotation(const fmatvec::Mat &WJR) { R.setJacobianOfRotation(WJR); }

      /**
       * \param gyroscopic acceleration of rotation of contour in inertial frame
       */
      virtual void setReferenceGyroscopicAccelerationOfRotation(const fmatvec::Vec &WjR) { R.setGyroscopicAccelerationOfRotation(WjR); }

      /**
       * \brief do things before initialisation
       */
      virtual void preinit() {}

      /**
       * \brief TODO
       */
      virtual void init();

      /**
       * \brief plots time series header
       */
      virtual void initPlot();
      /***************************************************/

      /* GETTER / SETTER */
      Frame* getFrame() { return &R; }
      const fmatvec::Vec& getReferencePosition() const { return R.getPosition(); }
      const fmatvec::SqrMat& getReferenceOrientation() const { return R.getOrientation(); }
      const fmatvec::Vec& getReferenceVelocity() const { return R.getVelocity(); }
      const fmatvec::Vec& getReferenceAngularVelocity() const { return R.getAngularVelocity(); }
      const fmatvec::Mat& getReferenceJacobianOfTranslation() const { return R.getJacobianOfTranslation(); }
      const fmatvec::Mat& getReferenceJacobianOfRotation() const { return R.getJacobianOfRotation(); }
      fmatvec::Mat& getReferenceJacobianOfTranslation() { return R.getJacobianOfTranslation(); }
      fmatvec::Mat& getReferenceJacobianOfRotation() { return R.getJacobianOfRotation(); }
      const fmatvec::Vec& getReferenceGyroscopicAccelerationOfTranslation() const { return R.getGyroscopicAccelerationOfTranslation(); }
      const fmatvec::Vec& getReferenceGyroscopicAccelerationOfRotation() const { return R.getGyroscopicAccelerationOfRotation(); }
      fmatvec::Vec& getReferenceGyroscopicAccelerationOfTranslation() { return R.getGyroscopicAccelerationOfTranslation(); }
      fmatvec::Vec& getReferenceGyroscopicAccelerationOfRotation() { return R.getGyroscopicAccelerationOfRotation(); }

      int gethSize(int i=0) const { return hSize[i]; }
      int gethInd(int i=0) const { return hInd[i]; }
      void sethSize(int size, int i=0) { hSize[i] = size; }
      void sethInd(int ind, int i=0) { hInd[i] = ind; }

      ObjectInterface* getParent() { return parent; }
      void setParent(ObjectInterface* parent_) { parent = parent_; }
      /***************************************************/

      /**
       * \brief TODO
       */
      void resizeJacobians(int j);

    protected:
      /**
       * \brief object the contour belongs to
       */
      ObjectInterface* parent;

      /**
       * \brief size and index of right hand side for frame JACOBIAN settings
       */
      int hSize[2], hInd[2];

      /**
       * coordinate system of contour
       */
      Frame R;

#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::RigidBody *openMBVRigidBody;
#endif
  };

  /**
   * \brief basic class for rigid contours
   * \author Thorsten Schindler
   * \date 2009-04-20 initial commit (Thorsten Schindler)
   */
  class RigidContour : public Contour {
    public:
      /**
       * \brief constructor
       * \param name of point
       */
      RigidContour(const std::string &name) : Contour(name) {}

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "RigidContour"; }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff);
      virtual void updateJacobiansForFrame(ContourPointData &cp);
      /***************************************************/
  };

  /**
   * \brief most primitive contour: the point (no extention)
   * \author Martin Foerg
   * \date 2009-03-19 comments (Thorsten Schindler)
   */
  class Point : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of point
       */
      Point(const std::string &name) : RigidContour(name) {}

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Point"; }
      /***************************************************/
  };

  /**
   * \brief unbounded line with constant normal
   * \author Martin Foerg
   * \date 2009-04-20 some comments (Thorsten Schindler)
   */
  class Line : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of line
       */      
      Line(const std::string &name) : RigidContour(name) {}

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Line"; }
      /***************************************************/
  };

  /**
   * \brief circular contour with contact possibility from outside
   * \author Martin Foerg
   * \date 2009-04-20 some commments (Thorsten Schindler)
   * \todo new name for binormal-methods TODO
   * \todo CircleSolid / CircleHollow should be unified TODO
   */
  class CircleSolid : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of circle
       */
      CircleSolid(const std::string &name) : RigidContour(name), r(0) {}

      /**
       * \brief constructor
       * \param name of circle
       * \param radius of circle
       */
      CircleSolid(const std::string &name, double r_) : RigidContour(name), r(r_) {}

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "CircleSolid"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setRadius(double r_) { r = r_; }
      double getRadius() const { return r; }
      /***************************************************/

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBV(bool enable=true);
#endif

    private:
      /**
       * \brief radius of circle
       */
      double r;
  };

  /**
   * \brief circle describing contact from inside
   * \author Roland Zander
   * \date 2009-04-20 some comments (Thorsten Schindler)
   */
  class CircleHollow : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of circle
       */
      CircleHollow(const std::string &name) : RigidContour(name), r(0.) {}

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "CircleHollow"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setRadius(double r_) { r = r_; }
      double getRadius() const { return r; }
      /***************************************************/

    private:
      /**
       * \brief radius of circle
       */
      double r;
  };

  /**
   * \brief planar slice of a frustum
   * \author Martin Foerg
   * \date 2009-04-20 some comments (Thorsten Schindler)
   */
  class Frustum2D : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of frustum
       */
      Frustum2D(const std::string &name) : RigidContour(name), r(2), h(0) {}

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Frustum2D"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setRadii(const fmatvec::Vec &r_) { r = r_; }
      const fmatvec::Vec& getRadii() const { return r; } 
      void setHeight(double h_) { h = h_; }
      double getHeight() const { return h; } 
      /***************************************************/

    private:
      /**
       * \brief radii of frustum in dirction of axis
       */
      fmatvec::Vec r;

      /**
       * \brief height of frustum
       */
      double h;
  };

  /** 
   * \brief basic class for contours described by a parametrisation
   * \author Thorsten Schindler
   * \date 2009-04-20 initial commit (Thorsten Schindler)
   */
  template <class AT>
    class ContourContinuum : public Contour {
      public:
        /**
         * \brief constructor 
         * \param name of contour
         */
        ContourContinuum(const std::string &name) : Contour(name) {}

        /* INHERITED INTERFACE OF ELEMENT */
        std::string getType() const { return "ContourContinuum"; }
        /***************************************************/
        
        /* INTERFACE FOR DERIVED CLASSES */
        /**
         * \brief compute necessary parameters for contact kinematics root function
         * \param contour point data
         */
        virtual void computeRootFunctionPosition(ContourPointData &cp) = 0;
        virtual void computeRootFunctionFirstTangent(ContourPointData &cp) = 0;
        virtual void computeRootFunctionNormal(ContourPointData &cp) = 0;
        virtual void computeRootFunctionSecondTangent(ContourPointData &cp) = 0;

        /* GETTER / SETTER */
        void setAlphaStart(AT as_) { as = as_; }
        void setAlphaEnd(AT ae_) { ae = ae_; }
        double getAlphaStart() const { return as; }
        double getAlphaEnd() const { return ae; }
        void setNodes(const std::vector<AT> &nodes_) { nodes = nodes_; }
        const std::vector<AT>& getNodes() const { return nodes; }
        /***************************************************/

      protected:
        AT as, ae;
        std::vector<AT> nodes;
    };

  /** 
   * \brief basic class for contours described by one contour parameter \f$s\f$
   * \author Roland Zander
   * \date 2009-04-20 frame-concept (Thorsten Schindler)
   */
  class Contour1s : public ContourContinuum<double> {
    public:
      /**
       * \brief constructor 
       * \param name of contour
       */
      Contour1s(const std::string &name) : ContourContinuum<double>(name), diameter(0.) {}

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Contour1s"; }
      /***************************************************/

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \return tangent in world frame
       * \param contour position
       */
      virtual fmatvec::Vec computeTangent(ContourPointData &cp) { updateKinematicsForFrame(cp,firstTangent); return cp.getFrameOfReference().getOrientation().col(1); }

      /**
       * \return binormal in world frame
       * \param Lagrangian position
       */
      virtual fmatvec::Vec computeBinormal(ContourPointData &cp) { updateKinematicsForFrame(cp,secondTangent); return cp.getFrameOfReference().getOrientation().col(2); }
      /***************************************************/

      /* GETTER / SETTER */
      void setDiameter(double diameter_) { diameter= diameter_; }
      double getDiameter() { return diameter; }
      /***************************************************/

    protected:
      /**
       * \brief diameter of neutral fibre
       */
      double diameter;
  };

  /** 
   * \brief analytical description of contours with one contour parameter
   * \author Robert Huber
   * \date 2009-04-20 some comments (Thorsten Schindler)
   */
  class Contour1sAnalytical : public Contour1s {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour1sAnalytical(const std::string &name) : Contour1s(name) {}

      /**
       * \brief destructor
       */
      virtual ~Contour1sAnalytical() { if (funcCrPC) delete funcCrPC; }

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Contour1sAnalytical"; }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff);
      /***************************************************/

      /* GETTER / SETTER */
      void setUserFunction(UserFunctionContour1s* f) { funcCrPC = f; }
      UserFunctionContour1s* getUserFunction() { return funcCrPC; }
      /***************************************************/

    protected:
      UserFunctionContour1s  *funcCrPC;
  };

  /** 
   * \brief numerical description of contours with one contour parameter
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

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) { static_cast<FlexibleBody*>(parent)->updateKinematicsForFrame(cp,ff); }
      virtual void updateJacobiansForFrame(ContourPointData &cp) { static_cast<FlexibleBody*>(parent)->updateJacobiansForFrame(cp); }
      /***************************************************/
      
      /* INHERITED INTERFACE OF CONTOURCONTINUUM */
      virtual void computeRootFunctionPosition(ContourPointData &cp) { updateKinematicsForFrame(cp,position); }
      virtual void computeRootFunctionFirstTangent(ContourPointData &cp) { updateKinematicsForFrame(cp,firstTangent); }
      virtual void computeRootFunctionNormal(ContourPointData &cp) { updateKinematicsForFrame(cp,normal); }
      virtual void computeRootFunctionSecondTangent(ContourPointData &cp) { updateKinematicsForFrame(cp,secondTangent); }
      /***************************************************/
  };

  /** 
   * \brief flexible cylinder for one dimensional flexible bodies
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-04-20 frame concept (Thorsten Schindler)
   */
  class CylinderFlexible : public Contour1sFlexible {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      CylinderFlexible(const std::string &name) : Contour1sFlexible(name) {}

      /* GETTER / SETTER */
      void setRadius(double r_) { r = r_; }
      double getRadius() const  { return r; }
      /***************************************************/

    protected:
      /**
       * \brief radius
       */
      double r;
  };

  /** 
   * \brief plane without borders
   * \author Martin Foerg
   * \date 2009-03-23 some comments (Thorsten Schindler)
   *
   * normal equals first column in orientation matrix
   */
  class Plane : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Plane(const std::string &name) : RigidContour(name) {}
  };

  /*! \brief RigidContour Area */
  class Area : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Area(const std::string &name);

      void setLimit1(double l) {lim1 = l;}
      void setLimit2(double l) {lim2 = l;}
      void setCd1(const fmatvec::Vec& Cd);
      void setCd2(const fmatvec::Vec& Cd);
      virtual void init();
      double getLimit1() const { return lim1; }
      double getLimit2() const { return lim2; }

      fmatvec::Vec computeWn() { return R.getOrientation()*Cn; }
      fmatvec::Vec computeWd1() { return R.getOrientation()*Cd1; }
      fmatvec::Vec computeWd2() { return R.getOrientation()*Cd2; }

    private:
      double lim1, lim2;
      fmatvec::Vec Cn, Cd1, Cd2;
  };

  /*! \brief RigidContour Edge */
  class Edge : public RigidContour {
    public:
      Edge(const std::string &name);

      void setLimit(double l) {lim = l;}
      void setCd(const fmatvec::Vec& Cd);
      void setCe(const fmatvec::Vec& Ce);
      double getLimit() const { return lim; }

      fmatvec::Vec computeWe() { return R.getOrientation()*Ce; }
      fmatvec::Vec computeWd() { return R.getOrientation()*Cd; }

    private:
      double lim;
      fmatvec::Vec Cn, Cd, Ce;
  };

  /**
   * \brief sphere 
   * \author Martin Foerg
   * \date 2009-04-20 some comments (Thorsten Schindler) 
   */
  class Sphere : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Sphere(const std::string &name) : RigidContour(name), r(0.) {}

      /* GETTER / SETTER */
      void setRadius(double r_) { r = r_; }
      double getRadius() const { return r; }
      /***************************************************/

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBV(bool enable=true);
#endif

    protected:
      /** 
       * \brief radius
       */
      double r;

  };

  /**
   * \brief frustum
   * \author Martin Foerg
   * \author Thorsten Schindler
   * \date 2009-04-20 some comments (Thorsten Schindler)
   */
  class Frustum : public RigidContour {
    public:
      /**
       * \brief constructor with contact from inside
       * \param name of contour
       */
      Frustum(const std::string &name) : RigidContour(name), r(2), h(0.), outCont(false) {}

      /**
       * \brief constructor
       * \param name of the contour
       * \param contact from outside?
       */
      Frustum(const std::string &name, bool outCont_) : RigidContour(name), r(2), h(0.), outCont(outCont_) {}

      /* GETTER / SETTER */
      void setRadii(const fmatvec::Vec &r_);
      const fmatvec::Vec& getRadii() const;
      void setHeight(double h_);
      double getHeight() const;
      void setOutCont(bool outCont_);
      bool getOutCont() const;
      /***************************************************/

    private:
      /** 
       * \brief upper r(1) and lower radius r(0) in direction of the axis
       */
      fmatvec::Vec r;

      /** 
       * \brief height
       */
      double h;

      /** 
       * \brief contact on outer or inner surface?
       */
      bool outCont;
  };

  inline void Frustum::setRadii(const fmatvec::Vec &r_) { r = r_; }
  inline const fmatvec::Vec& Frustum::getRadii() const { return r; }
  inline void Frustum::setHeight(double h_) { h = h_; }
  inline double Frustum::getHeight() const { return h; }
  inline void Frustum::setOutCont(bool outCont_) { outCont = outCont_; }
  inline bool Frustum::getOutCont() const { return outCont; }

  /**
   * \brief basic contour described by two contour parameters \f$\vs\f$
   * \author Roland Zander
   * \date 2009-04-20 frame concept (Thorsten Schindler)
   */
  class Contour2s : public ContourContinuum<fmatvec::Vec> {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour2s(const std::string &name) : ContourContinuum<fmatvec::Vec>(name) {}

      /**
       * \return tangents in world frame
       * \param Lagrangian position
       */
      virtual fmatvec::Mat computeTangentialPlane(fmatvec::Vec alpha) { ContourPointData cp(alpha); updateKinematicsForFrame(cp,cosy); return cp.getFrameOfReference().getOrientation()(0,1,2,2); }
  };

  /** 
   * \brief numerical description of contours with two contour parameter
   * \author Thorsten Schindler
   * \date 2009-04-21 initial comment (Thorsten Schindler)
   */
  class Contour2sFlexible : public Contour2s {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour2sFlexible(const std::string &name) : Contour2s(name) {}

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) { static_cast<FlexibleBody*>(parent)->updateKinematicsForFrame(cp,ff); }
      virtual void updateJacobiansForFrame(ContourPointData &cp) { static_cast<FlexibleBody*>(parent)->updateJacobiansForFrame(cp); }
      /***************************************************/
  };

  /*! \brief Basis-Class for Contour interpolation between Point s, standard contact Point-ContourInterpolation is implemented
    special interpolations only need to provide (as derived class) the pure virtuals predefined here
    */
  class ContourInterpolation : public Contour {
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

    protected:
      /** list of Point s holding ContourInterpolation */
      std::vector<Point*> iPoints;
      /** number of Contour-parameters used by ContourInterpolation: 1 for lines, 2 for surfaces */
      int contourParameters;
      /** size of iPoints, number of Point s used for interpolation */
      int numberOfPoints;

  };

  /*! \brief Quad for 3D interpolation  \see{OpenGL-documentation}
  */

  class ContourQuad : public ContourInterpolation {
    public:
      ContourQuad(const std::string & name);

      virtual void init();

      bool testInsideBounds(const ContourPointData &cp);
      double computePointWeight(const fmatvec::Vec &s, int i);
      double computePointWeight(const fmatvec::Vec &s, int i, int diff);

      fmatvec::Vec computeWn(const ContourPointData &cp);
  };

  /**
   * \brief contour consisting of primitive contour elements
   * \author Martin Foerg
   * \date 2009-04-20 some comments (Thorsten Schindler) 
   */
  class CompoundContour : public Contour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      CompoundContour(const std::string &name);

      /* INHERITED INTERFACE OF CONTOUR */
      void setReferencePosition(const fmatvec::Vec &WrOP);
      void setReferenceOrientation(const fmatvec::SqrMat &AWC);
      void setReferenceVelocity(const fmatvec::Vec &WvP);
      void setReferenceAngularVelocity(const fmatvec::Vec &WomegaC);
      void setReferenceJacobianOfTranslation(const fmatvec::Mat &WJP);
      void setReferenceGyroscopicAccelerationOfTranslation(const fmatvec::Vec &WjP);
      void setReferenceJacobianOfRotation(const fmatvec::Mat &WJR);
      void setReferenceGyroscopicAccelerationOfRotation(const fmatvec::Vec &WjR);
      /***************************************************/

      void init();
      Contour* getContourElement(int i) { return element[i]; }
      void addContourElement(Contour* ce, const fmatvec::Vec& re);
      unsigned int getNumberOfElements() { return element.size(); }

    private:
      std::vector<Contour*> element;
      std::vector<fmatvec::Vec> Kr, Wr;
  };


  /*! \brief Cuboid with 8 vertices, 12 edges and 6 faces */
  class Cuboid : public CompoundContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Cuboid(const std::string &name);

      /* GETTER / SETTER */
      void setLength(double l_) { l = l_; }
      void setHeight(double h_) { h = h_; }
      void setDepth(double d_) { d = d_; }
      /***************************************************/
      void preinit();

    private:
      /**
       * \brief length, height and depth of cuboid
       */
      double l,h,d;
  };

}

#endif /* _CONTOUR_H_ */

