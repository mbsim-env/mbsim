#ifndef _CVTREFERENCECURVE_H
#define _CVTREFERENCECURVE_H

#include <mbsimFlexibleBody/flexible_body/flexible_body_1S_reference_curve.h>

/*!
 * \brief Computes Theta out of geometric ratio and vice versa
 */
double ig2ThetaViceVersa(double input);

/*!
 * \brief compute some important / characterizing belt numbers based on the given values of...
 * \param rP              Primary Radius
 * \param theta           Ratio [-2;2]
 * \param pulleyDistance  Distance of the two pulley shafts
 */
void computeBeltNumbers(double rP, double theta, double pulleyDistance, double & rS, double & phiMax, double & endSecondary, double & endPushPart, double & endPrimary, double & endLoosePart);

class PlanarBeltKinematicFunction : public MBSim::Function<fmatvec::Vec(fmatvec::Vec)> {
  public:
    PlanarBeltKinematicFunction(double length, double pulleyDistance, double ig_target);

    virtual ~PlanarBeltKinematicFunction();

    /*!
     * \input argument is the primary radius
     */
    fmatvec::Vec operator()(const fmatvec::Vec & rP);

  protected:
    /*!
     * \brief length of belt
     */
    double length;

    /*!
     * \brief distance of pulleys
     */
    double pulleyDistance;

    /*!
     * \brief target geometric ratio
     */
    double ig_target;
};

class SimpleRefCurveFunction : public MBSim::Function<fmatvec::Vec2(double)> {
  public:
    SimpleRefCurveFunction(double length, double pulleyDistance, double theta);

    virtual ~SimpleRefCurveFunction();

    /*!
     * \input argument is the primary radius
     */
    fmatvec::Vec2 operator()(const double & x);

    /*!
     * \input argument is the primary radius
     */
    fmatvec::Vec2 parDer(const double & x);

    /*!
     * \brief getter functions for solution of newton algorithm
     */
    double getPrimaryRadius();
    double getSecondaryRadius();
    double getEndSecondary();
    double getEndPushPart();

  protected:
    /*!
     * \brief length of belt
     */
    double length;

    /*!
     * \brief distance of pulleys
     */
    double pulleyDistance;

    /*!
     * \brief target geometric ratio
     */
    double theta;

    /*!
     * \brief x coordinate of end secondary
     */
    double endSecondary;

    /*!
     * \brief x coordinate of push part
     */
    double endPushPart;

    /*!
     * \brief coordinate of primary start
     */
    fmatvec::Vec2 rPriStart;

    /*!
     * \brief coordinate of secondary start
     */
    fmatvec::Vec2 rSecEnd;

    /*!
     * \brief radius primary
     */
    double rP;

    /*!
     * \brief radius secondary
     */
    double rS;

    /*!
     * \brief maximal angle
     */
    double phiMax;
};

/*!
 * \brief A Pre-Processor Class to compute the control points and the interpolated curves of the control points for different geometrical ratios
 */
class CVTReferenceSurface : public MBSimFlexibleBody::ReferenceCurve {
  public:
    CVTReferenceSurface();
    virtual ~CVTReferenceSurface();

    /*INTERFACE FUNCTIONS*/
    /*!
     * \brief set the necessary boundary-conditions for the interpolation
     */
    virtual void setConstraints(double length, double pulleyDistance, double saddleCenter, double thetaLower = -0.5, double thetaUpper = 0.5, int halfGridSize = 20);

    /*!
     * \brief provide the parameters for the simple reference function concerning the misaligment
     */
//        virtual void provideParameters(const ElementParameters elePars, const RingSetParameters ringPars, PulleySetParameters * pulleyPars, const ElementPulleyContactParameters elpuPars);
    /*!
     * \brief get the reference curve at a certain geometric ratio
     */
    virtual void computeReference() = 0;

    /*!
     * \brief computes the vector/point on the surface for specific derivatives
     */
    virtual fmatvec::Vec3 computeVecAt(double xi, double Theta, int derXi, int derTheta) = 0;

    /*!
     * \brief computes the primary Radius w.r.t. the given theta
     */
    virtual double computerP(double Theta) = 0;


    /*END INTERFACE FUNCTIONS*/

    double getThetaLower() {
      return thetaLower;
    }

    double getThetaUpper() {
      return thetaUpper;
    }

  protected:
    /*!
     * \brief length of belt
     */
    double length;

    /*!
     * \brief distance of pulleys
     */
    double pulleyDistance;

    /*!
     * \brief lower geometric ratio
     */
    double thetaLower;

    /*!
     * \brief upper geometric ratio
     */
    double thetaUpper;

    int gridSize() {
      return 2 * halfGridSize;
    }

    /*!
     * \brief size of grid (number of Control Points)
     */
    int halfGridSize;

    /*!
     * \brief center position of the saddle
     */
    double saddleCenter;

    /*!
     * \brief center position over theta for primary (first index (=0)) and secondary (second index (=1))
     */
    MBSim::NurbsCurve centerPositions;
};

class CVTReferenceSurfaceParts : public CVTReferenceSurface {
  public:
    CVTReferenceSurfaceParts();
    virtual ~CVTReferenceSurfaceParts();

    /*!
     * \brief interpolate curves for the control points
     */
    virtual void computeReference();

    virtual fmatvec::Vec3 computeVecAt(double xi, double Theta, int derXi, int derTheta);

    virtual double computerP(double theta);

    virtual fmatvec::VecV computeC1Positions(const fmatvec::Vec & qRef);

  private:
    /*
     * \brief an interpolated curve of the secondary radius depending on the current ratio Theta
     */
    MBSim::NurbsCurve primaryRadius;

    /*!
     * \brief compute the vectors or its derivatives w.r.t.
     */
    void computeVecAt00(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt01(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt02(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt10(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt11(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt12(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt20(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt21(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt22(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt30(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt31(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt32(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt40(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt41(fmatvec::Vec3 & retVec, double xi, double Theta);
    void computeVecAt42(fmatvec::Vec3 & retVec, double xi, double Theta);

};

#endif
