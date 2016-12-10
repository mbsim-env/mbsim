#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_rcm.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/contact.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/circle.h"
#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
//#include "mbsim/contact_kinematics/circlesolid_flexibleband.h"
//#include "mbsimFlexibleBody/contact_kinematics/point_flexibleband.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/links/spring_damper.h"
#include "mbsim/links/kinetic_excitation.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/kinetics/kinetics.h"
#include "mbsim/functions/composite_function.h"

#include "beltDriveFunctions.h"

#include <openmbvcppinterface/spineextrusion.h>
#include "openmbvcppinterface/sphere.h"
#include <openmbvcppinterface/polygonpoint.h>
#include <openmbvcppinterface/arrow.h>
#include "openmbvcppinterface/coilspring.h"

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

class Moment : public MBSim::Function<VecV(double)> {
  double M0;
  double M1;
  double omegaM;
  public:
    Moment(double M0_, double M1_, double omegaM_) : M0(M0_), M1(M1_), omegaM(omegaM_) {}
    VecV operator()(const double& tVal) {
      VecV M(1);
      M(0) = M0 + M1*sin(tVal*omegaM);
      return M;
    };
};

class tpSin : public MBSim::Function<Vec(double)> {
   protected:
      Vec    J;
      double a, b, c, d;
   public:
      tpSin(const Vec &J_, double a_, double b_, double c_, double d_): J(J_), a(a_), b(b_), c(c_), d(d_) {}
      Vec operator()(const double& t) {
         double om = a + b*t + c*sin(d*t) ;
         //cout << "  tpSin(t=" << t << ") = " << J*om << endl;
         return J*om;
      }
};
class tpCos : public MBSim::Function<Vec(double)> {
   protected:
      Vec    J;
      double a, b, c, d;
   public:
      tpCos(const Vec &J_, double a_, double b_, double c_, double d_): J(J_), a(a_), b(b_), c(c_), d(d_) {}
      Vec operator()(const double& t) {
         double omp = a + b*t + c*cos(d*t) ;
         //cout << "  tpCos(t=" << t << ") = " << J*omp << endl;
         return J*omp;
      }
};
class SinusExcitedOnConstVelocity : public MBSim::Function<double(double)> {
   protected:
      Vec    JR;
      double omega0, phi0;
      double omegaExcitation, amplitudeExcitation;
      MBSim::Function<Vec(double)>* position;
      MBSim::Function<Vec(double)>* velocity;
      MBSim::Function<Vec(double)>* acceleration;
   public:
      SinusExcitedOnConstVelocity(Vec &JR_, double phi0_, double omega0_, double amplitudeExcitation_, double omegaExcitation_): JR(JR_), omega0(omega0_), phi0(phi0_), omegaExcitation(omegaExcitation_), amplitudeExcitation(amplitudeExcitation_) {
        position     = new tpCos(JR,   phi0, omega0,omegaExcitation>epsroot()?-amplitudeExcitation/omegaExcitation:0.0, omegaExcitation);
        velocity     = new tpSin(JR, omega0,    0.0,omegaExcitation>epsroot()? amplitudeExcitation                :0.0, omegaExcitation);
        acceleration = new tpCos(JR,    0.0,    0.0,omegaExcitation>epsroot()? amplitudeExcitation*omegaExcitation:0.0, omegaExcitation);
      }
      ~SinusExcitedOnConstVelocity() {
        delete position;
        delete velocity;
        delete acceleration;
      }
//      Vec operator() (const double &t, const void *) { return (*position)(t);}
      double operator ()(const double &t) {
         return phi0 + omega0 * t - ( omegaExcitation>epsroot()?-amplitudeExcitation/omegaExcitation:0.0 ) * (cos(omegaExcitation*t) - 1);
      }
      double parDer(const double &t) {
         return omega0 + ( omegaExcitation>epsroot()?-amplitudeExcitation/omegaExcitation:0.0 ) * sin(omegaExcitation*t) * omegaExcitation;
      }
      double parDerParDer(const double &t) {
         return ( omegaExcitation>epsroot()?-amplitudeExcitation/omegaExcitation:0.0 ) * cos(omegaExcitation*t) * omegaExcitation * omegaExcitation;
      }

      const MBSim::Function<Vec(double)>& getPositionFunction    () const {return *position    ;}
      const MBSim::Function<Vec(double)>& getVelocityFunction    () const {return *velocity    ;}
      const MBSim::Function<Vec(double)>& getAccelerationFunction() const {return *acceleration;}
      MBSim::Function<Vec(double)>& getPositionFunction    ()       {return *position    ;}
      MBSim::Function<Vec(double)>& getVelocityFunction    ()       {return *velocity    ;}
      MBSim::Function<Vec(double)>& getAccelerationFunction()       {return *acceleration;}
};


System::System(const string &projectName) : DynamicSystemSolver(projectName) {

// global environment
  Vec grav(3,INIT,0.0);//"[0.0;-9.81;0.0]");
//  grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

// belt globals
//  const double l0 = 4.0; // length
  const double b0_contact = 0.0; // width for contact
  //const double b0         = 5.0e-3; // width for visualisation and stiffness
//  const double E = 5.e7; // E-Modul  
//  const double A = b0*b0; // cross-section area
//  const double I1 = 1./12.*b0*b0*b0*b0/10.; // moment inertia
//  const double rho = 2.5e3; // density  
  const double A    = 1.0;
  const double EA   = 0.1 * 1e+6; // longitudinal stiffness
  const double EI   = 0.5 * 1e-5; // longitudinal stiffness
  const double rhoA = 0.1       ; // longitudinal stiffness
  const double E = EA / A;
  const int elements = 60; // number of finite elements

  const double     mu = 0.20; // poly V belts lead to "rise" in friction coefficient
//  const double omega0 = - 800  *2*M_PI/60; // first value in RPM
  const double omega0 = - 100; // first value in RPM
  const double     F0 = 0.5e3;

  const double  cSpring = 8.0 * 1e+4; // stiffness of tensioner spring
  const double  dSpring = 0.5 * 1e+3; // stiffness of tensioner spring
  const double l0Spring =  65 * 1e-3; // stiffness of tensioner spring
//  const double F0Spring = 80        ; // stiffness of tensioner spring
  const Vec  JtransSpring("[0;1;0]");
  const int      nWindings = 10;

// disk globals
  const Vec        massDisks = 1.0e-3*Vec("[300;1000;125]"); // masses of disks
  const Vec     inertiaDisks = 1.0e-3*Vec("[100;   5;0.5]"); // masses of disks
  const Vec       radiiDisks = 0.5e-3*Vec("[150;  50; 50]"); // radius of disks
  const int           nDisks = massDisks.size();
        VecInt sideInOut(nDisks);
        Mat    positionDisks = Mat(3,nDisks);    // radius of ball
  assert(nDisks == radiiDisks.size() && nDisks == inertiaDisks.size());

  positionDisks.col(0) = 1.0e-3*Vec("[  0;  0;  0]"); sideInOut(0) = -1; // 1: inside; -1:outside
  positionDisks.col(1) = 1.0e-3*Vec("[200;  0;  0]"); sideInOut(1) = -1; // 1: inside; -1:outside
  positionDisks.col(2) = 1.0e-3*Vec("[120; 50;  0]"); sideInOut(2) = +1; // 1: inside; -1:outside

        double     v0 = omega0 * radiiDisks(0);

  const double nodesPerElement = 0.25;
  const double nominalContactRadius = radiiDisks(1);

  bool enableContactPoints = true;

// initial condition for belt
  Mat start(2,nDisks), end(2,nDisks);
  for(int i=0;i<nDisks;i++) {
    int nEnd = (i+1)%nDisks;
    computeGeometry(
        positionDisks( 0, i   ), positionDisks( 1, i   ), 2*radiiDisks( i    ), sideInOut( i ),
        positionDisks( 0, nEnd), positionDisks( 1, nEnd), 2*radiiDisks( nEnd ), sideInOut( nEnd ),
        &start(0,i), &start(1,i),  &end(0,i), &end(1,i) ); 
/*****    cout << "   start " << i << "   end " << nEnd <<endl;        ******/
/*****    cout << "   s " << start(0,i) << "   e " << end(0,i) <<endl; ******/
/*****    cout << "   s " << start(1,i) << "   e " << end(1,i) <<endl; ******/
  }

  Vec wrapAngle(nDisks);
  for(int i=0;i<nDisks;i++){
    int nPre = (i-1+nDisks)%nDisks;
    Vec theVec = end.col(nPre) - positionDisks.col(i)(RangeV(0,1));
    Vec  Span1 = end.col(i)    - start.col(i);
    Vec  Span2 = end.col(nPre) - start.col(nPre) ;

    computeWrapAngle(theVec(0), theVec(1), Span1(0), Span1(1), Span2(0), Span2(1), &wrapAngle(i));

/********  cout << "disk " << i << " (pre="<<nPre<<")"<< endl;
  cout << trans(theVec) << endl;
  cout << trans(Span1) << endl;
  cout << trans(Span2) << endl;
  cout << "  wrap = " << wrapAngle(i) << endl;   ******/
  }

  double lengthPath = 0.0;
  Mat trackParts(4,nDisks);
  for(int i=0;i<nDisks;i++) {
    trackParts(0,i) = wrapAngle(i)*radiiDisks(i);
    trackParts(1,i) = sqrt( pow(end(0,i)-start(0,i),2) + pow(end(1,i)-start(1,i),2) ) ;
    lengthPath += trackParts(0,i);
    trackParts(2,i) = lengthPath;
    lengthPath += trackParts(1,i);
    trackParts(3,i) = lengthPath;
  }
/******  cout << "Lenght of track = " << lengthPath << endl; *******/
/******  cout << "trackParts = " << trackParts << endl;      *******/

  const double eps = 0.0e-3;
  double l0 = lengthPath/elements;
  Vec q0(5*elements,INIT,0.0);
  Vec u0(5*elements,INIT,0.0);

  for(int i=0;i<elements;i++) {
     static int part      = 0;
     static int pSpanWrap = 0; // 0: wrap; 1: span
     cout << pSpanWrap << endl;
     int nPre = (part-1+nDisks)%nDisks;
/*******     cout << " Element " << i << ":\t part=" << part <<"\t nPre=" << nPre << " pSpanWrap=" << pSpanWrap << endl; *********/

     double s = l0*i;
     if(s < trackParts(2,part)) {
        if(part > 0) s -= trackParts(3,nPre);
        double phi = s/radiiDisks(part);
        if(sideInOut( part )>0)
          phi *= -1.0;
/********     cout << "      sLocal   = " << s << endl;    *****/
/********     cout << "      phiLocal = " << phi << endl;  *****/
        pSpanWrap = 0;
        Vec  r0 = end.col(nPre) - positionDisks.col(part)(RangeV(0,1));
/********        cout << "      radius = " << nrm2(r0) << endl; ********/
        r0 *= 1.0+eps;
        Vec dir = (end.col(nPre) - start.col(nPre))/trackParts(1,nPre);
        SqrMat T(2); T(0,0) = cos(phi); T(0,1) = -sin(phi); T(1,0) = -T(0,1); T(1,1) = T(0,0);
        q0(RangeV(5*i+0,5*i+1)) = positionDisks.col(part)(RangeV(0,1)) + T*r0;
/********        cout << "      q0  " << q0(RangeV(5*i+0,5*i+1)) << endl; ********/
        dir = T*dir;
        q0(      5*i+2)        = atan2( dir(1), dir(0) );
        u0(RangeV(5*i+0,5*i+1)) =   v0*dir;
        u0(      5*i+2)        = - v0/radiiDisks(part) * sideInOut ( part );
        while(  i>0 && ( fabs(q0(5*i+2) - q0(5*(i-1)+2))>M_PI )   )
           if (q0(5*i+2) > q0(5*(i-1)+2))
              q0(5*i+2) -= 2*M_PI;
           else
              q0(5*i+2) += 2*M_PI;
        double a_ = -sideInOut( part )*(sqrt(radiiDisks(part)*radiiDisks(part) + (l0*l0)/16.) - radiiDisks(part));
        if(part == 2) a_ /= 3.0;
        q0(5*i+3) = a_;
        if(i>0)
           q0(5*(i       -1)+4) = a_;
        else
           q0(5*(elements-1)+4) = a_;
     } else if(i*l0 < trackParts(3,part)) {
        s -= trackParts(2,part);
/********     cout << "      sLocal   = " << s << endl; ************/
        pSpanWrap = 1;
        Vec dir = (end.col(part) - start.col(part))/trackParts(1,part);
/********        cout << "      dir " << dir << endl; ************/
        q0(RangeV(5*i+0,5*i+1)) = start.col(part) + dir * s;
        u0(RangeV(5*i+0,5*i+1)) =                   dir * v0;
/********        cout << "      q0  " << q0(RangeV(5*i+0,5*i+1)) << endl; ************/
        q0(      5*i+2)        = atan2( dir(1), dir(0) );
        while(  i>0 && ( fabs(q0(5*i+2) - q0(5*(i-1)+2))>M_PI )   )
           if (q0(5*i+2) > q0(5*(i-1)+2))
              q0(5*i+2) -= 2*M_PI;
           else
              q0(5*i+2) += 2*M_PI;
     }
     else
     {
        part++;
        i--;
     }
     assert(part<nDisks);
  }

  {
    ofstream test("track.dat");
    test.precision(6);
    test << scientific;
    for(int i=0;i<elements;i++) {
      for(int j=0;j<5;j++)
        test << "\t" << q0(5*i+j);
      for(int j=0;j<5;j++)
        test << "\t" << u0(5*i+j);
      test << endl;
    }
    for(int j=0;j<5;j++)
       if(j!=2)
          test << "\t" << q0(5*0+j);
       else
          test << "\t" << q0(5*0+j)+2*M_PI;
    for(int j=0;j<5;j++)
      test << "\t" << u0(5*0+j);
    test << endl;
    test.close();
  }

  double beltLength = lengthPath * (1.0 - F0/(E*A));

  cout << "length of path = " << lengthPath << endl;
  cout << "length of belt = " << beltLength << endl;

// implementation
  FlexibleBody1s21RCM *belt = new FlexibleBody1s21RCM("Belt", false);
  belt->setLength(beltLength);
  belt->setEModul(E);
  belt->setCrossSectionalArea(A);
  belt->setMomentInertia(EI/E);
  belt->setDensity(rhoA/A);
  belt->setFrameOfReference(this->getFrame("I"));
  belt->setNumberElements(elements);
//  belt->initRelaxed(0.0);
  belt->setq0(q0);
  belt->setu0(u0);
  VecInt elementPlotList("[0;5;10]");
  belt->setElementPlotList(elementPlotList);
  this->addObject(belt);

  std::shared_ptr<OpenMBV::SpineExtrusion> cuboid=OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>();
  cuboid->setNumberOfSpinePoints(5*elements+1); // resolution of visualisation
  cuboid->setDiffuseColor(2/3.0, 1, 1); // color in (minimalColorValue, maximalColorValue)
  cuboid->setScaleFactor(1.); // orthotropic scaling of cross section
  shared_ptr<vector<shared_ptr<OpenMBV::PolygonPoint> > > rectangle = make_shared<vector<shared_ptr<OpenMBV::PolygonPoint> > >(); // clockwise ordering, no doubling for closure
  double h0 = 50.0e-3;
  double b0_vis = 1.0e-3;
  shared_ptr<OpenMBV::PolygonPoint>  corner1 = OpenMBV::PolygonPoint::create( b0_vis*0.5, h0*0.5,1);
  rectangle->push_back(corner1);
  shared_ptr<OpenMBV::PolygonPoint>  corner2 = OpenMBV::PolygonPoint::create( b0_vis*0.5,-h0*0.5,1);
  rectangle->push_back(corner2);
  shared_ptr<OpenMBV::PolygonPoint>  corner3 = OpenMBV::PolygonPoint::create(-b0_vis*0.5,-h0*0.5,1);
  rectangle->push_back(corner3);
  shared_ptr<OpenMBV::PolygonPoint>  corner4 = OpenMBV::PolygonPoint::create(-b0_vis*0.5, h0*0.5,1);
  rectangle->push_back(corner4);

  cuboid->setContour(rectangle);
  belt->setOpenMBVSpineExtrusion(cuboid);

  Contour1sFlexible *neutral = new Contour1sFlexible("Neutral");
  belt->addContour(neutral);

  vector<FlexibleBand*> band;
  for(int iBand = 0; iBand < radiiDisks.size();iBand++) {
    stringstream name;
    name << "Band" << iBand;
    FlexibleBand* bd = new FlexibleBand(name.str());
    int nNodes = (int)((nodesPerElement*elements)*sqrt(nominalContactRadius/radiiDisks(iBand)) + 0.5);
    Vec nodes(nNodes);
    for(int i=0;i<nNodes;i++)
      nodes(i) = i*beltLength/(nNodes-1);
    cout << nodes << endl;
    bd->setNodes(nodes);
    bd->setWidth(1.0);
    Vec2 r;
    r(0) = -0.5*b0_contact*sideInOut(iBand);
    bd->setRelativePosition(r);
    bd->setRelativeOrientation(sideInOut(iBand)==1?0:M_PI);
    bd->setContourOfReference(neutral);
    belt->addContour(bd);
    band.push_back(bd);
  }

  for(int i=0;i<nDisks;i++) {
    bool diskHasRotationalDOFs = false;
    stringstream name;
    name.clear();
    name << "Disk" << i; 
    RigidBody *disk = new RigidBody(name.str());

    name.clear();
    name << "B" << i;
    this->addFrame(new FixedRelativeFrame(name.str(),positionDisks.col(i),SqrMat(3,EYE),this->getFrame("I")));
    disk->setFrameOfReference(this->getFrame(name.str()));
    disk->setFrameForKinematics(disk->getFrame("C"));
    disk->setMass(massDisks(i));
    SymMat Theta(3);
    Theta(0,0) = inertiaDisks(i);//2./5.*massDisks(i)*radiiDisks(i)*radiiDisks(i);
    Theta(1,1) = inertiaDisks(i);//2./5.*massDisks(i)*radiiDisks(i)*radiiDisks(i);
    Theta(2,2) = inertiaDisks(i);//2./5.*massDisks(i)*radiiDisks(i)*radiiDisks(i);
    disk->setInertiaTensor(Theta);

    disk->setPlotFeature(Element::globalPosition,enabled);
    disk->setPlotFeature(Element::globalVelocity,enabled);
    disk->getFrame("C")->setPlotFeature(Element::globalPosition,enabled);
    disk->getFrame("C")->setPlotFeature(Element::globalVelocity,enabled);

    Vec JR("[0;0;1.0]");
    if(i==0)
    {
      SinusExcitedOnConstVelocity *excitation = new SinusExcitedOnConstVelocity(JR, 0.0        , omega0, 0.0*omega0,3*omega0);
      disk->setRotation(new CompositeFunction<RotMat3(double(double))>(new RotationAboutFixedAxis<double>(JR),excitation));
//      disk->setGuidingVelocityOfRotation             (&(excitation->getVelocityFunction()    ))  ;
//      disk->setDerivativeOfGuidingVelocityOfRotation (&(excitation->getAccelerationFunction()))  ;
      diskHasRotationalDOFs = false;
    }
    else {
       disk->setRotation(new RotationAboutFixedAxis<VecV>(JR));
       diskHasRotationalDOFs = true;
    }

    if(i==nDisks-1) {
       disk->setTranslation(new LinearTranslation<VecV>(JtransSpring));
       Vec vInit(2,INIT,0.);
       vInit(1) = -v0*sideInOut( i )/radiiDisks(i);
       disk->setGeneralizedInitialVelocity(vInit);
    } else if(diskHasRotationalDOFs) {
       Vec vInit(1,INIT,0.);
       vInit(0) = -v0*sideInOut( i )/radiiDisks(i);
       disk->setGeneralizedInitialVelocity(vInit);
    }

    Circle *cDisk = new Circle("cDisk");
    cDisk->setRadius(0.98*radiiDisks(i));
    Vec BR(3,INIT,0.);// BR(1)=-r;
    disk->addFrame(new FixedRelativeFrame("cDisk",BR,SqrMat(3,EYE),disk->getFrame("C")));
    cDisk->setFrameOfReference(disk->getFrame("cDisk"));
    disk->addContour(cDisk);
    cDisk->enableOpenMBV();
    this->addObject(disk);

//    std::shared_ptr<OpenMBV::Sphere> cylinder=OpenMBV::ObjectFactory::create<OpenMBV::Sphere>();
//    cylinder->setRadius(radiiDisks(i));
//    cylinder->setDiffuseColor(1/3.0, 1, 1);
//    disk->setOpenMBVRigidBody(cylinder);

    //  ContactKinematicsSolidCircleFlexibleBand *ck = new ContactKinematicsSolidCircleFlexibleBand();
    name.clear();
    name << "Contact" << i;
    Contact *contact = new Contact(name.str());
    //  contact->setContactKinematics(ck);
    //contact->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(c,d)));
    cout << name.str() << " is ";
// #define rigidContact 1
// #if rigidContact
    if (true) {
       contact->setNormalForceLaw(new UnilateralConstraint);
       contact->setNormalImpactLaw(new UnilateralNewtonImpact(.0));
       if(mu>0.0) {
          contact->setTangentialForceLaw(new PlanarCoulombFriction(mu));
          contact->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
       }
      cout << "rigid";
    }
    else
    {
      contact->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e6,1e3)));
      cout << "flexible";
    }
// #endif
    cout << " contact " << endl;
//    if(sideInOut(i) < 0)
//      contact->connect(disk->getContour("cDisk"),belt->getContour("Top"));
//    else
    contact->connect(disk->getContour("cDisk"),band[i]);
//    contact->setPlotFeature(linkLagrangeParameters, enabled);
    contact->enableOpenMBVNormalForce(_scaleLength=0.0002);
    if(mu>0)
      contact->enableOpenMBVTangentialForce(_scaleLength=0.0002);
    if(enableContactPoints)
      contact->enableOpenMBVContactPoints(0.0075);
    this->addLink(contact);

    if(i==1) {
      double omega0 = -100;
      double M0, M1, omegaM;
      KineticExcitation* ke;
      ke = new KineticExcitation("TorqueGenerator");
      addLink(ke);
      ke->connect(disk->getFrame("C"));
      if(omega0 < 0) {
        M0 = +10.0; M1 = +5.0; omegaM = fabs(2*omega0); }
      else {
        M0 = -10.0; M1 = -5.0; omegaM = fabs(2*omega0); }
      ke->setMomentDirection(JR);
      ke->setMomentFunction(new Moment(M0,M1,omegaM));
      ke->enableOpenMBVMoment(_scaleLength=0.005 * 15./(fabs(M0)+fabs(M1)));
    }

    if(i==nDisks-1) {
       // const Vec f0 = (end.col(nDisks-2) - start.col(nDisks-2))/trackParts(1,nDisks-2);
       // const Vec f1 = (end.col(nDisks-1) - start.col(nDisks-1))/trackParts(1,nDisks-1);
       // const double F0Spring = trans(JtransSpring(RangeV(0,1))) * ( f0 + f1 ) * F0;
       const double F0Spring = 2.3215000000E+02;

       Vec posSpring = positionDisks.col(nDisks-1) + (l0Spring - F0Spring/cSpring ) * JtransSpring;
       this->addFrame(new FixedRelativeFrame("BS",posSpring,SqrMat(3,EYE),this->getFrame("I")));

       // ----------------------- Definition der 1. Feder --------------------  
       SpringDamper *spring = new SpringDamper("TensionerSpring");
       addLink(spring);
       spring->setForceFunction(new LinearSpringDamperForce(cSpring,dSpring));
       spring->setUnloadedLength(l0Spring);
       spring->connect(disk->getFrame("C"),this->getFrame("BS"));
       spring->enableOpenMBVCoilSpring(_springRadius=0.1*radiiDisks(i),_crossSectionRadius=0.005*radiiDisks(i),_numberOfCoils=nWindings);
    }
  }
}

