#include "nurbs.h"
#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>

using namespace PLib;

class TestNurbs: public CppUnit::TestCase {
  CPPUNIT_TEST_SUITE( TestNurbs ); 
  CPPUNIT_TEST( testBasisFunctions );
  CPPUNIT_TEST( testBasicEvaluation );
  CPPUNIT_TEST_SUITE_END();
public:
  TestNurbs() : TestCase("TestNurbs") {; }
  TestNurbs(std::string name) : TestCase(name) {; }

  void setUp();
  void tearDown();

  void testBasicEvaluation();
  void testBasisFunctions();

protected:
  PlNurbsCurvef curve;
  Vector_HPoint3Df control_points;
  Vector_HPoint3Df curve_points;
  Vector_FLOAT knots;
  int degree;
};

CPPUNIT_TEST_SUITE_REGISTRATION( TestNurbs );


/*! \brief setup the Test case
 * Setting up the test variables such that they correspond to the value
 * defined in example 4.1 of the NURBS book.
 */
void TestNurbs::setUp(){
  degree=2;
  control_points.resize(5);
  knots.resize(8);

  control_points[0] = HPoint3Df(0,0,1,1);
  // Point3D is 1,1,1 but in HPoint, it is in the format wx,wy,wz,w
  control_points[1] = HPoint3Df(4,4,4,4); 
  control_points[2] = HPoint3Df(3,2,1,1);
  control_points[3] = HPoint3Df(4,1,1,1);
  control_points[4] = HPoint3Df(5,-1,1,1);

  knots[0] = 0;
  knots[1] = 0;
  knots[2] = 0;
  knots[3] = 1;
  knots[4] = 2;
  knots[5] = 3;
  knots[6] = 3;
  knots[7] = 3;

  curve.reset(control_points,knots,degree);
}

void TestNurbs::tearDown(){
}

/*! \brief Testing the evaluation of a NURBS curve
  Test the evaluation in homogenous and normal space of a point
  on a NURBS curve.
  The test is based on example 4.1 from the NURBS book.
 */
void TestNurbs::testBasicEvaluation(){
  HPoint3Df hpoint = curve(1.0); 
  CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0/2.0,hpoint.x(),0.0001);
  CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0,hpoint.y(),0.0001); 
  CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0/2.0,hpoint.z(),0.0001);
  CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0/2.0,hpoint.w(),0.0001);

  Point3Df point = project(hpoint);
  CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0/5.0,point.x(),0.0001);
  CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0/5.0,point.y(),0.0001); 
  CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0,point.z(),0.0001);
}

/*! \brief Testing the computation of the Basis Functions
  Test that the computation of the Basis functions is working properly.
  The example used is the one from Exemple 4.1 inside The NURBS book.
 */
void TestNurbs::testBasisFunctions(){
  float u=1;
  Vector_FLOAT N;

  int span = curve.findSpan(u);
  CPPUNIT_ASSERT_EQUAL(3,span);

  curve.basisFuns(u,span,N);
  CPPUNIT_ASSERT_EQUAL(3,N.size());
  
  CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0/2.0,N[0],0.0001);
  CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0/2.0,N[1],0.0001);
  CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0,N[2],0.0001);
}

#ifdef NO_IMPLICIT_TEMPLATES

namespace CppUnit {

  template TestSuiteFactory<TestNurbs>;
  template TestCaller<TestNurbs, CppUnit::NoExceptionExpected>;

  namespace TestAssert {

    // Test if it is GCC 3.0 or above
    #if GCC_VERSION >= 30000
    template void assertEquals<int>(int const&, int const&, CppUnit::SourceLine, std::basic_string<char, std::char_traits<char>, std::allocator<char> > const&);
    #else
    template void CppUnit::TestAssert::assertEquals<int>(int const &, int const &, CppUnit::SourceLine, basic_string<char, string_char_traits<char>, __default_alloc_template<true, 0> > const &);
    #endif
  }
}

#endif

