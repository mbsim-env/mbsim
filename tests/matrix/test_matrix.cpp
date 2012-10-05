#include "matrix.h"
#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>

using namespace PLib;

template <class T>
class TestMatrix : public CppUnit::TestCase {

public:  
  TestMatrix(std::string name) : TestCase(name) {; }
  void setUp();
  void tearDown();

  void testInit();
  void testAddition();
  void testSubstraction();
  void testTranspose();

protected:
  Matrix<T>* matrixA;
  Matrix<T>* matrixB;
  Matrix<T>* matrixC;

  T value;
};

class TestMatrixInt : public TestMatrix<int>{
  CPPUNIT_TEST_SUITE( TestMatrixInt );
  CPPUNIT_TEST( testInit );
  CPPUNIT_TEST( testAddition );
  CPPUNIT_TEST( testSubstraction );
  CPPUNIT_TEST( testTranspose );
  CPPUNIT_TEST_SUITE_END();

public:
  TestMatrixInt() : TestMatrix<int>("TestMatrix<int>"){;}
};

class TestMatrixFloat : public TestMatrix<float>{
  CPPUNIT_TEST_SUITE( TestMatrixFloat );
  CPPUNIT_TEST( testInit );
  CPPUNIT_TEST( testAddition );
  CPPUNIT_TEST( testSubstraction );
  CPPUNIT_TEST( testTranspose );
  CPPUNIT_TEST_SUITE_END();
public:
  TestMatrixFloat() : TestMatrix<float>("TestMatrix<float>"){;}
};

class TestMatrixDouble : public TestMatrix<double>{
  CPPUNIT_TEST_SUITE( TestMatrixDouble );
  CPPUNIT_TEST( testInit );
  CPPUNIT_TEST( testAddition );
  CPPUNIT_TEST( testSubstraction );
  CPPUNIT_TEST( testTranspose );
  CPPUNIT_TEST_SUITE_END();
public:
  TestMatrixDouble() : TestMatrix<double>("TestMatrix<double>"){;}
};

CPPUNIT_TEST_SUITE_REGISTRATION( TestMatrixInt );
CPPUNIT_TEST_SUITE_REGISTRATION( TestMatrixFloat );
CPPUNIT_TEST_SUITE_REGISTRATION( TestMatrixDouble );

template <class T>
void TestMatrix<T>::setUp(){
  value = T(1);
  matrixA = new Matrix<T>(3,3);
  matrixA->reset(value);
  (*matrixA)(0,2) = T(0);
  matrixB = new Matrix<T>(3,3);
  matrixB->reset(value);
  (*matrixB)(0,2) = T(0);
  matrixC = new Matrix<T>;
}

template <class T>
void TestMatrix<T>::tearDown(){
  delete matrixA;
  delete matrixB;
  delete matrixC;
}

/** \brief verify that the matrix is initialize properly
 */
template <class T>
void TestMatrix<T>::testInit(){
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      if(i==0 && j==2){
	CPPUNIT_ASSERT_DOUBLES_EQUAL( (double)(*matrixA)(i,j), 0.0 , 0.00001);
      }
      else{
	CPPUNIT_ASSERT_DOUBLES_EQUAL( (double)(*matrixA)(i,j), double(value),0.00001);	
      }
    }
  }
}

template <class T>
void TestMatrix<T>::testAddition(){ 
  matrixC->reset(T(20));
  *matrixC = *matrixA + *matrixB;
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      double result = (double)(*matrixC)(i,j);
      if(i==0 && j==2){
	CPPUNIT_ASSERT_DOUBLES_EQUAL( result , 0.0,0.00001);
      }
      else{
	CPPUNIT_ASSERT_DOUBLES_EQUAL( result, 2.0*double(value),0.00001);	
      }
    }
  }
}

template <class T>
void TestMatrix<T>::testSubstraction(){
  *matrixC = *matrixA - *matrixB;
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      CPPUNIT_ASSERT_EQUAL( (*matrixC)(i,j), T(0));
    }
  }
}

template <class T>
void TestMatrix<T>::testTranspose(){
  *matrixC = matrixA->transpose();
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      CPPUNIT_ASSERT_EQUAL( (*matrixC)(j,i), (*matrixA)(i,j));
    }
  }
}



#ifdef NO_IMPLICIT_TEMPLATES


template TestMatrix<int>;
template TestMatrix<float>;
template TestMatrix<double>;

namespace CppUnit{
  namespace TestAssert{

    template void assertEquals<double>(double const&, double const&, CppUnit::SourceLine, std::string const&);
    template void assertEquals<float>(float const&, float const&, CppUnit::SourceLine, std::string const&);
    template void assertEquals<int>(int const&, int const&, CppUnit::SourceLine, std::string const&);
  }

  template TestCaller<TestMatrixInt, CppUnit::NoExceptionExpected>;
  template TestCaller<TestMatrixFloat, CppUnit::NoExceptionExpected>;
  template TestCaller<TestMatrixDouble, CppUnit::NoExceptionExpected>;

  template TestSuiteFactory<TestMatrixInt>;
  template TestSuiteFactory<TestMatrixFloat>;
  template TestSuiteFactory<TestMatrixDouble>;
}



#endif

