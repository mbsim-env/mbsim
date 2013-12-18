#include "matrix.h"
#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>

using namespace PLib;

template <class T>
class TestBasic2DArray : public CppUnit::TestCase {

public:  
  TestBasic2DArray(std::string name) : TestCase(name) {; }
  void setUp();
  void tearDown();

  void testInit();
  void testResize();
  void testResizeKeep();

protected:
  Basic2DArray<T>* array;

  T value;
  int n_rows;
  int n_cols;
};

class TestBasic2DArrayInt : public TestBasic2DArray<int>{
  CPPUNIT_TEST_SUITE( TestBasic2DArrayInt );
  CPPUNIT_TEST( testInit );
  CPPUNIT_TEST( testResize );
  CPPUNIT_TEST( testResizeKeep );
  CPPUNIT_TEST_SUITE_END();

public:
  TestBasic2DArrayInt() : TestBasic2DArray<int>("TestBasic2DArray<int>"){;}
};

class TestBasic2DArrayFloat : public TestBasic2DArray<float>{
  CPPUNIT_TEST_SUITE( TestBasic2DArrayFloat );
  CPPUNIT_TEST( testInit );
  CPPUNIT_TEST( testResize );
  CPPUNIT_TEST( testResizeKeep );
  CPPUNIT_TEST_SUITE_END();
public:
  TestBasic2DArrayFloat() : TestBasic2DArray<float>("TestBasic2DArray<float>"){;}
};

class TestBasic2DArrayDouble : public TestBasic2DArray<double>{
  CPPUNIT_TEST_SUITE( TestBasic2DArrayDouble );
  CPPUNIT_TEST( testInit );
  CPPUNIT_TEST( testResize );
  CPPUNIT_TEST( testResizeKeep );
  CPPUNIT_TEST_SUITE_END();
public:
  TestBasic2DArrayDouble() : TestBasic2DArray<double>("TestBasic2DArray<double>"){;}
};

class TestBasic2DArrayHPoint2Df : public TestBasic2DArray<HPoint2Df>{
  CPPUNIT_TEST_SUITE( TestBasic2DArrayDouble );
  CPPUNIT_TEST( testInit );
  CPPUNIT_TEST( testResize );
  CPPUNIT_TEST( testResizeKeep );
  CPPUNIT_TEST_SUITE_END();
public:
  TestBasic2DArrayHPoint2Df() : TestBasic2DArray<HPoint2Df>("TestBasic2DArray<HPoint2Df>"){;}
};

class TestBasic2DArrayHPoint2Dd : public TestBasic2DArray<HPoint2Dd>{
  CPPUNIT_TEST_SUITE( TestBasic2DArrayDouble );
  CPPUNIT_TEST( testInit );
  CPPUNIT_TEST( testResize ); 
  CPPUNIT_TEST( testResizeKeep );
  CPPUNIT_TEST_SUITE_END();
public:
  TestBasic2DArrayHPoint2Dd() : TestBasic2DArray<HPoint2Dd>("TestBasic2DArray<HPoint2Dd>"){;}
};

class TestBasic2DArrayHPoint3Df : public TestBasic2DArray<HPoint3Df>{
  CPPUNIT_TEST_SUITE( TestBasic2DArrayDouble );
  CPPUNIT_TEST( testInit );
  CPPUNIT_TEST( testResize );
  CPPUNIT_TEST( testResizeKeep );
  CPPUNIT_TEST_SUITE_END();
public:
  TestBasic2DArrayHPoint3Df() : TestBasic2DArray<HPoint3Df>("TestBasic2DArray<HPoint3Df>"){;}
};

class TestBasic2DArrayHPoint3Dd : public TestBasic2DArray<HPoint3Dd>{
  CPPUNIT_TEST_SUITE( TestBasic2DArrayDouble );
  CPPUNIT_TEST( testInit );
  CPPUNIT_TEST( testResize );
  CPPUNIT_TEST( testResizeKeep );
  CPPUNIT_TEST_SUITE_END();
public:
  TestBasic2DArrayHPoint3Dd() : TestBasic2DArray<HPoint3Dd>("TestBasic2DArray<HPoint3Dd>"){;}
};

CPPUNIT_TEST_SUITE_REGISTRATION( TestBasic2DArrayInt );
CPPUNIT_TEST_SUITE_REGISTRATION( TestBasic2DArrayFloat );
CPPUNIT_TEST_SUITE_REGISTRATION( TestBasic2DArrayDouble );
CPPUNIT_TEST_SUITE_REGISTRATION( TestBasic2DArrayHPoint3Df );
CPPUNIT_TEST_SUITE_REGISTRATION( TestBasic2DArrayHPoint2Df );
CPPUNIT_TEST_SUITE_REGISTRATION( TestBasic2DArrayHPoint3Dd );
CPPUNIT_TEST_SUITE_REGISTRATION( TestBasic2DArrayHPoint2Dd );


template <class T>
void TestBasic2DArray<T>::setUp(){
  value = T(1);
  n_rows = 10;
  n_cols = 20;
  array = new Basic2DArray<T>(n_rows,n_cols);
  array->reset(value);
}

template <class T>
void TestBasic2DArray<T>::tearDown(){
  delete array;
}

/** \brief verify that the matrix is initialize properly
 */
template <class T>
void TestBasic2DArray<T>::testInit(){
  for(int i=0;i<n_rows;i++){
    for(int j=0;j<n_cols;j++){
      CPPUNIT_ASSERT_EQUAL( value, (*array)(i,j) );	
    }
  }
}

template <class T>
void TestBasic2DArray<T>::testResize(){ 
  array->resize(2*n_rows,2*n_cols);
  T empty;
  empty = 0;
  for(int i=0;i<2*n_rows;i++){
    for(int j=0;j<2*n_cols;j++){
      CPPUNIT_ASSERT_EQUAL( empty, (*array)(i,j) );
    }
  }  
}

template <class T>
void TestBasic2DArray<T>::testResizeKeep(){ 
  array->resizeKeep(2*n_rows,2*n_cols);
  T empty;
  empty = T(0);
  value = T(1);
  for(int i=0;i<2*n_rows;i++){
    for(int j=0;j<2*n_cols;j++){
      T result = array->elem(i,j);
      if(i< n_rows && j<n_cols){ 
	CPPUNIT_ASSERT_EQUAL( value, result);	
      }
      else{ 
	CPPUNIT_ASSERT_EQUAL( empty, result);	
      }
    }
  }  
}



#ifdef NO_IMPLICIT_TEMPLATES


template TestBasic2DArray<int>;
template TestBasic2DArray<float>;
template TestBasic2DArray<double>;
template TestBasic2DArray<HPoint_nD<float,3> >;
template TestBasic2DArray<HPoint3Dd >;
template TestBasic2DArray<HPoint2Df >;
template TestBasic2DArray<HPoint2Dd >;

namespace CppUnit{
  namespace TestAssert{
    
    template void assertEquals<double>(double const&, double const&, CppUnit::SourceLine, std::string const&);
    template void assertEquals<float>(float const&, float const&, CppUnit::SourceLine, std::string const&);
    template void assertEquals<int>(int const&, int const&, CppUnit::SourceLine, std::string const&);
    template void assertEquals<PLib::HPoint_nD<float, 3> >(PLib::HPoint_nD<float, 3> const&, PLib::HPoint_nD<float, 3> const&, CppUnit::SourceLine, std::string const&);
    template void assertEquals<PLib::HPoint_nD<float, 2> >(PLib::HPoint_nD<float, 2> const&, PLib::HPoint_nD<float, 2> const&, CppUnit::SourceLine, std::string const&);
    template void assertEquals<PLib::HPoint_nD<double, 3> >(PLib::HPoint_nD<double, 3> const&, PLib::HPoint_nD<double, 3> const&, CppUnit::SourceLine, std::string const&);
    template void assertEquals<PLib::HPoint_nD<double, 2> >(PLib::HPoint_nD<double, 2> const&, PLib::HPoint_nD<double, 2> const&, CppUnit::SourceLine, std::string const&);
    
  }

  template TestSuiteFactory<TestBasic2DArrayInt>;
  template TestSuiteFactory<TestBasic2DArrayFloat>;
  template TestSuiteFactory<TestBasic2DArrayDouble>;
  template TestSuiteFactory<TestBasic2DArrayHPoint3Df>;
  template TestSuiteFactory<TestBasic2DArrayHPoint3Dd>;
  template TestSuiteFactory<TestBasic2DArrayHPoint2Df>;
  template TestSuiteFactory<TestBasic2DArrayHPoint2Dd>;
  template TestCaller<TestBasic2DArrayInt, CppUnit::NoExceptionExpected>;
  template TestCaller<TestBasic2DArrayFloat, CppUnit::NoExceptionExpected>;
  template TestCaller<TestBasic2DArrayDouble, CppUnit::NoExceptionExpected>;
  template TestCaller<TestBasic2DArrayHPoint3Df, CppUnit::NoExceptionExpected>;
  template TestCaller<TestBasic2DArrayHPoint3Dd, CppUnit::NoExceptionExpected>;
  template TestCaller<TestBasic2DArrayHPoint2Df, CppUnit::NoExceptionExpected>;
  template TestCaller<TestBasic2DArrayHPoint2Dd, CppUnit::NoExceptionExpected>;
}


#endif


