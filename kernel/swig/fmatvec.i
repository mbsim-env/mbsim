// check target language
#ifndef SWIGPYTHON
  #error "Only Pyhton as target language is supported."
#endif

// init numpy
%init %{
  _import_array();
%}

// use SWIG_exception to throw a target language exception
%include exception.i

// helper define to convert c++ exceptions in typemaps to target language exceptions
%define FMATVEC_CATCHARG
  catch(const std::exception &ex) {
    Py_XDECREF($result);
    SWIG_exception(SWIG_RuntimeError, (std::string("In function $symname, argument $argnum: ")+ex.what()).c_str());
  }
%enddef

// wrap the following
//MFMF missing %include <fmatvec/range.h>
//MFMF and add all %template...

// fmatvec typedefs and template instantiations
%include <fmatvec/fmatvec.h>

%template() fmatvec::Vector   <fmatvec::Ref      , int   >;
%template() fmatvec::Vector   <fmatvec::Var      , int   >;
%template() fmatvec::Vector   <fmatvec::Fixed< 1>, int   >;
%template() fmatvec::Vector   <fmatvec::Fixed< 2>, int   >;
%template() fmatvec::Vector   <fmatvec::Fixed< 3>, int   >;
%template() fmatvec::Vector   <fmatvec::Ref      , double>;
%template() fmatvec::Vector   <fmatvec::Var      , double>;
%template() fmatvec::Vector   <fmatvec::Fixed< 1>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed< 2>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed< 3>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed< 4>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed< 5>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed< 6>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed< 7>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed< 8>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed< 9>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed<10>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed<11>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed<12>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed<13>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed<14>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed<15>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed<16>, double>;
%template() fmatvec::Vector   <fmatvec::Fixed<17>, double>;
%template() fmatvec::RowVector<fmatvec::Var      , int   >;
%template() fmatvec::RowVector<fmatvec::Ref      , double>;
%template() fmatvec::RowVector<fmatvec::Var      , double>;
%template() fmatvec::RowVector<fmatvec::Fixed< 1>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed< 2>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed< 3>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed< 4>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed< 5>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed< 6>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed< 7>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed< 8>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed< 9>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed<10>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed<11>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed<12>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed<13>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed<14>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed<15>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed<16>, double>;
%template() fmatvec::RowVector<fmatvec::Fixed<17>, double>;

%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Var      , int   >;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 1>, int   >;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 2>, int   >;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 3>, int   >;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 4>, int   >;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Var      , int   >;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Var      , int   >;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Var      , int   >;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Var      , int   >;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Ref      , fmatvec::Ref      , double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Var      , double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Var      , double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Var      , double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Var      , double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Var      , double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 5>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 6>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 7>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 8>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 9>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<10>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<11>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<12>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<13>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<14>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<15>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<16>, double>;
%template() fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<17>, double>;
%template() fmatvec::Matrix<fmatvec::Rotation   , fmatvec::Fixed< 3>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Ref      , fmatvec::Ref      , double>;
%template() fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Var      , fmatvec::Var      , double>;
%template() fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Fixed< 1>, fmatvec::Fixed< 1>, double>;
%template() fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Fixed< 2>, fmatvec::Fixed< 2>, double>;
%template() fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Fixed< 3>, fmatvec::Fixed< 3>, double>;
%template() fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Fixed< 4>, fmatvec::Fixed< 4>, double>;
%template() fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Fixed< 5>, fmatvec::Fixed< 5>, double>;

//MFMF not working %template() fmatvec::Matrix<fmatvec::Sparse     , fmatvec::Ref      , fmatvec::Ref      , double>;
//MFMF not working %template() fmatvec::Matrix<fmatvec::Diagonal   , fmatvec::Ref      , fmatvec::Ref      , double>;
//MFMF not working %template() fmatvec::Matrix<fmatvec::GeneralBand, fmatvec::Ref      , fmatvec::Ref      , double>;

%template() fmatvec::SquareMatrix<fmatvec::Ref      , double>;
%template() fmatvec::SquareMatrix<fmatvec::Var      , double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed< 1>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed< 2>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed< 3>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed< 4>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed< 5>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed< 6>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed< 7>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed< 8>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed< 9>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed<10>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed<11>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed<12>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed<13>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed<14>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed<15>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed<16>, double>;
%template() fmatvec::SquareMatrix<fmatvec::Fixed<17>, double>;

// wrap output (member of function return value) as a numpy.ndarray which uses the date memory from the c++ member
%typemap(out) fmatvec::Vector*,
              fmatvec::Vector&,
              fmatvec::Vector*,
              fmatvec::Vector&,
              fmatvec::RowVector*,
              fmatvec::RowVector&,
              fmatvec::RowVector*,
              fmatvec::RowVector& {
  try {
    npy_intp dims[1];
    dims[0]=$1->size();
    $result=PyArray_SimpleNewFromData(1, dims, numPyType<$1_basetype::AtomicType>(), &(*$1)(0));
    if(!$result)
      throw std::runtime_error("Cannot create ndarray");
  }
  FMATVEC_CATCHARG
}

// wrap output (function return value) as a numpy.ndarray with its own data memory
%typemap(out) fmatvec::Vector,
              const fmatvec::Vector&,
              fmatvec::Vector,
              const fmatvec::Vector&,
              fmatvec::RowVector,
              const fmatvec::RowVector&,
              fmatvec::RowVector,
              const fmatvec::RowVector& {
  try {
    npy_intp dims[1];
    dims[0]=derefIfPointer($1).size();
    $result=PyArray_SimpleNew(1, dims, numPyType<$1_basetype::AtomicType>());
    if(!$result)
      throw std::runtime_error("Cannot create ndarray");
    std::copy(&derefIfPointer($1)(0), &derefIfPointer($1)(0)+derefIfPointer($1).size(),
              static_cast<$1_basetype::AtomicType*>(PyArray_GETPTR1(reinterpret_cast<PyArrayObject*>($result), 0)));
  }
  FMATVEC_CATCHARG
}

// add a local variable
// (we cannot use the SWIG syntax "typemap(...) TYPE (LocalVarType localVar)" here since LocalVarType depends on the template type)
%typemap(arginit) fmatvec::Vector*,
                  fmatvec::Vector&,
                  const fmatvec::Vector&,
                  fmatvec::Vector,
                  const fmatvec::Vector,
                  fmatvec::Vector*,
                  fmatvec::Vector&,
                  const fmatvec::Vector&,
                  fmatvec::Vector,
                  const fmatvec::Vector,
                  fmatvec::RowVector*,
                  fmatvec::RowVector&,
                  const fmatvec::RowVector&,
                  fmatvec::RowVector,
                  const fmatvec::RowVector,
                  fmatvec::RowVector*,
                  fmatvec::RowVector&,
                  const fmatvec::RowVector&,
                  fmatvec::RowVector,
                  const fmatvec::RowVector
"$1_basetype localVar$argnum;"

// wrap input (member or function parameter) as a numpy.ndarray with its own data memory
%typemap(in) fmatvec::Vector*,
             fmatvec::Vector&,
             const fmatvec::Vector&,
             fmatvec::Vector,
             const fmatvec::Vector,
             fmatvec::Vector*,
             fmatvec::Vector&,
             const fmatvec::Vector&,
             fmatvec::Vector,
             const fmatvec::Vector,
             fmatvec::RowVector*,
             fmatvec::RowVector&,
             const fmatvec::RowVector&,
             fmatvec::RowVector,
             const fmatvec::RowVector,
             fmatvec::RowVector*,
             fmatvec::RowVector&,
             const fmatvec::RowVector&,
             fmatvec::RowVector,
             const fmatvec::RowVector {
  try {
    void *inputp;
    int res=SWIG_ConvertPtr($input, &inputp, $1_descriptor, 0);
    if(SWIG_IsOK(res))
      assignOrCopy($1, reinterpret_cast<$1_basetype*>(inputp));
    else if(PyArray_Check($input)) {
      PyArrayObject *input=reinterpret_cast<PyArrayObject*>($input);
      if(PyArray_NDIM(input)!=1)
        throw std::runtime_error("Must have 1 dimension.");
      int type=PyArray_TYPE(input);
      checkNumPyType<$1_basetype::AtomicType>(type);
      assignIfPointer($1, localVar$argnum);
      npy_intp *dims=PyArray_SHAPE(input);
      derefIfPointer($1).resize(dims[0]);
      for(int i=0; i<dims[0]; ++i)
        derefIfPointer($1)(i)=arrayGet<$1_basetype::AtomicType>(input, type, i);
    }
    else
      throw std::runtime_error("Wrong type.");
  }
  FMATVEC_CATCHARG
}

// wrap output function parameter -> do nothing for const references (no output)
%typemap(argout) const fmatvec::Vector&,
                 const fmatvec::Vector&,
                 const fmatvec::RowVector&,
                 const fmatvec::RowVector& {
}

// wrap output function parameter -> copy result back to input function parameter
%typemap(argout) fmatvec::Vector&,
                 fmatvec::Vector&,
                 fmatvec::RowVector&,
                 fmatvec::RowVector& {
  try {
    PyArrayObject *input=reinterpret_cast<PyArrayObject*>($input);
    int type=PyArray_TYPE(input);
    if(type!=numPyType<$1_basetype::AtomicType>())
      throw std::runtime_error(std::string("Must have atomic type ")+typeid($1_basetype::AtomicType).name());
    std::copy(&localVar$argnum(0), &localVar$argnum(0)+localVar$argnum.size(),
              static_cast<$1_basetype::AtomicType*>(PyArray_GETPTR1(input, 0)));
  }
  FMATVEC_CATCHARG
}



// wrap output (member of function return value) as a numpy.ndarray which uses the date memory from the c++ member
%typemap(out) fmatvec::Matrix*,
              fmatvec::Matrix&,
              fmatvec::Matrix*,
              fmatvec::Matrix&,
              fmatvec::SquareMatrix*,
              fmatvec::SquareMatrix&,
              fmatvec::SquareMatrix*,
              fmatvec::SquareMatrix& {
  try {
    npy_intp dims[2];
    dims[0]=$1->rows();
    dims[1]=$1->cols();
    $result=PyArray_SimpleNewFromData(2, dims, numPyType<$1_basetype::AtomicType>(), &(*$1)(0, 0));
    if(!$result)
      throw std::runtime_error("Cannot create ndarray");
    npy_intp *strides=PyArray_STRIDES(reinterpret_cast<PyArrayObject*>($result));
    strides[$1->transposed() ? 1 : 0]=sizeof($1_basetype::AtomicType)*1;
    strides[$1->transposed() ? 0 : 1]=sizeof($1_basetype::AtomicType)*$1->ldim();
  }
  FMATVEC_CATCHARG
}

// wrap output (function return value) as a numpy.ndarray with its own data memory
%typemap(out) fmatvec::Matrix,
              const fmatvec::Matrix&,
              fmatvec::Matrix,
              const fmatvec::Matrix&,
              fmatvec::SquareMatrix,
              const fmatvec::SquareMatrix&,
              fmatvec::SquareMatrix,
              const fmatvec::SquareMatrix& {
  try {
    npy_intp dims[2];
    dims[0]=derefIfPointer($1).rows();
    dims[1]=derefIfPointer($1).cols();
    $result=PyArray_SimpleNew(2, dims, numPyType<$1_basetype::AtomicType>());
    if(!$result)
      throw std::runtime_error("Cannot create ndarray");
    PyArrayObject *input=reinterpret_cast<PyArrayObject*>($result);
    for(int r=0; r<dims[0]; r++)
      for(int c=0; c<dims[1]; c++)
        *static_cast<$1_basetype::AtomicType*>(PyArray_GETPTR2(input, r, c))=
          derefIfPointer($1)(r, c);
  }
  FMATVEC_CATCHARG
}

// add a local variable
// (we cannot use the SWIG syntax "typemap(...) TYPE (LocalVarType localVar)" here since LocalVarType depends on the template type)
%typemap(arginit) fmatvec::Matrix*,
                  fmatvec::Matrix&,
                  const fmatvec::Matrix&,
                  fmatvec::Matrix,
                  const fmatvec::Matrix,
                  fmatvec::Matrix*,
                  fmatvec::Matrix&,
                  const fmatvec::Matrix&,
                  fmatvec::Matrix,
                  const fmatvec::Matrix,
                  fmatvec::SquareMatrix*,
                  fmatvec::SquareMatrix&,
                  const fmatvec::SquareMatrix&,
                  fmatvec::SquareMatrix,
                  const fmatvec::SquareMatrix,
                  fmatvec::SquareMatrix*,
                  fmatvec::SquareMatrix&,
                  const fmatvec::SquareMatrix&,
                  fmatvec::SquareMatrix,
                  const fmatvec::SquareMatrix
"$1_basetype localVar$argnum;"

// wrap input (member or function parameter) as a numpy.ndarray with its own data memory
%typemap(in) fmatvec::Matrix*,
             fmatvec::Matrix&,
             const fmatvec::Matrix&,
             fmatvec::Matrix,
             const fmatvec::Matrix,
             fmatvec::Matrix*,
             fmatvec::Matrix&,
             const fmatvec::Matrix&,
             fmatvec::Matrix,
             const fmatvec::Matrix,
             fmatvec::SquareMatrix*,
             fmatvec::SquareMatrix&,
             const fmatvec::SquareMatrix&,
             fmatvec::SquareMatrix,
             const fmatvec::SquareMatrix,
             fmatvec::SquareMatrix*,
             fmatvec::SquareMatrix&,
             const fmatvec::SquareMatrix&,
             fmatvec::SquareMatrix,
             const fmatvec::SquareMatrix {
  try {
    void *inputp;
    int res=SWIG_ConvertPtr($input, &inputp, $1_descriptor, 0);
    if(SWIG_IsOK(res))
      assignOrCopy($1, reinterpret_cast<$1_basetype*>(inputp));
    else if(PyArray_Check($input)) {
      PyArrayObject *input=reinterpret_cast<PyArrayObject*>($input);
      if(PyArray_NDIM(input)!=2)
        throw std::runtime_error("Must have 2 dimension.");
      int type=PyArray_TYPE(input);
      checkNumPyType<$1_basetype::AtomicType>(type);
      assignIfPointer($1, localVar$argnum);
      npy_intp *dims=PyArray_SHAPE(input);
      derefIfPointer($1).resize(dims[0], dims[1]);
      for(int r=0; r<dims[0]; ++r)
        for(int c=0; c<dims[1]; ++c)
          derefIfPointer($1)(r, c)=arrayGet<$1_basetype::AtomicType>(input, type, r, c);
    }
    else
      throw std::runtime_error("Wrong type.");
  }
  FMATVEC_CATCHARG
}

// wrap output function parameter -> do nothing for const references (no output)
%typemap(argout) const fmatvec::Matrix&,
                 const fmatvec::Matrix&,
                 const fmatvec::SquareMatrix&,
                 const fmatvec::SquareMatrix& {
}

// wrap output function parameter -> copy result back to input function parameter
%typemap(argout) fmatvec::Matrix&,
                 fmatvec::Matrix&,
                 fmatvec::SquareMatrix&,
                 fmatvec::SquareMatrix& {
  try {
    PyArrayObject *input=reinterpret_cast<PyArrayObject*>($input);
    int type=PyArray_TYPE(input);
    if(type!=numPyType<$1_basetype::AtomicType>())
      throw std::runtime_error(std::string("Must have atomic type ")+typeid($1_basetype::AtomicType).name());
    for(int r=0; r<localVar$argnum.rows(); ++r)
      for(int c=0; c<localVar$argnum.cols(); ++c)
        *static_cast<$1_basetype::AtomicType*>(PyArray_GETPTR2(input, r, c))=
          localVar$argnum(r, c);
  }
  FMATVEC_CATCHARG
}

// add code to generated file (helper functions and includes)
%{

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include <typeinfo>

template<typename AT> void checkNumPyType(int type);
template<> void checkNumPyType<int>(int type) {
  if(type!=NPY_SHORT    && type!=NPY_USHORT    &&
     type!=NPY_INT      && type!=NPY_UINT      &&
     type!=NPY_LONG     && type!=NPY_ULONG     &&
     type!=NPY_LONGLONG && type!=NPY_ULONGLONG)
    throw std::runtime_error("Value is not of type integer.");
}
template<> void checkNumPyType<double>(int type) {
  if(type!=NPY_SHORT    && type!=NPY_USHORT    &&
     type!=NPY_INT      && type!=NPY_UINT      &&
     type!=NPY_LONG     && type!=NPY_ULONG     &&
     type!=NPY_LONGLONG && type!=NPY_ULONGLONG &&
     type!=NPY_FLOAT    && type!=NPY_DOUBLE    && type!=NPY_LONGDOUBLE)
    throw std::runtime_error("Value is not of type floating point.");
}

template<typename AT> AT arrayGet(PyArrayObject *a, int type, int r, int c=-1);
template<> int arrayGet<int>(PyArrayObject *a, int type, int r, int c) {
  switch(type) {
    case NPY_SHORT:      return *static_cast<npy_short*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_USHORT:     return *static_cast<npy_ushort*>    (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_INT:        return *static_cast<npy_int*>       (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_UINT:       return *static_cast<npy_uint*>      (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONG:       return *static_cast<npy_long*>      (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_ULONG:      return *static_cast<npy_ulong*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONGLONG:   return *static_cast<npy_longlong*>  (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_ULONGLONG:  return *static_cast<npy_ulonglong*> (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
  }
  throw std::runtime_error("Value is not of type floating point (wrong element type).");
}
template<> double arrayGet<double>(PyArrayObject *a, int type, int r, int c) {
  switch(type) {
    case NPY_SHORT:      return *static_cast<npy_short*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_USHORT:     return *static_cast<npy_ushort*>    (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_INT:        return *static_cast<npy_int*>       (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_UINT:       return *static_cast<npy_uint*>      (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONG:       return *static_cast<npy_long*>      (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_ULONG:      return *static_cast<npy_ulong*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONGLONG:   return *static_cast<npy_longlong*>  (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_ULONGLONG:  return *static_cast<npy_ulonglong*> (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_FLOAT:      return *static_cast<npy_float*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_DOUBLE:     return *static_cast<npy_double*>    (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONGDOUBLE: return *static_cast<npy_longdouble*>(c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
  }
  throw std::runtime_error("Value is not of type floating point (wrong element type).");
}

template<class T> inline T& derefIfPointer(T & t) { return  t; }
template<class T> inline T& derefIfPointer(T*& t) { return *t; }

template<class T> inline void assignIfPointer(T & d, T& s) {}
template<class T> inline void assignIfPointer(T*& d, T& s) { d=&s; }

template<class T> inline void assignOrCopy(T & d, T* s) { d=*s; }
template<class T> inline void assignOrCopy(T*& d, T* s) { d= s; }

template<typename AT> constexpr int numPyType();
template<> constexpr int numPyType<int>() { return NPY_LONG; }
template<> constexpr int numPyType<double>() { return NPY_DOUBLE; }

%}
