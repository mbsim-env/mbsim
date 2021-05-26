%module fmatvec;

// check target language
#ifndef SWIGPYTHON
  #error "Only Python as target language is supported."
#endif

%{
#include <fmatvec/range.h>
#include <fmatvec/atom.h>
%}

%import <fmatvec/types.h>
%include <fmatvec/fmatvec.h>

// helper define to convert c++ exceptions in typemaps to target language exceptions
%define FMATVEC_CATCHARG
  catch(const std::exception &ex) {
    Py_XDECREF($result);
    SWIG_exception(SWIG_RuntimeError, (std::string("In function $symname, argument $argnum: ")+ex.what()).c_str());
  }
%enddef

// wrap the following
%include <fmatvec/range.h>
%template(Index) fmatvec::Range<fmatvec::Var,fmatvec::Var>;


%import "std_string.i"



// wrap fmatvec::Atom
%feature("director") fmatvec::Atom;
namespace fmatvec {
  %extend Atom {
    void msg(const MsgType &type, const std::string &msg) {
      $self->msg(type)<<msg<<std::endl;
    }
    static void msgStatic(const MsgType &type, const std::string &msg) {
      fmatvec::Atom::msgStatic(type)<<msg<<std::endl;
    }
  };
}
%ignore fmatvec::Atom::setCurrentMessageStream;
%ignore fmatvec::Atom::setMessageStreamActive;
%ignore fmatvec::Atom::getMessageStream;
%ignore fmatvec::Atom::adoptMessageStreams;
%ignore fmatvec::Atom::msg;
%ignore fmatvec::Atom::msgStatic;
%ignore fmatvec::PrePostfixedStream;
%include <fmatvec/atom.h>


// fmatvec typedefs and template instantiations
%include <fmatvec/fmatvec.h>


%feature("valuewrapper") fmatvec::Vector   <fmatvec::Ref      , int   >;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Var      , int   >;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed< 1>, int   >;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed< 2>, int   >;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed< 3>, int   >;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Ref      , double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Var      , double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Vector   <fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Var      , int   >;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Ref      , double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Var      , double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::RowVector<fmatvec::Fixed<17>, double>;

%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Var      , int   >;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 1>, int   >;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 2>, int   >;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 3>, int   >;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 4>, int   >;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Var      , int   >;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Var      , int   >;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Var      , int   >;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Var      , int   >;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Ref      , fmatvec::Ref      , double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Var      , double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Var      , fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Var      , double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Var      , double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Var      , double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Var      , double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 1>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 2>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 3>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 4>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 5>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 6>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 7>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 8>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed< 9>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<10>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<11>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<12>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<13>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<14>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<15>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<16>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::General    , fmatvec::Fixed<17>, fmatvec::Fixed<17>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::Rotation   , fmatvec::Fixed< 3>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Ref      , fmatvec::Ref      , double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Var      , fmatvec::Var      , double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Fixed< 1>, fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Fixed< 2>, fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Fixed< 3>, fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Fixed< 4>, fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::Matrix<fmatvec::Symmetric  , fmatvec::Fixed< 5>, fmatvec::Fixed< 5>, double>;

//MISSING not working %feature("valuewrapper") fmatvec::Matrix<fmatvec::Sparse     , fmatvec::Ref      , fmatvec::Ref      , double>;
//MISSING not working %feature("valuewrapper") fmatvec::Matrix<fmatvec::Diagonal   , fmatvec::Ref      , fmatvec::Ref      , double>;
//MISSING not working %feature("valuewrapper") fmatvec::Matrix<fmatvec::GeneralBand, fmatvec::Ref      , fmatvec::Ref      , double>;

%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Ref      , double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Var      , double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed< 1>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed< 2>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed< 3>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed< 4>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed< 5>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed< 6>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed< 7>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed< 8>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed< 9>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed<10>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed<11>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed<12>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed<13>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed<14>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed<15>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed<16>, double>;
%feature("valuewrapper") fmatvec::SquareMatrix<fmatvec::Fixed<17>, double>;

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

//MISSING not working %template() fmatvec::Matrix<fmatvec::Sparse     , fmatvec::Ref      , fmatvec::Ref      , double>;
//MISSING not working %template() fmatvec::Matrix<fmatvec::Diagonal   , fmatvec::Ref      , fmatvec::Ref      , double>;
//MISSING not working %template() fmatvec::Matrix<fmatvec::GeneralBand, fmatvec::Ref      , fmatvec::Ref      , double>;

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



// VECTOR

// function return value: copy memory from fmatvec $1 to PyObject $result
%typemap(out   , noblock=1)       fmatvec::Vector ,       fmatvec::RowVector  ,
                            const fmatvec::Vector , const fmatvec::RowVector  ,
                            const fmatvec::Vector&, const fmatvec::RowVector& {
  try {
    _typemapOutVecOwnMemory<$1_basetype>($1, $result);
  }
  FMATVEC_CATCHARG
}

// function return value: use memory from fmatvec $1 in PyObject $result
%typemap(out   , noblock=1)       fmatvec::Vector*,       fmatvec::RowVector* ,
                                  fmatvec::Vector&,       fmatvec::RowVector& {
  try {
    _typemapOutVecShareMemory<$1_basetype>($1, $result);
  }
  FMATVEC_CATCHARG
}

// local fmatvec variable used for in/out function parameter
%typemap(arginit, noblock=1)       fmatvec::Vector*,       fmatvec::RowVector* ,
                                   fmatvec::Vector&,       fmatvec::RowVector& ,
                             const fmatvec::Vector&, const fmatvec::RowVector& {
  $1_basetype localVar$argnum;
}

// no local variable required for in function parameter
%typemap(arginit, noblock=1)       fmatvec::Vector ,       fmatvec::RowVector  ,
                             const fmatvec::Vector , const fmatvec::RowVector  {
}

// function in parameter value: copy memory from PyObject $input to fmatvec $1
%typemap(in    , noblock=1)       fmatvec::Vector ,       fmatvec::RowVector  ,
                            const fmatvec::Vector , const fmatvec::RowVector  {
  try {
    _typemapInVecValue<$1_basetype>($1, $input, $1_descriptor);
  }
  FMATVEC_CATCHARG
}

// function in/out (and const reference) parameter value: copy memory from PyObject $input to fmatvec localVar$argnum
%typemap(in    , noblock=1)       fmatvec::Vector*,       fmatvec::RowVector* ,
                                  fmatvec::Vector&,       fmatvec::RowVector& ,
                            const fmatvec::Vector&, const fmatvec::RowVector& {
  try {
    _typemapInVecPtr<$1_basetype>($1, $input, $1_descriptor, localVar$argnum);
  }
  FMATVEC_CATCHARG
}

// no argout required for function in parameter
%typemap(argout, noblock=1)       fmatvec::Vector ,       fmatvec::RowVector  ,
                            const fmatvec::Vector , const fmatvec::RowVector  ,
                            const fmatvec::Vector&, const fmatvec::RowVector& {
}

// function out parameter value: copy memory from fmatvec localVar$argnum to PyObject $input
%typemap(argout, noblock=1)       fmatvec::Vector&,       fmatvec::RowVector& ,
                                  fmatvec::Vector*,       fmatvec::RowVector* {
  try {
    _typemapArgoutVec<$1_basetype>($1, $input, $1_descriptor, localVar$argnum);
  }
  FMATVEC_CATCHARG
}



// MATRIX

// function return value: copy memory from fmatvec $1 to PyObject $result
%typemap(out   , noblock=1)       fmatvec::Matrix ,       fmatvec::SquareMatrix  ,
                            const fmatvec::Matrix , const fmatvec::SquareMatrix  ,
                            const fmatvec::Matrix&, const fmatvec::SquareMatrix& {
  try {
    _typemapOutMatOwnMemory<$1_basetype>($1, $result);
  }
  FMATVEC_CATCHARG
}

// function return value: use memory from fmatvec $1 in PyObject $result
%typemap(out   , noblock=1)       fmatvec::Matrix*,       fmatvec::SquareMatrix* ,
                                  fmatvec::Matrix&,       fmatvec::SquareMatrix& {
  try {
    _typemapOutMatShareMemory<$1_basetype>($1, $result);
  }
  FMATVEC_CATCHARG
}

// local fmatvec variable used for in/out function parameter
%typemap(arginit, noblock=1)       fmatvec::Matrix*,       fmatvec::SquareMatrix* ,
                                   fmatvec::Matrix&,       fmatvec::SquareMatrix& ,
                             const fmatvec::Matrix&, const fmatvec::SquareMatrix& {
  $1_basetype localVar$argnum;
}

// no local variable required for in function parameter
%typemap(arginit, noblock=1)       fmatvec::Matrix ,       fmatvec::SquareMatrix  ,
                             const fmatvec::Matrix , const fmatvec::SquareMatrix  {
}

// function in parameter value: copy memory from PyObject $input to fmatvec $1
%typemap(in    , noblock=1)       fmatvec::Matrix ,       fmatvec::SquareMatrix  ,
                            const fmatvec::Matrix , const fmatvec::SquareMatrix  {
  try {
    _typemapInMatValue<$1_basetype>($1, $input, $1_descriptor);
  }
  FMATVEC_CATCHARG
}

// function in/out (and const reference) parameter value: copy memory from PyObject $input to fmatvec localVar$argnum
%typemap(in    , noblock=1)       fmatvec::Matrix*,       fmatvec::SquareMatrix* ,
                                  fmatvec::Matrix&,       fmatvec::SquareMatrix& ,
                            const fmatvec::Matrix&, const fmatvec::SquareMatrix& {
  try {
    _typemapInMatPtr<$1_basetype>($1, $input, $1_descriptor, localVar$argnum);
  }
  FMATVEC_CATCHARG
}

// no argout required for function in parameter
%typemap(argout, noblock=1)       fmatvec::Matrix ,       fmatvec::SquareMatrix  ,
                            const fmatvec::Matrix , const fmatvec::SquareMatrix  ,
                            const fmatvec::Matrix&, const fmatvec::SquareMatrix& {
}

// function out parameter value: copy memory from fmatvec localVar$argnum to PyObject $input
%typemap(argout, noblock=1)       fmatvec::Matrix&,       fmatvec::SquareMatrix& ,
                                  fmatvec::Matrix*,       fmatvec::SquareMatrix* {
  try {
    _typemapArgoutMat<$1_basetype>($1, $input, $1_descriptor, localVar$argnum);
  }
  FMATVEC_CATCHARG
}
