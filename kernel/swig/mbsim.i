// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") mbsim

// The following part is only used to generate a list of unwrapped classes.
// When this is enabled swig will fail but output all classes with are unwrapped.
%include "showUnwrappedClasses.i" 
#ifdef SHOW_UNWRAPPED_CLASSES
  // list here classes with should not be wrapped (these are remove from the list of unwrapped classes)
  WRAPPED_CLASS(MBXMLUtils::FQN)
#endif


// BEGIN: The following code block is special to MBSim kernel this is not required by any .i file for a MBSim module

// include the general mbsim SWIG configuration (used by all MBSim modules)
%include "mbsim_include.i"
%import fmatvec.i

%typemap(directorin) xercesc::DOMElement* %{
  MapPyXercesDOMElement mapPyXercesDOMElement$argnum;
  _typemapDirectorinDOMElement($1, $input, mapPyXercesDOMElement$argnum);
%}

%typemap(in) xercesc::DOMElement* {
  try {
    _typemapInDOMElement($1, $input);
  }
  FMATVEC_CATCHARG
}

// create directors for everything
%feature("director");

// END: The following code block is special to MBSim kernel this is not required by any .i file for a MBSim module

// includes needed in the generated swig c++ code
%{
#include <config.h> // to use consistent preprocessor defines
#include "mbsim/links/frame_link.h"
#include "mbsim/frames/frame.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/frames/fixed_contour_frame.h"
#include "mbsim/frames/floating_contour_frame.h"
#include "mbsim/frames/floating_relative_contour_frame.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/frames/floating_relative_frame.h"
#include "mbsim/environment.h"
#include "mbsim/observers/cartesian_coordinates_observer.h"
#include "mbsim/observers/cylinder_coordinates_observer.h"
#include "mbsim/observers/natural_coordinates_observer.h"
#include "mbsim/observers/absolute_kinematics_observer.h"
#include "mbsim/observers/relative_kinematics_observer.h"
#include "mbsim/observers/rigid_body_group_observer.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/contact.h"
#include "mbsim/links/contour_link.h"
#include "mbsim/links/floating_frame_link.h"
#include "mbsim/links/directional_spring_damper.h"
#include "mbsim/links/isotropic_rotational_spring_damper.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/kinetic_excitation.h"
#include "mbsim/links/frame_link.h"
#include "mbsim/links/spring_damper.h"
#include "mbsim/links/rigid_body_link.h"
#include "mbsim/links/gear.h"
#include "mbsim/links/generalized_friction.h"
#include "mbsim/links/generalized_spring_damper.h"
#include "mbsim/links/kinematic_excitation.h"
#include "mbsim/links/generalized_acceleration_excitation.h"
#include "mbsim/links/generalized_position_excitation.h"
#include "mbsim/links/generalized_velocity_excitation.h"
#include "mbsim/constraints/gear_constraint.h"
#include "mbsim/constraints/joint_constraint.h"
#include "mbsim/constraints/kinematic_constraint.h"
#include "mbsim/constraints/generalized_acceleration_constraint.h"
#include "mbsim/constraints/generalized_position_constraint.h"
#include "mbsim/constraints/generalized_velocitiy_constraint.h"
#include "mbsim/contours/contour_interpolation.h"
#include "mbsim/contours/contour_quad.h"
#include "mbsim/contours/rigid_contour.h"
#include "mbsim/contours/circle.h"
#include "mbsim/contours/compound_contour.h"
#include "mbsim/contours/cuboid.h"
#include "mbsim/contours/room.h"
#include "mbsim/contours/edge.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/line_segment.h"
#include "mbsim/contours/planar_contour.h"
#include "mbsim/contours/planar_frustum.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/plate.h"
#include "mbsim/contours/planewithfrustum.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/polynomial_frustum.h"
#include "mbsim/contours/spatial_contour.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/graph.h"
#include "mbsim/group.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/constitutive_laws/friction_force_law.h"
#include "mbsim/constitutive_laws/planar_coulomb_friction.h"
#include "mbsim/constitutive_laws/planar_stribeck_friction.h"
#include "mbsim/constitutive_laws/regularized_planar_friction.h"
#include "mbsim/constitutive_laws/regularized_spatial_friction.h"
#include "mbsim/constitutive_laws/spatial_coulomb_friction.h"
#include "mbsim/constitutive_laws/spatial_stribeck_friction.h"
#include "mbsim/constitutive_laws/friction_impact_law.h"
#include "mbsim/constitutive_laws/planar_coulomb_impact.h"
#include "mbsim/constitutive_laws/planar_stribeck_impact.h"
#include "mbsim/constitutive_laws/spatial_coulomb_impact.h"
#include "mbsim/constitutive_laws/spatial_stribeck_impact.h"
#include "mbsim/constitutive_laws/generalized_force_law.h"
#include "mbsim/constitutive_laws/bilateral_constraint.h"
#include "mbsim/constitutive_laws/regularized_bilateral_constraint.h"
#include "mbsim/constitutive_laws/regularized_unilateral_constraint.h"
#include "mbsim/constitutive_laws/unilateral_constraint.h"
#include "mbsim/constitutive_laws/generalized_impact_law.h"
#include "mbsim/constitutive_laws/bilateral_impact.h"
#include "mbsim/constitutive_laws/unilateral_newton_impact.h"
#include "mbsim/contact_kinematics/contact_kinematics.h"
#include "mbsim/contact_kinematics/circle_circle.h"
#include "mbsim/contact_kinematics/circle_extrusion.h"
#include "mbsim/contact_kinematics/circle_frustum.h"
#include "mbsim/contact_kinematics/circle_line.h"
#include "mbsim/contact_kinematics/circle_linesegment.h"
#include "mbsim/contact_kinematics/circle_planarcontour.h"
#include "mbsim/contact_kinematics/circle_planarfrustum.h"
#include "mbsim/contact_kinematics/circle_plane.h"
#include "mbsim/contact_kinematics/compoundcontour_compoundcontour.h"
#include "mbsim/contact_kinematics/compoundcontour_contour.h"
#include "mbsim/contact_kinematics/edge_edge.h"
#include "mbsim/contact_kinematics/line_planarcontour.h"
#include "mbsim/contact_kinematics/plate_polynomialfrustum.h"
#include "mbsim/contact_kinematics/point_circle.h"
#include "mbsim/contact_kinematics/point_contourinterpolation.h"
#include "mbsim/contact_kinematics/point_extrusion.h"
#include "mbsim/contact_kinematics/point_frustum.h"
#include "mbsim/contact_kinematics/point_line.h"
#include "mbsim/contact_kinematics/point_linesegment.h"
#include "mbsim/contact_kinematics/point_planarcontour.h"
#include "mbsim/contact_kinematics/point_plane.h"
#include "mbsim/contact_kinematics/point_planewithfrustum.h"
#include "mbsim/contact_kinematics/point_plate.h"
#include "mbsim/contact_kinematics/point_polynomialfrustum.h"
#include "mbsim/contact_kinematics/point_spatialcontour.h"
#include "mbsim/contact_kinematics/point_sphere.h"
#include "mbsim/contact_kinematics/sphere_frustum.h"
#include "mbsim/contact_kinematics/sphere_plane.h"
#include "mbsim/contact_kinematics/sphere_plate.h"
#include "mbsim/contact_kinematics/sphere_polynomialfrustum.h"
#include "mbsim/contact_kinematics/sphere_sphere.h"
using namespace MBSim; // SWIGs namespace handling seems to be buggy -> this fixes this
using namespace fmatvec; // SWIGs namespace handling seems to be buggy -> this fixes this
%}

// wrap some std::vector<...> types used by the above wrapped classes
%ignore swigignore;
%template(VectorElement) std::vector<MBSim::Element*>;
%template(VectorString)  std::vector<std::string>;
//MFMF%template(VectorMat)     std::vector<fmatvec::Mat>;
//MFMF%template(VectorVec)     std::vector<fmatvec::Vec>;
%template(VectorFrame)   std::vector<MBSim::Frame*>;



// wrap the following classes
%include "mbsim/element.h"



// Wrap MBSim::Function<Sig>
// SWIG cannot handle template partial specializations of the form MBSim::Function<double(int)>.
// Hence, we need to wrap it specially:

// wrap function.h -> this only wraps MBSim::FunctionBase (all others are templates)
%include "mbsim/functions/function.h"

// Define a SWIG macro for MBSim::Function<Ret(Arg)> with also renames it to allowed SWIG name.
// This definition must be keept in sync with the definition in mbsim/functions/function.h
%define FUNCTION1(Ret, Arg, namePostfix)
%rename(Function_##namePostfix) MBSim::Function<Ret(Arg)>;
class MBSim::Function<Ret(Arg)> : public MBSim::FunctionBase, virtual public fmatvec::Atom {
  public:
    Function();
    void initializeUsingXML(xercesc::DOMElement *element);
    typedef typename Der<Ret, Arg>::type DRetDArg;
    typedef typename Der<DRetDArg, Arg>::type DDRetDDArg;
    enum { retSize1 = StaticSize<Ret>::size1, retSize2 = StaticSize<Ret>::size2 };
    static constexpr int argSize = StaticSize<Arg>::size1;
    virtual std::pair<int, int> getRetSize() const;
    virtual int getArgSize() const;
    virtual Ret operator()(const Arg &arg)=0;
    virtual DRetDArg parDer(const Arg &arg);
    virtual Ret dirDer(const Arg &argDir, const Arg &arg);
    virtual DDRetDDArg parDerParDer(const Arg &arg);
    virtual DRetDArg parDerDirDer(const Arg &argDir, const Arg &arg);
    virtual Ret dirDerDirDer(const Arg &argDir_1, const Arg &argDir_2, const Arg &arg);
    virtual bool constParDer() const;
};
%enddef

// instantiate MBSim::Function<Ret(Arg)> for different types
FUNCTION1(double          , double       ,   d_d)
FUNCTION1(double          , int          ,   d_int)
FUNCTION1(fmatvec::MatV   , fmatvec::VecV,   MatV_VecV)
FUNCTION1(fmatvec::RotMat3, double       ,   RotMat3_d)
FUNCTION1(fmatvec::RotMat3, fmatvec::VecV,   RotMat3_VecV)
FUNCTION1(fmatvec::SqrMat , fmatvec::Vec ,   SqrMat_Vec)
FUNCTION1(fmatvec::SqrMatV, double       ,   SqrMatV_d)
FUNCTION1(fmatvec::SymMatV, double       ,   SymMatV_d)
FUNCTION1(fmatvec::Vec1   , double       ,   Vec1_d)
FUNCTION1(fmatvec::Vec2   , double       ,   Vec2_d)
FUNCTION1(fmatvec::Vec3   , double       ,   Vec3_d)
FUNCTION1(fmatvec::Vec3   , fmatvec::Vec2,   Vec3_Vec2)
FUNCTION1(fmatvec::Vec3   , fmatvec::VecV,   Vec3_VecV)
FUNCTION1(fmatvec::Vec    , fmatvec::Vec ,   Vec_Vec)
FUNCTION1(fmatvec::VecV   , double       ,   VecV_d)
FUNCTION1(fmatvec::VecV   , fmatvec::VecV,   VecV_VecV)
FUNCTION1(int             , fmatvec::Vec ,   int_Vec)
FUNCTION1(fmatvec::Vec    , double       ,   Vec_d)

// Define a SWIG macro for MBSim::Function<Ret(Arg1,Arg2)> with also renames it to allowed SWIG name.
// This definition must be keept in sync with the definition in mbsim/functions/function.h
%define FUNCTION2(Ret, Arg1, Arg2, namePostfix)
%rename(Function_##namePostfix) MBSim::Function<Ret(Arg1,Arg2)>;
class MBSim::Function<Ret(Arg1, Arg2)> : public MBSim::FunctionBase, virtual public fmatvec::Atom {
  public:
    Function();
    void initializeUsingXML(xercesc::DOMElement *element);
    typedef typename Der<Ret, Arg1>::type DRetDArg1;
    typedef typename Der<Ret, Arg2>::type DRetDArg2;
    typedef typename Der<DRetDArg1, Arg1>::type DDRetDDArg1;
    typedef typename Der<DRetDArg2, Arg2>::type DDRetDDArg2;
    typedef typename Der<DRetDArg1, Arg2>::type DDRetDArg1DArg2;
    enum { retSize1 = StaticSize<Ret>::size1, retSize2 = StaticSize<Ret>::size2 };
    static constexpr int arg1Size = StaticSize<Arg1>::size1;
    static constexpr int arg2Size = StaticSize<Arg2>::size1;
    virtual std::pair<int, int> getRetSize() const;
    virtual int getArg1Size() const;
    virtual int getArg2Size() const;
    virtual Ret operator()(const Arg1 &arg1, const Arg2 &arg2)=0;
    virtual DRetDArg1 parDer1(const Arg1 &arg1, const Arg2 &arg2);
    virtual Ret dirDer1(const Arg1 &arg1Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDArg2 parDer2(const Arg1 &arg1, const Arg2 &arg2);
    virtual Ret dirDer2(const Arg2 &arg2Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual DDRetDDArg1 parDer1ParDer1(const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDArg1 parDer1DirDer1(const Arg1 &arg1Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual Ret dirDer1DirDer1(const Arg1 &arg1Dir_1, const Arg1 &arg1Dir_2, const Arg1 &arg1, const Arg2 &arg2);
    virtual DDRetDDArg2 parDer2ParDer2(const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDArg2 parDer2DirDer2(const Arg2 &arg2Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual Ret dirDer2DirDer2(const Arg2 &arg2Dir_1, const Arg2 &arg2Dir_2, const Arg1 &arg1, const Arg2 &arg2);
    virtual DDRetDArg1DArg2 parDer1ParDer2(const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDArg1 parDer1DirDer2(const Arg2 &arg2Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual Ret dirDer2DirDer1(const Arg2 &arg1Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDArg2 parDer2DirDer1(const Arg1 &arg1Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual bool constParDer1() const;
    virtual bool constParDer2() const;
};
%enddef

// instantiate MBSim::Function<Ret(Arg1,Arg2)> for different types
FUNCTION2(double          , double       , double       ,   d_d_d)
FUNCTION2(double          , fmatvec::Vec , fmatvec::Vec ,   d_Vec_Vec)
FUNCTION2(fmatvec::RotMat3, fmatvec::VecV, double       ,   RotMat3_VecV_d)
FUNCTION2(fmatvec::Vec3   , fmatvec::VecV, double       ,   Vec3_VecV_d)
FUNCTION2(fmatvec::Vec    , fmatvec::Vec , double       ,   Vec_Vec_d)
FUNCTION2(fmatvec::VecV   , fmatvec::VecV, double       ,   VecV_VecV_d)
FUNCTION2(fmatvec::VecV   , fmatvec::VecV, fmatvec::VecV,   VecV_VecV_VecV)



// wrap the following classes
%rename(lambda_) MBSim::Link::lambda; // lambda is a python keyword -> rename it to lambda_
%include "mbsim/links/link.h"
%include "mbsim/links/frame_link.h"
%include "mbsim/frames/frame.h"
%include "mbsim/frames/contour_frame.h"
%include "mbsim/frames/fixed_contour_frame.h"
%include "mbsim/frames/floating_contour_frame.h"
%include "mbsim/frames/floating_relative_contour_frame.h"
%include "mbsim/frames/fixed_relative_frame.h"
%include "mbsim/frames/floating_relative_frame.h"
%include "mbsim/environment.h"
%include "mbsim/observers/observer.h"
%include "mbsim/observers/coordinates_observer.h"
%include "mbsim/observers/cartesian_coordinates_observer.h"
%include "mbsim/observers/cylinder_coordinates_observer.h"
%include "mbsim/observers/natural_coordinates_observer.h"
%include "mbsim/observers/kinematics_observer.h"
%include "mbsim/observers/absolute_kinematics_observer.h"
%include "mbsim/observers/relative_kinematics_observer.h"
%include "mbsim/observers/rigid_body_group_observer.h"
%include "mbsim/objects/object.h"
%include "mbsim/objects/body.h"
%include "mbsim/objects/rigid_body.h"
%include "mbsim/links/link.h"
%include "mbsim/links/contact.h"
%include "mbsim/links/contour_link.h"
%include "mbsim/links/floating_frame_link.h"
%include "mbsim/links/directional_spring_damper.h"
%include "mbsim/links/isotropic_rotational_spring_damper.h"
%include "mbsim/links/joint.h"
%include "mbsim/links/kinetic_excitation.h"
%include "mbsim/links/frame_link.h"
%include "mbsim/links/spring_damper.h"
%include "mbsim/links/rigid_body_link.h"
%include "mbsim/links/gear.h"
%include "mbsim/links/generalized_friction.h"
%include "mbsim/links/generalized_spring_damper.h"
%include "mbsim/links/kinematic_excitation.h"
%include "mbsim/links/generalized_acceleration_excitation.h"
%include "mbsim/links/generalized_position_excitation.h"
%include "mbsim/links/generalized_velocity_excitation.h"
%include "mbsim/constraints/constraint.h"
%include "mbsim/constraints/gear_constraint.h"
%include "mbsim/constraints/joint_constraint.h"
%include "mbsim/constraints/kinematic_constraint.h"
%include "mbsim/constraints/generalized_acceleration_constraint.h"
%include "mbsim/constraints/generalized_position_constraint.h"
%include "mbsim/constraints/generalized_velocitiy_constraint.h"
%include "mbsim/contours/contour.h"
%include "mbsim/contours/contour_interpolation.h"
%include "mbsim/contours/contour_quad.h"
%include "mbsim/contours/rigid_contour.h"
%include "mbsim/contours/circle.h"
%include "mbsim/contours/compound_contour.h"
%include "mbsim/contours/cuboid.h"
%include "mbsim/contours/room.h"
%include "mbsim/contours/edge.h"
%include "mbsim/contours/frustum.h"
%include "mbsim/contours/line.h"
%include "mbsim/contours/line_segment.h"
%include "mbsim/contours/planar_contour.h"
%include "mbsim/contours/planar_frustum.h"
%include "mbsim/contours/plane.h"
%include "mbsim/contours/plate.h"
%include "mbsim/contours/planewithfrustum.h"
%include "mbsim/contours/point.h"
%include "mbsim/contours/polynomial_frustum.h"
%include "mbsim/contours/spatial_contour.h"
%include "mbsim/contours/sphere.h"
%include "mbsim/dynamic_system.h"
%include "mbsim/graph.h"
%include "mbsim/group.h"
%rename(global_) MBSim::DynamicSystemSolver::global; // global is a python keyword -> rename it to global_
%include "mbsim/dynamic_system_solver.h"
%include "mbsim/constitutive_laws/friction_force_law.h"
%include "mbsim/constitutive_laws/planar_coulomb_friction.h"
%include "mbsim/constitutive_laws/planar_stribeck_friction.h"
%include "mbsim/constitutive_laws/regularized_planar_friction.h"
%include "mbsim/constitutive_laws/regularized_spatial_friction.h"
%include "mbsim/constitutive_laws/spatial_coulomb_friction.h"
%include "mbsim/constitutive_laws/spatial_stribeck_friction.h"
%include "mbsim/constitutive_laws/friction_impact_law.h"
%include "mbsim/constitutive_laws/planar_coulomb_impact.h"
%include "mbsim/constitutive_laws/planar_stribeck_impact.h"
%include "mbsim/constitutive_laws/spatial_coulomb_impact.h"
%include "mbsim/constitutive_laws/spatial_stribeck_impact.h"
%include "mbsim/constitutive_laws/generalized_force_law.h"
%include "mbsim/constitutive_laws/bilateral_constraint.h"
%include "mbsim/constitutive_laws/regularized_bilateral_constraint.h"
%include "mbsim/constitutive_laws/regularized_unilateral_constraint.h"
%include "mbsim/constitutive_laws/unilateral_constraint.h"
%include "mbsim/constitutive_laws/generalized_impact_law.h"
%include "mbsim/constitutive_laws/bilateral_impact.h"
%include "mbsim/constitutive_laws/unilateral_newton_impact.h"
%include "mbsim/contact_kinematics/contact_kinematics.h"
%include "mbsim/contact_kinematics/circle_circle.h"
%include "mbsim/contact_kinematics/circle_extrusion.h"
%include "mbsim/contact_kinematics/circle_frustum.h"
%include "mbsim/contact_kinematics/circle_line.h"
%include "mbsim/contact_kinematics/circle_linesegment.h"
%include "mbsim/contact_kinematics/circle_planarcontour.h"
%include "mbsim/contact_kinematics/circle_planarfrustum.h"
%include "mbsim/contact_kinematics/circle_plane.h"
%include "mbsim/contact_kinematics/compoundcontour_compoundcontour.h"
%include "mbsim/contact_kinematics/compoundcontour_contour.h"
%include "mbsim/contact_kinematics/edge_edge.h"
%include "mbsim/contact_kinematics/line_planarcontour.h"
%include "mbsim/numerics/functions/criteria_functions.h"
%include "mbsim/contact_kinematics/plate_polynomialfrustum.h"
%include "mbsim/contact_kinematics/point_circle.h"
%include "mbsim/contact_kinematics/point_contourinterpolation.h"
%include "mbsim/contact_kinematics/point_extrusion.h"
%include "mbsim/contact_kinematics/point_frustum.h"
%include "mbsim/contact_kinematics/point_line.h"
%include "mbsim/contact_kinematics/point_linesegment.h"
%include "mbsim/contact_kinematics/point_planarcontour.h"
%include "mbsim/contact_kinematics/point_plane.h"
%include "mbsim/contact_kinematics/point_planewithfrustum.h"
%include "mbsim/contact_kinematics/point_plate.h"
%include "mbsim/numerics/functions/newton_method_jacobian_functions.h"
%include "mbsim/contact_kinematics/point_polynomialfrustum.h"
%include "mbsim/contact_kinematics/point_spatialcontour.h"
%include "mbsim/contact_kinematics/point_sphere.h"
%include "mbsim/contact_kinematics/sphere_frustum.h"
%include "mbsim/contact_kinematics/sphere_plane.h"
%include "mbsim/contact_kinematics/sphere_plate.h"
%include "mbsim/contact_kinematics/sphere_polynomialfrustum.h"
%include "mbsim/contact_kinematics/sphere_sphere.h"



//////////////////////////////////////////////////////////////////////////////////////////////////////////
// The following code is special to MBSim kernel this is not required by any .i file for a MBSim module //
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// disable warning 473
#pragma SWIG nowarn=SWIGWARN_TYPEMAP_DIRECTOROUT_PTR

%import openmbvcppinterface/OpenMBV.i


// wrap python error to c++ exception
%feature("director:except") %{
  _directorExcept($error);
%}

// wrap c++ exception to python error
%exception %{
  try {
    $action
  }
  catch(const std::exception &e) {
    PyErr_SetString(PyExc_MemoryError, e.what());
    SWIG_fail;
  }
  catch(...) {
    PyErr_SetString(PyExc_MemoryError, "Unknown c++ exception");
    SWIG_fail;
  }
%}

// includes needed in the generated swig c++ code
%{
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/join.hpp>
#include <xercesc/dom/DOMProcessingInstruction.hpp>
%}



// wrap part of the MBSim object factory
%include "mbsim/objectfactory_part.h"



%inline %{

// internal helper function create a MBXMLUtils::FQN (MBXMLUtils is not wrapped at all)
MBXMLUtils::FQN _FQN(const std::string &ns, const std::string &name) {
  return MBXMLUtils::FQN(ns, name);
}

// internal helper function to cast a MBSim::AllocateBase to the Python director object
// or None if this MBSim::AllocateBase does not represent a director
PyObject* _dynamic_cast_Director(const MBSim::AllocateBase *x) {
  const Swig::Director *d=dynamic_cast<const Swig::Director*>(x);
  PyObject* ret=d ? d->swig_get_self() : Py_None;
  Py_INCREF(ret);
  return ret;
}

// Internal helper function to get the processing instruction of a pyScript element.
// This is required since xml.etree.cElementTree skip all comments and processing instruction when parsing!
std::string _getPyScriptProcessingInstruction(xercesc::DOMElement *e, const std::string &ns) {
  xercesc::DOMElement *pys=MBXMLUtils::E(e)->getFirstElementChildNamed(MBXMLUtils::FQN(ns, "pyScript"));
  if(!pys)
    return "";
  xercesc::DOMProcessingInstruction *pi=MBXMLUtils::E(pys)->getFirstProcessingInstructionChildNamed("ScriptParameter");
  if(!pi)
    return "";
  return MBXMLUtils::X()%pi->getData();
}

%}



%pythoncode %{

# internal helper class to register a director class in the MBSim::ObjectFactory
class _AllocatePython(AllocateBase):
  def __init__(self, className):
    super(_AllocatePython, self).__init__() 
    self.className=className
  def __call__(self):
    return self.className().__disown__()
  def __eq__(self, other):
    otherDirector=_dynamic_cast_Director(other)
    if otherDirector==None:
      return False
    if type(otherDirector)==_AllocatePython and otherDirector.className==self.className:
      return True
    return False;

# internal helper class to register a director class in the MBSim::ObjectFactory
class _DeallocatePython(DeallocateBase):
  def __init__(self):
    super(_DeallocatePython, self).__init__() 
  def __call__(self, e):
    del e

# internal helper class to register a director class in the MBSim::ObjectFactory
class _GetSingletonPython(AllocateBase):
  def __init__(self, className):
    super(_GetSingletonPython, self).__init__() 
    self.className=className
  def __call__(self):
    return self.classNname.getInstance().__disown__()
  def __eq__(self, other):
    otherDirector=_dynamic_cast_Director(other)
    if otherDirector==None:
      return False
    if type(otherDirector)==_GetSingletonPython and otherDirector.className==self.className:
      return True
    return False;

# internal helper class to register a director class in the MBSim::ObjectFactory
class _DeallocateSingletonPython(DeallocateBase):
  def __init__(self):
    super(_DeallocateSingletonPython, self).__init__() 
  def __call__(self, e):
    pass

# get namespace of moudle moudleName (sourounded with { and } for the python moudle moduleName
def _getNSOf(moduleName):
  import sys
  ns=sys.modules[moduleName].NS
  return ns[1:-1]

_moduleData={}

# fix local xml name (remove template)
def _fixXMLLocalName(name):
  c=name.find("_")
  if c>0:
    return name[0:c]
  return name

# internal helper function to extend a Python class
def _extendClass(className):
  import xml.etree.cElementTree as ET
  # create XML schema element and store it in _moduleData
  # store also all required modules in _moduleData
  allBaseClassName=className.__bases__
  if len(allBaseClassName)!=1:
    raise RuntimeError('Can only handle classed with one base class.')
  baseClassName=allBaseClassName[0]
  if not hasattr(className, 'getSchema'):
    # no getSchema method -> empty XML
    xsdpart=None
  else:
    # getSchema method defined -> use the returned Schema part
    xsdpart=className.getSchema()
  xsd1=ET.Element(XS+"element", {"name": ET.QName(_getNSOf(className.__module__), className.__name__),
                                 "substitutionGroup": ET.QName(_getNSOf(baseClassName.__module__),
                                                               _fixXMLLocalName(baseClassName.__name__)),
                                 "type": ET.QName(_getNSOf(className.__module__), className.__name__+"Type")})
  xsd2=ET.Element(XS+"complexType", {"name": ET.QName(_getNSOf(className.__module__), className.__name__+"Type")})
  cc=ET.Element(XS+"complexContent")
  xsd2.append(cc)
  ext=ET.Element(XS+"extension", {'base': ET.QName(_getNSOf(baseClassName.__module__),
                                                   _fixXMLLocalName(baseClassName.__name__)+'Type')})
  cc.append(ext)
  if xsdpart!=None:
    ext.append(xsdpart)
  xsd=[xsd1, xsd2]
  global _moduleData
  if not className.__module__ in _moduleData:
    _moduleData[className.__module__]={'xsdElements': [], 'requiredModules': set()}
  _moduleData[className.__module__]['xsdElements'].extend(xsd)
  _moduleData[className.__module__]['requiredModules'].add(baseClassName.__module__)






# register class in the MBSim::ObjectFactory.
# This also extents className with some special members.
def registerClass(className):
  _extendClass(className)
  registerClass_internal(_FQN(_getNSOf(className.__module__), _fixXMLLocalName(className.__name__)),
    _AllocatePython(className).__disown__(), _DeallocatePython().__disown__())

# register singelton class in the MBSim::ObjectFactory
# This also extents className with some special members.
def registerClassAsSingleton(className):
  _extendClass(className)
  registerClass_internal(_FQN(_getNSOf(className.__module__), _fixXMLLocalName(className.__name__)),
    _GetSingletonPython(className).__disown__(), _DeallocateSingletonPython().__disown__())

# create the xsd for this module.
def generateXMLSchemaFile(moduleName):
  import os.path
  import xml.etree.cElementTree as ET
  global _moduleData

  # register namespace/prefix mappings
  ET.register_namespace("pv", PV[1:-1])
  ET.register_namespace("xs", XS[1:-1])
  ET.register_namespace("", _getNSOf(moduleName))
  for module in _moduleData[moduleName]['requiredModules']:
    ET.register_namespace(module, _getNSOf(module))
  # create xsd file
  xsd=ET.Element(XS+"schema", {"targetNamespace": _getNSOf(moduleName),
                               "elementFormDefault": "qualified",
                               "attributeFormDefault": "unqualified"})
  xsd.append(ET.Element(XS+"import", {"namespace": PV[1:-1]}))
  for module in _moduleData[moduleName]['requiredModules']:
    xsd.append(ET.Element(XS+"import", {'namespace': _getNSOf(module)}))
  xsd.extend(_moduleData[moduleName]['xsdElements'])
  return xsd

# create XML schema for a pyScript element
def pyScriptSchema():
  import xml.etree.cElementTree as ET
  xsdpart=ET.Element(XS+"sequence")
  xsdpart.append(ET.Element(XS+"element", {'name': 'pyScript', 'minOccurs': '0',
                                           'type': ET.QName(PV+'script')}))
  return xsdpart

# initializeUsingXML for a pyScript element
def pyScriptInitializeUsingXML(self, e, className):
  import xml.etree.cElementTree as ET
  import re
  import numpy
  super(className, self).initializeUsingXML(e)
  pys=e.find("{"+_getNSOf(className.__module__)+"}"+"pyScript")
  code=pys.text
  # fix python indentation
  lines=code.split("\n") # split to a vector of lines
  indent=-1
  firstNoneSpaceRE=re.compile(r'[^ ]')
  for lineNr, l in enumerate(lines):
    m=firstNoneSpaceRE.search(l) # get first none space character
    if m==None: continue # not found -> pure empty line -> do not modify
    pos=m.start()
    if l[pos]=='#': continue # found and first char is '#' -> pure comment line -> do not modify
    # now we have a line with a python statement
    if indent==-1: indent=pos # at the first python statement line use the current indent as indent for all others
    if l[0:indent]!=' '*indent: # check if line starts with at least indent spaces ...
      # ... if not its an indentation error
      raise RuntimeError("Unexpected indentation at line "+str(lineNr)+": "+code);
    lines[lineNr]=l[indent:] # remove the first indent spaces from the line
  code="\n".join(lines) # join the lines to a single string
  # read the processing instruction <?scriptParameter ...?> with the parameters
  globals={}
  # we cannot use python here, see _getPyScriptProcessingInstruction
  parstr=_getPyScriptProcessingInstruction(e, _getNSOf(className.__module__))
  scalarRE=re.compile(r"^scalar:([_a-zA-Z0-9]+)=(.*)$")
  vectorRE=re.compile(r"^vector:([_a-zA-Z0-9]+)=\[(.*)\]$")
  matrixRE=re.compile(r"^matrix:([_a-zA-Z0-9]+)=\[(.*)\]$")
  stringRE=re.compile(r"^string:([_a-zA-Z0-9]+)='(.*)'$")
  for line in parstr.split("\n"): # loop over all varibles (one is listed per line)
    line=line.strip() # remove leading/trailing spaces
    if line=="": continue # skip empty lines
    # handles scalars -> return as float
    if line.startswith("scalar:"):
      m=scalarRE.search(line)
      value=float(m.group(2))
    # handles vector -> return as numpy.array 1D
    elif line.startswith("vector:"):
      m=vectorRE.search(line)
      value=numpy.array(m.group(2).split(";")).astype(numpy.double)
    # handles matrix -> return as numpy.array 2D
    elif line.startswith("matrix:"):
      m=matrixRE.search(line)
      first=True
      for r in m.group(2).split(";"):
        rn=numpy.array([r.split(",")]).astype(numpy.double)
        if first:
          first=False
          value=rn
        else:
          value=numpy.concatenate((value, rn))
    # handles sting -> return as str
    elif line.startswith("string:"):
      m=stringRE.search(line)
      value=m.group(2)
    # add parameter to dict globas
    globals[m.group(1)]=value
  # execute the python script using all variables of the preprocessor
  globals[pys.attrib['objName']]=self # add self to list of variables (under the name ob the objName attribute)
  exec(code, globals)

# XML namespace of this module (prefixed with { and postfixed with })
NS="{http://www.mbsim-env.de/MBSim}"

XS="{http://www.w3.org/2001/XMLSchema}"
PV="{http://www.mbsim-env.de/MBXMLUtils}"

%}
