#############################################
### Include the subdirectories
#############################################

# add the project library with the sources
add_library(mbsim SHARED
            graph.cc
            single_contact.cc
            element.cc
            numerics/gaussian_quadratur.cc
            numerics/nonsmooth_algebra.cc
            numerics/functions/damping_functions.cc
            numerics/functions/newton_method_jacobian_functions.cc
            numerics/functions/lcp_reformulation_functions.cc
            numerics/functions/criteria_functions.cc
            numerics/linear_complementarity_problem/lemke_algorithm.cc
            numerics/linear_complementarity_problem/linear_complementarity_problem.cc
            numerics/nonlinear_algebra/multi_dimensional_fixpoint_solver.cc
            numerics/nonlinear_algebra/multi_dimensional_newton_method.cc
            numerics/nurbs/nurbs_defs.cc
            numerics/nurbs/nurbs_surface.cc
            numerics/nurbs/nurbs_curve.cc
            mbsim_event.cc
            contour.cc
            integrators/dopri5_integrator.cc
            integrators/euler_explicit_integrator.cc
            integrators/theta_time_stepping_ssc_integrator.cc
            integrators/lsoder_integrator.cc
            integrators/rksuite_integrator.cc
            integrators/dop853_integrator.cc
            integrators/lsodar_integrator.cc
            integrators/radau5_integrator.cc
            integrators/theta_time_stepping_integrator.cc
            integrators/daspk_integrator.cc
            integrators/lsode_integrator.cc
            integrators/odex_integrator.cc
            integrators/integrator.cc
            integrators/auto_time_stepping_ssc_integrator.cc
            integrators/quasi_static_integrator.cc
            integrators/time_stepping_integrator.cc
            integrators/time_stepping_ssc_integrator.cc
            integrators/hets2_integrator.cc
            integrators/fortran/opkda1.f
			integrators/fortran/opkda2.f
			integrators/fortran/opkdmain.f
			integrators/fortran/dopri5.f
			integrators/fortran/dop853.f
			integrators/fortran/odex.f
			integrators/fortran/radau5.f
			integrators/fortran/dc_lapack.f
			integrators/fortran/rksuite.f
            spring_damper.cc
            observers/frame_observer.cc
            observers/coordinates_observer.cc
            observers/rigid_body_group_observer.cc
            observers/kinematics_observer.cc
            observers/rigid_body_observer.cc
            contours/contour1s_analytical.cc
            contours/planewithfrustum.cc
            contours/sphere.cc
            contours/rectangle.cc
            contours/contour1s.cc
            contours/edge.cc
            contours/circle.cc
            contours/line.cc
            contours/contour_interpolation.cc
            contours/frustum2d.cc
            contours/polynomial_frustum.cc
            contours/frustum.cc
            contours/point.cc
            contours/line_segment.cc
            contours/contour_quad.cc
            contours/plane.cc
            contours/room.cc
            contours/compound_contour.cc
            contours/cuboid.cc
            contact.cc
            extern_generalized_io.cc
            group.cc
            functions/basic_functions.cc
            functions/piecewise_polynom_function.cc
            functions/symbolic_functions.cc
            functions/kinetic_functions.cc
            functions/fourier_function.cc
            functions/tabular_functions.cc
            functions/kinematic_functions.cc
            object.cc
            objectfactory.cc
            environment.cc
            functions_contact.cc
            utils/rotarymatrices.cc
            utils/contact_utils.cc
            utils/colors.cc
            utils/eps.cc
            utils/octave_utils.cc
            utils/cppCasADiToXML.cc
            utils/utils.cc
            utils/nonlinear_algebra.cc
            utils/ansatz_functions.cc
            utils/openmbv_utils.cc
            utils/stopwatch.cc
            kinetic_excitation.cc
            link_mechanics.cc
            observer.cc
            kinematic_excitation.cc
            friction.cc
            frame.cc
            link.cc
            dynamic_system.cc
            solver.cc
            constraint.cc
            isotropic_rotational_spring_damper.cc
            gear.cc
            dynamic_system_solver.cc
            contact_kinematics/circlesolid_circlesolid.cc
            contact_kinematics/point_sphere.cc
            contact_kinematics/circlesolid_line.cc
            contact_kinematics/point_plane.cc
            contact_kinematics/sphere_plane.cc
            contact_kinematics/point_contourinterpolation.cc
            contact_kinematics/circlesolid_circlehollow.cc
            contact_kinematics/sphere_polynomialfrustum.cc
            contact_kinematics/circlesolid_contour1s.cc
            contact_kinematics/sphere_sphere.cc
            contact_kinematics/rectangle_polynomialfrustum.cc
            contact_kinematics/circlesolid_linesegment.cc
            contact_kinematics/line_contour1s.cc
            contact_kinematics/compoundcontour_compoundcontour.cc
            contact_kinematics/point_contour1s.cc
            contact_kinematics/sphere_frustum.cc
            contact_kinematics/compoundcontour_contour.cc
            contact_kinematics/sphere_rectangle.cc
            contact_kinematics/point_line.cc
            contact_kinematics/point_polynomialfrustum.cc
            contact_kinematics/point_rectangle.cc
            contact_kinematics/circle_frustum.cc
            contact_kinematics/circlesolid_plane.cc
            contact_kinematics/point_circlesolid.cc
            contact_kinematics/point_planewithfrustum.cc
            contact_kinematics/point_frustum.cc
            contact_kinematics/contact_kinematics.cc
            contact_kinematics/circlesolid_frustum2d.cc
            contact_kinematics/edge_edge.cc
            contact_kinematics/point_line_segment.cc
            gearing.cc
            constitutive_laws.cc
            joint.cc
            rotational_spring_damper.cc
            analysers/eigenanalyser.cc
            body.cc
            rigid_body.cc
            )
    
SET_TARGET_PROPERTIES(mbsim PROPERTIES COMPILE_FLAGS "${OPENMBVCPPINTERFACE_CFLAGS_STR}")
target_link_libraries(mbsim "-L${OPENMBVCPPINTERFACE_LIBRARY_DIRS}" ${OPENMBVCPPINTERFACE_LIBRARIES})

# Create a config.h due to the checks made before
configure_file ("${PROJECT_SOURCE_DIR}/mbsim/config.h.in"
                "${PROJECT_BINARY_DIR}/config.h" )
# Include the path when compiling
include_directories(${PROJECT_BINARY_DIR})
include_directories("${CMAKE_SOURCE_DIR}")

#############################################
### Install the library
#############################################

install(TARGETS mbsim
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib)

install(DIRECTORY ./
  DESTINATION include/mbsim
  FILES_MATCHING PATTERN "*.h")
