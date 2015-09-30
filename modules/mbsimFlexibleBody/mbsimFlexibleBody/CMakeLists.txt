#############################################
### Include the subdirectories
#############################################

# add the project library with the sources
add_library(mbsimFlexibleBody SHARED 
            contact_kinematics/point_nurbsdisk2s.cc
            contact_kinematics/point_cylinderflexible.cc
            contact_kinematics/circlehollow_cylinderflexible.cc
            contact_kinematics/point_contour2s.cc
            contact_kinematics/circlesolid_flexibleband.cc
            contact_kinematics/circle_nurbsdisk2s.cc
            contact_kinematics/point_flexibleband.cc
            contours/neutral_contour/contour_1s_neutral_cosserat.cc
            contours/neutral_contour/contour_1s_neutral_reference_curve.cc
            contours/neutral_contour/neutral_contour_components/neutral_nurbs_2s.cc
            contours/neutral_contour/neutral_contour_components/neutral_nurbs_position_1s.cc
            contours/neutral_contour/neutral_contour_components/neutral_nurbs_velocity_1s.cc
            contours/neutral_contour/neutral_contour_components/neutral_nurbs_velocity_2s.cc
            contours/neutral_contour/neutral_contour_components/neutral_nurbs_angle_1s.cc
            contours/neutral_contour/neutral_contour_components/neutral_nurbs_local_position_2s.cc
            contours/neutral_contour/neutral_contour_components/neutral_nurbs_position_2s.cc
            contours/neutral_contour/neutral_contour_components/neutral_nurbs_dotangle_1s.cc
            contours/neutral_contour/neutral_contour_components/neutral_nurbs_local_position_1s.cc
            contours/neutral_contour/neutral_contour_components/neutral_nurbs_1s.cc
            contours/neutral_contour/contour_1s_neutral_linear_external_FFR.cc
            contours/neutral_contour/contour_2s_neutral_linear_external_FFR.cc
            contours/flexible_band.cc
            contours/contour_1s_neutral_factory.cc
            contours/contour1s_flexible.cc
            contours/nurbs_curve_1s.cc
            contours/nurbs_disk_2s.cc
            fixed_nodal_frame.cc
            flexible_body/flexible_body_2s_13_disk.cc
            flexible_body/flexible_body_1S_reference_curve.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/N_12.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/N_22.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/NR_13.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/M_RR.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/NR_23.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/K.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/N_33.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/N_21.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/N_compl.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/R_ij.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/N_11.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin/R_compl.cc
            flexible_body/finite_elements/superelement_linear_external.cc
            flexible_body/finite_elements/finite_element_1S_reference_curve.cc
            flexible_body/finite_elements/finite_element_1s_21_cosserat_rotation.cc
            flexible_body/finite_elements/finite_element_2s_13_mfr_mindlin.cc
            flexible_body/finite_elements/finite_element_1s_21_rcm.cc
            flexible_body/finite_elements/finite_element_1s_33_cosserat_translation.cc
            flexible_body/finite_elements/finite_element_1s_21_ancf.cc
            flexible_body/finite_elements/finite_element_1s_23_bta.cc
            flexible_body/finite_elements/finite_element_2s_13_disk.cc
            flexible_body/finite_elements/finite_element_1s_21_cosserat_translation.cc
            flexible_body/finite_elements/finite_element_1s_33_ancf.cc
            flexible_body/finite_elements/finite_element_1s_33_cosserat_rotation.cc
            flexible_body/finite_elements/finite_element_1s_33_rcm.cc
            flexible_body/finite_elements/finite_element_linear_external_lumped_node.cc
            flexible_body/finite_elements/finite_element_1s_33_rcm/trafo33RCM.cc
            flexible_body/finite_elements/finite_element_1s_33_rcm/weight33RCM.cc
            flexible_body/flexible_body_1s_23_bta.cc
#            flexible_body/flexible_body_1s_01_torsion.cc
            flexible_body/flexible_body_1s_33_cosserat.cc
            flexible_body/flexible_body_linear_external.cc
            flexible_body/flexible_body_1s_21_rcm.cc
            flexible_body/flexible_body_1s_21_cosserat.cc
            flexible_body/flexible_body_linear_external_ffr.cc
            flexible_body/flexible_body_1s_21_ancf.cc
            flexible_body/flexible_body_1s_33_ancf.cc
            flexible_body/flexible_body_1s_cosserat.cc
            flexible_body/flexible_body_2s_13_mfr_mindlin.cc
            flexible_body/flexible_body_1s_33_rcm.cc
            flexible_body/flexible_body_ffr.cc
            flexible_body/flexible_body_2s_13.cc
            flexible_body.cc
            node_frame.cc
            utils/contact_utils.cc
            utils/cardan.cc
            utils/revcardan.cc
            utils/angles.cc
            )
    
SET_TARGET_PROPERTIES(mbsimFlexibleBody PROPERTIES COMPILE_FLAGS "${MBSIM_CFLAGS_STR}")
target_link_libraries(mbsimFlexibleBody ${MBSIM_LDFLAGS})


# Create a config.h due to the checks made before
configure_file ("${PROJECT_SOURCE_DIR}/mbsimFlexibleBody/config.h.in"
                "${PROJECT_BINARY_DIR}/config.h" )
# Include the path when compiling
include_directories(${PROJECT_BINARY_DIR})
include_directories("${CMAKE_SOURCE_DIR}")

#############################################
### Install the library
#############################################

install(TARGETS mbsimFlexibleBody
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib)

install(DIRECTORY ./
  DESTINATION include/mbsimFlexibleBody
  FILES_MATCHING PATTERN "*.h")
