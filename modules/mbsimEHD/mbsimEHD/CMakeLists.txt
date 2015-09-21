#############################################
### Include the subdirectories
#############################################

# add the project library with the sources
add_library(mbsimEHD SHARED 
    ehd_pressure_element.cc
    ehd_contact.cc
    ehd_mesh.cc
    element_shapes.cc
    lubricant.cc
    contact_kinematics/contact_kinematics_ehd_interface.cc
    contact_kinematics/cylindersolid_cylinderhollow_ehd.cc
    numerics/lagrange_shape_functions.cc)
    
SET_TARGET_PROPERTIES(mbsimEHD PROPERTIES COMPILE_FLAGS "${MBSIM_CFLAGS_STR}")
target_link_libraries(mbsimEHD ${MBSIM_LDFLAGS})

#############################################
### Install the library
#############################################

install(TARGETS mbsimEHD
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib)

install(DIRECTORY ./
  DESTINATION include/mbsimEHD
  FILES_MATCHING PATTERN "*.h")
