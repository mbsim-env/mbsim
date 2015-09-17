#############################################
### Include the subdirectories
#############################################

# add the project library with the sources
add_library(mbsimEHD SHARED 
    ehd_pressure_element.cc
    cylindersolid_cylinderhollow_ehd.cc
    ehd_contact.cc
    ehd_mesh.cc
    lagrange_shape_functions.cc
    lubricant.cc)
    
SET_TARGET_PROPERTIES(mbsimEHD PROPERTIES COMPILE_FLAGS "${MBSIM_CFLAGS_STR}")
target_link_libraries(mbsimEHD ${MBSIM_LDFLAGS})

#############################################
### Install the library
#############################################

install(TARGETS mbsimEHD
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib)

install(DIRECTORY ./
  DESTINATION include/mbsimEHD/
  FILES_MATCHING PATTERN "*.h")
