#############################################
### Include the subdirectories
#############################################



#############################################
### Add sources
#############################################

SET(mbsimEHD_SRCS
    cylindersolid_cylinderhollow_ehd.h  cylindersolid_cylinderhollow_ehd.cc
    ehd_contact.h                       ehd_contact.cc
    ehd_mesh.h                          ehd_mesh.cc
    ehd_pressure_element.h              ehd_pressure_element.cc
    lagrange_shape_functions.h          lagrange_shape_functions.cc
    lubricant.h                         lubricant.cc
    )


### Set sublibrary
add_library(mbsimEHD SHARED ${mbsimEHD_SRCS})

#############################################
### Install the library
#############################################

install(TARGETS mbsimEHD
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib)

install(DIRECTORY ./
  DESTINATION include/mbsimEHD/
  FILES_MATCHING PATTERN "*.h")
