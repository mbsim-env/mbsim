#############################################
### Include the subdirectories
#############################################



#############################################
### Add sources
#############################################

SET(mbsimEHD_SRCS
    lubricant.h    lubricant.cc
    )


### Set sublibrary
add_library(mbsimEHD ${mbsimEHD_LIBRARY_TYPE} ${mbsimEHD_SRCS})

#############################################
### Install the library
#############################################

install(TARGETS mbsimEHD
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib)

install(DIRECTORY ./
  DESTINATION include/mbsimEHD/
  FILES_MATCHING PATTERN "*.h")
