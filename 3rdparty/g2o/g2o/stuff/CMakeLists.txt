ADD_LIBRARY( g2o_stuff
color_macros.h  macros.h  misc.h  os_specific.c  os_specific.h  property.cpp  property.h  string_tools.cpp  string_tools.h  timeutil.cpp  timeutil.h)

#ADD_LIBRARY(g2o_stuff
#  filesys_tools.h    string_tools.h
#  color_macros.h     macros.h         timeutil.cpp
#  command_args.cpp   misc.h           sparse_helper.cpp  timeutil.h
#  command_args.h     os_specific.c    sparse_helper.h
#  filesys_tools.cpp  os_specific.h    string_tools.cpp
#  property.cpp       property.h       
#  sampler.cpp        sampler.h        unscented.h
#  tictoc.cpp tictoc.h
#  g2o_stuff_api.h
#)

#SET_TARGET_PROPERTIES(stuff PROPERTIES OUTPUT_NAME g2o_stuff)

INSTALL(TARGETS g2o_stuff
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)


#IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux" AND NOT ANDROID)
#  TARGET_LINK_LIBRARIES(stuff rt)
#ENDIF()

#FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

#INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/stuff)
