# ----------------------------------------------------------------------------
#   Basic Configuration
# ----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

PROJECT(slamuco)
set(PROJECT_VERSION "1.0.1")
string(REGEX MATCHALL "[0-9]" PROJECT_VERSION_PARTS "${PROJECT_VERSION}")
list(GET PROJECT_VERSION_PARTS 0 PROJECT_VERSION_MAJOR)
list(GET PROJECT_VERSION_PARTS 1 PROJECT_VERSION_MINOR)
list(GET PROJECT_VERSION_PARTS 2 PROJECT_VERSION_PATCH)
set(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")


IF(NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE "Release" )
ENDIF()
#------------------------------------------------------
# Lib Names and Dirs
#------------------------------------------------------

if(WIN32)
    # Postfix of DLLs:
    SET(PROJECT_DLLVERSION "${PROJECT_VERSION_MAJOR}${PROJECT_VERSION_MINOR}${PROJECT_VERSION_PATCH}")
    SET(RUNTIME_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls and binaries")
    SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for binaries")
    SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls")
else()
    # Postfix of so's:
    set(PROJECT_DLLVERSION)
    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/lib/cmake/ /usr/lib/cmake  )

endif()



#

OPTION(USE_OWN_EIGEN3	"Set to OFF to use a standard eigen3 version" ON)
OPTION(BUILD_UTILS	"Set to OFF to not build utils" OFF)
OPTION(BUILD_UTILS_DEVELOPER	"Set to OFF to not build utils" OFF)
OPTION(BUILD_TESTS	"Set to OFF to not build tests" OFF)


OPTION(BUILD_SHARED_LIBS 	"Set to OFF to build static libraries" ON)
OPTION(USE_TIMERS "SET ON/OFF for see time of execution" OFF)
OPTION(USE_AVX "SET ON/OFF " ON)
OPTION(WITHCUDA "SET ON/OFF " ON)
OPTION(WITH_CVBA "SET ON/OFF" OFF)
OPTION(SEARCHQT  "SET ON/OFF" OFF)
OPTION(XFEATURES2D "SET ON/OFF" OFF)

# ----------------------------------------------------------------------------
#   Find Dependencies
# ----------------------------------------------------------------------------
find_package(OpenCV 3 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})


#Find OpenNI2
#OPENNI2
### OPENNI 2
set(OPENNI2LIB_FOUND "NO" )

if(NOT WIN32)
  find_path(OpenNI2_INCLUDE  NAMES OpenNI2/OpenNI.h openni2/OpenNI.h)
  find_library(LIBOPENNI2_LIBRARY NAMES OpenNI2  )
else()
  find_path(OpenNI2_INCLUDE  NAMES OpenNI.h PATHS  ${OPENNI2_DIR}/Include/ )
  find_library(LIBOPENNI2_LIBRARY NAMES OpenNI2 PATHS ${OPENNI2_DIR}/Lib )
endif()

#message(FATAL_ERROR "NI=${OpenNI2_INCLUDE}")
if ( (OpenNI2_INCLUDE STREQUAL "OpenNI2_INCLUDE-NOTFOUND") OR (LIBOPENNI2_LIBRARY STREQUAL "LIBOPENNI2_LIBRARY-NOTFOUND"))
       message(STATUS_MESSAGE "OpenNi  not found inc=${OpenNI2_INCLUDE}")
  else()
    if (WIN32)
        include_directories(${OpenNI2_INCLUDE})
    else()
      include_directories(${OpenNI2_INCLUDE}/openni2)
    endif()
    message(STATUS  "OpenNI.h=${OpenNI2_INCLUDE} LIBOPENNI2_LIBRARY=${LIBOPENNI2_LIBRARY}")
    set(OPENNI2LIB_FOUND "YES" )
    add_definitions(-DUSE_OPENNI2)
endif()


OPTION(BUILD_OWN_ARUCO	"Set to OFF to force find aruco in the system" ON)
 OPTION(BUILD_OWN_CVBA 	"Set to OFF to force find cvba in the system" ON)
OPTION(BUILD_OWN_XFLANN "Set to OFF to force find xflann in the system" ON)
OPTION(BUILD_OWN_G2O "Set to OFF to force find g2o in the system" ON)


SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${OpenCV_LIBS} ${aruco_LIBS}  ${cvba_LIBS}    ${cvba_LIBRARIES} ${fbow_LIBS} )





 
# ----------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
# ----------------------------------------------------------------------------
CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

# ----------------------------------------------------------------------------
# create configuration file from .in file (If you use windows take care with paths)
# ----------------------------------------------------------------------------

CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/config.cmake.in" "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
INSTALL(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" DESTINATION share/${PROJECT_NAME} )




# ----------------------------------------------------------------------------
#   Program Optimization and debug (Extracted from OpenCV)
# ----------------------------------------------------------------------------
set(WARNINGS_ARE_ERRORS 		OFF CACHE BOOL "Treat warnings as errors")
set(WHOLE_PROGRAM_OPTIMIZATION 	OFF CACHE BOOL "Flags for whole program optimization.")

set(EXTRA_CXX_FLAGS "-march=native -Wall")
find_package(OpenMP  )
if (OPENMP_FOUND)
    add_definitions( -DUSE_OMP)
ENDIF()
 IF(USE_TIMERS)
 add_definitions(-DUSE_TIMERS)
 ENDIF()

 IF(CMAKE_COMPILER_IS_GNUCXX OR MINGW)
 SET(GENERAL_FLAGS " -Wall -std=c++11 -mmmx -msse -msse2 -msse3 ${OpenMP_CXX_FLAGS}")
 add_definitions( -DUSE_SSE)
 IF(WARNINGS_ARE_ERRORS)
     SET(GENERAL_FLAGS   "${GENERAL_FLAGS}  -Werror ")
 ENDIF()
 IF(USE_AVX)
     SET(GENERAL_FLAGS "${GENERAL_FLAGS}  -mavx ")
 ENDIF()
 add_definitions(-DUSE_AVX)
 SET(CMAKE_CXX_FLAGS_RELEASE         "${GENERAL_FLAGS}  -O3 -g0  -DNDEBUG")
 SET(CMAKE_CXX_FLAGS_DEBUG           "${GENERAL_FLAGS}  -O0 -g3  -DDEBUG -D_DEBUG -DPRINT_DEBUG_MESSAGES")
 SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO  "${GENERAL_FLAGS}  -O1 -g3  -DNDEBUG -DPRINT_DEBUG_MESSAGES")

 ELSE()  # MSVC

ADD_DEFINITIONS(-DNOMINMAX)


ENDIF()#END OF COMPILER SPECIFIC OPTIONS

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_EXE_LINKER_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${EXTRA_EXE_LINKER_FLAGS_RELEASE}")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${EXTRA_EXE_LINKER_FLAGS_DEBUG}")

#------------------------------------------------
# DIRS
#------------------------------------------------
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/libs)
link_directories(${CMAKE_BINARY_DIR}/libs)
 #MESSAGE(FATAL_ERROR "KKJ=${CMAKE_BINARY_DIR}/3rdparty/libs")
IF(NOT USE_OWN_EIGEN3)
    find_package( Eigen3 REQUIRED )
ELSE()
    SET(EIGEN3_INCLUDE_DIR "3rdparty/eigen3/eigen3")
    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "3rdparty/eigen3/eigen3")
ENDIF()
include_directories( ${EIGEN3_INCLUDE_DIR} )



IF(NOT BUILD_OWN_G2O)
    MESSAGE(FATAL_ERROR "NEED TO USE THE G2O LIBRARY IN 3RTHPARTY")
ELSE ()
    include_directories(3rdparty/g2o/)
    ADD_SUBDIRECTORY(3rdparty/g2o/g2o)
ENDIF()

SET(G2O_LIBS    g2o_stuff  g2o_core    g2o_types_sba)



add_definitions(-DXFLANN_OPENCV)
IF(NOT BUILD_OWN_XFLANN)
    find_package(xflann   REQUIRED)
ELSE ()
    ADD_SUBDIRECTORY(3rdparty/xflann/xflann)
    include_directories(3rdparty/xflann)
ENDIF()
IF(NOT BUILD_OWN_ARUCO)
    find_package(aruco   REQUIRED)
ELSE ()
    ADD_SUBDIRECTORY(3rdparty/aruco/aruco)
    include_directories(3rdparty/aruco)

ENDIF()

IF(WITH_CVBA)
    IF(NOT BUILD_OWN_CVBA)
        find_package(cvba   REQUIRED)
    ELSE ()
        ADD_SUBDIRECTORY(3rdparty/cvba/cvba)
        include_directories(3rdparty/cvba)
    ENDIF()
    add_definitions(-DUSE_CVBA)
    SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} cvba)
ENDIF()



IF(XFEATURES2D)
add_definitions(-DXFEATURES2D)
ENDIF()

find_package(Pangolin REQUIRED)
INCLUDE_DIRECTORIES(${Pangolin_INCLUDE_DIRS})

find_package(apriltag REQUIRED)
if(${apriltag_FOUND} AND TARGET apriltag::apriltag)
    message("apriltag library and target found")
endif()
INCLUDE_DIRECTORIES(${apriltag_INCLUDE_DIRS})

SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES}    aruco xflann  ${G2O_LIBS})

ADD_SUBDIRECTORY(src)
IF (BUILD_UTILS)
  ADD_SUBDIRECTORY(utils)
ENDIF()
IF (BUILD_UTILS_DEVELOPER)
  ADD_SUBDIRECTORY(utils_developer)
ENDIF()
IF (BUILD_TESTS)
ADD_SUBDIRECTORY(tests)
ENDIF()
#ADD_SUBDIRECTORY(spm-utils)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src )
LINK_LIBRARIES(${PROJECT_NAME})
ADD_EXECUTABLE(spm-slam spm-slam.cpp  )
ADD_EXECUTABLE(apriltag-slam apriltag-slam.cpp)
ADD_EXECUTABLE(testPose testPose.cpp)
target_link_libraries(testPose
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES})


# ----------------------------------------------------------------------------
# display status message for important variables
# ----------------------------------------------------------------------------
message( STATUS )
MESSAGE( STATUS "-------------------------------------------------------------------------------" )
message( STATUS "General configuration for ${PROJECT_NAME} ${PROJECT_VERSION}")
MESSAGE( STATUS "-------------------------------------------------------------------------------" )
message( STATUS )
message("    Built as dynamic libs?:" ${BUILD_SHARED_LIBS})
message("    Compiler:"                   "${CMAKE_COMPILER}"   "${CMAKE_CXX_COMPILER}")

message( STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message( STATUS "C++ flags (Release):      ${CMAKE_CXX_FLAGS_RELEASE}")
message( STATUS "C++ flags (Debug):        ${CMAKE_CXX_FLAGS_DEBUG}")
message( STATUS "C++ flags (Relase+Debug): ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
message( STATUS "CMAKE_BINARY_DIR:         ${CMAKE_BINARY_DIR}")

MESSAGE( STATUS )
MESSAGE( STATUS "CMAKE_SYSTEM_PROCESSOR = ${CMAKE_SYSTEM_PROCESSOR}" )
MESSAGE( STATUS "BUILD_SHARED_LIBS = ${BUILD_SHARED_LIBS}" )
MESSAGE( STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}" )
MESSAGE( STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}" )
MESSAGE( STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}" )
MESSAGE( STATUS "BUILD_UTILS= ${BUILD_UTILS}" )
MESSAGE( STATUS "BUILD_TESTS= ${BUILD_TESTS}" )
MESSAGE( STATUS "OPENCV_DIR= ${OpenCV_DIR}" )
MESSAGE( STATUS "USE_OWN_EIGEN3= ${USE_OWN_EIGEN3}" )
IF(NOT USE_OWN_EIGEN3)
MESSAGE( STATUS "    EIGEN= ${EIGEN3_INCLUDE_DIR}" )
ENDIF()
MESSAGE( STATUS "BUILD_OWN_ARUCO= ${BUILD_OWN_ARUCO}" )
MESSAGE( STATUS "BUILD_OWN_FBOW= ${BUILD_OWN_FBOW}" )
MESSAGE( STATUS "BUILD_OWN_CVBA= ${BUILD_OWN_CVBA}" )
MESSAGE( STATUS "BUILD_OWN_XFLANN= ${BUILD_OWN_XFLANN}" )
MESSAGE( STATUS "BUILD_OWN_G2O= ${BUILD_OWN_G2O}" )

MESSAGE(STATUS "REQUIRED_LIBRARIES=${REQUIRED_LIBRARIES}")

MESSAGE( STATUS )
MESSAGE( STATUS "OpenCV_LIB_DIR=${OpenCV_LIB_DIR}")
MESSAGE( STATUS "CMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}")

MESSAGE( STATUS )
MESSAGE( STATUS )
MESSAGE( STATUS "Change a value with: cmake -D<Variable>=<Value>" )
MESSAGE( STATUS )
