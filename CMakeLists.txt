cmake_minimum_required(VERSION 2.8.3)
project(srrg_core)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#ds default flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fPIC -std=c++17")

find_package(srrg_cmake_modules REQUIRED)
set(CMAKE_MODULE_PATH ${srrg_cmake_modules_INCLUDE_DIRS})
find_package(catkin REQUIRED)

set(CMAKE_BUILD_TYPE Release)

message("${PROJECT_NAME}|PROCESSOR_TYPE: [${CMAKE_HOST_SYSTEM_PROCESSOR}]")
if (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES armv7l)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  message("${PROJECT_NAME}|ENABLING ARM NEON OPTIMIZATIONS")
endif ()

#ds before looking for OpenCV - check if a supported ros version is installed - in that case we will use that OpenCV version
if("$ENV{ROS_DISTRO}" STREQUAL "kinetic" OR "$ENV{ROS_DISTRO}" STREQUAL "indigo" OR "$ENV{ROS_DISTRO}" STREQUAL "melodic")
  message("${PROJECT_NAME}|found ROS version: '$ENV{ROS_DISTRO}' linking against ROS OpenCV")

  #ds load opencv through ROS
  find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_geometry
  )
  set(OpenCV_LIBS ${catkin_LIBRARIES})
else()

  #ds look for a separately installed opencv library
  find_package(OpenCV REQUIRED)
endif()
include_directories(${OpenCV_INCLUDE_DIRS})
message("${PROJECT_NAME}|OpenCV version ${OpenCV_VERSION}")

#ds check if we have contributed features in opencv (optional)
string(FIND "${OpenCV_LIBS}" "xfeatures2d" FOUND_OPENCV_CONTRIB)
if(NOT ${FOUND_OPENCV_CONTRIB} EQUAL -1)
  message("${PROJECT_NAME}|found xfeatures2d library, building contributed OpenCV components")
  add_definitions(-DSRRG_CORE_HAS_OPENCV_CONTRIB)
endif()

#disable eigen alignment
add_definitions(-DEIGEN_DONT_ALIGN)

# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES
  srrg_boss_library
  srrg_core_types_library
  srrg_system_utils_library
  srrg_image_utils_library
  srrg_messages_library
  srrg_path_map_library
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

#ds help the catkin tool on 16.04 (cmake seems unable to find single libraries, although catkin claims the link_directories call is not required)
#ds in order to avoid linking against the catkin_LIBRARIES bulk everytime enable this so one can select single libraries
link_directories(${catkin_LIBRARY_DIRS})

#ds add source
add_subdirectory(${PROJECT_SOURCE_DIR}/src)

#ia print all the flags
message("${PROJECT_NAME}|compiling with these flags [ ${CMAKE_CXX_FLAGS} ]")
