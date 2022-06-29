CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(rs4se)

add_compile_options(-std=c++11)

# ADD_COMPILE_OPTIONS(
#   -Wall
#   -Wextra
#   -fsanitize=address
#   -fno-omit-frame-pointer
# )
#
# LINK_LIBRARIES(
#   -fsanitize=address
#   -static-libasan # Use -static-libsan for Clang
# )

# PACKAGES
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
FIND_PACKAGE(
  catkin
  REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

INCLUDE_DIRECTORIES(
  include
  ${catkin_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR})


add_library(auto_exp
  src/auto_exp.cpp 
  include/auto_exp/auto_exp.h 
  src/ 
)


target_link_libraries(auto_exp
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS} 
)

# EXECUTABLES
SET(DEPS
  ${realsense2_LIBRARY}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  boost_system
  auto_exp
)


ADD_EXECUTABLE(intel_d435i src/intel_d435i.cpp)
TARGET_LINK_LIBRARIES(intel_d435i ${DEPS})

ADD_EXECUTABLE(test_rs4se src/test_rs4se.cpp)
TARGET_LINK_LIBRARIES(test_rs4se ${DEPS})
