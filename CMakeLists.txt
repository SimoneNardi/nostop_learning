cmake_minimum_required(VERSION 2.8.3)
project(nostop_learning)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED roscpp base_math)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES ${PROJECT_NAME}
  DEPENDS base_math
)

###########
## Build ##
###########

include_directories(include/${PROJECT_NAME}/ ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
include_directories(src/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)
include_directories(${CMAKE_INSTALL_PREFIX}/include) # this line is to find lemon include files.

set(Learning_SOURCES
	src/agent.cpp
	src/agentTrajectory.cpp
	src/probability.cpp
	src/area.cpp
	src/discretizedArea.cpp
	src/guard.cpp
	src/learningAlgorithm.cpp
	src/shape2D.cpp
	src/structuredArea.cpp
	src/unStructuredArea.cpp
	src/world.cpp
	src/line2D.cpp
	src/coverageAlgorithm.cpp
	src/DISLAlgorithm.cpp
	src/PIPIPAlgorithm.cpp
	src/ParetoEfficientAlgorithm.cpp
	src/CoarseCorrelatedAlgorithm.cpp
	src/learningWorld.cpp
	src/polygon.cpp
	src/circle.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED ${Learning_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} lemon)
add_dependencies(${PROJECT_NAME} nostop_learning_gencpp base_math lemon)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
         EXPORT ${PROJECT_NAME}
#         ARCHIVE DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}" COMPONENT lib
#         RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}" COMPONENT bin
         LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}" COMPONENT shlib
)