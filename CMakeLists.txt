################################################################################
# CMAKE PROPERTIES
################################################################################

# REQUIRE CMAKE VERSION
CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR) 

################################################################################
# PROJECT PROPERTIES 
################################################################################

# PROJECT NAME
PROJECT(Falcon)

# SET DEFAULT BUILD TYPE
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
ENDIF(NOT CMAKE_BUILD_TYPE)

# SYSTEM MESSAGES
MESSAGE("System: ${CMAKE_SYSTEM_NAME}")
MESSAGE("CMake Version: ${CMAKE_VERSION}")
MESSAGE("Build Type: ${CMAKE_BUILD_TYPE}")

# ADD LOCAL CMAKE FILES
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

# FIND LIBRARIES
MESSAGE("Libnifalcon Library")
FIND_PACKAGE(libnifalcon QUIET REQUIRED)
IF(NOT LIBNIFALCON_FOUND)
  MESSAGE(FATAL_ERROR "  Libnifalcon library was not found")
ELSE(NOT LIBNIFALCON_FOUND)
  MESSAGE("  Libraries: ${LIBNIFALCON_LIBRARIES}")
  MESSAGE("  Include Directory: ${LIBNIFALCON_INCLUDE_DIRS}")
ENDIF(NOT LIBNIFALCON_FOUND)

MESSAGE("Boost C++ Library")

SET(Boost_USE_MULTITHREADED ON)
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
FIND_PACKAGE(Boost 1.54.0 COMPONENTS log thread system QUIET REQUIRED)
IF(NOT Boost_FOUND)
  MESSAGE(FATAL_ERROR "  Boost library was not found")
ELSE(NOT Boost_FOUND)
  MESSAGE("  Libraries: ${Boost_LIBRARIES}")
  MESSAGE("  Include Directory: ${Boost_INCLUDE_DIRS}")
  MESSAGE("  Library Directory: ${Boost_LIBRARY_DIRS}")
ENDIF(NOT Boost_FOUND)

# COMPILER OPTIONS 
ADD_DEFINITIONS(-Wall -std=c++11)

# EXECUTABLES NAMING CONVENTION
IF(CMAKE_BUILD_TYPE STREQUAL Debug)
  SET(BUILD_POSTFIX "_d")
ELSE(CMAKE_BUILD_TYPE STREQUAL Debug)
  SET(BUILD_POSTFIX "")
ENDIF(CMAKE_BUILD_TYPE STREQUAL Debug)

# EXECUTABLE BUILD DIRECTORY
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

################################################################################
# DIVE INTO THE CODE 
################################################################################

# DEFINE INCLUDE DIRECTORY
INCLUDE_DIRECTORIES(include)

# OBTAIN ALL SOURCE FILES
FILE(GLOB_RECURSE SOURCES src/*.cpp)
FILE(GLOB_RECURSE HEADERS include/*.hpp)

# ENTER SOURCE DIRECTORY
ADD_EXECUTABLE(falcon ${SOURCES} ${HEADERS})
SET_TARGET_PROPERTIES(falcon PROPERTIES OUTPUT_NAME falcon${BUILD_POSTFIX})
TARGET_LINK_LIBRARIES(falcon ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(falcon ${LIBNIFALCON_LIBRARIES})
