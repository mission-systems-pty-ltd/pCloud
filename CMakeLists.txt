cmake_minimum_required(VERSION 3.5)

SET(EXEC_NAME pCloud)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $ENV{BIN_HOME})

# This allows us to use the header-only JSON library
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

# Switching on C++11 has unleashed deprecation warnings for MOOS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations" )

#===============================================================================
# FINDING MOOS' HEADERS AND LIBRARIES...
#===============================================================================

# moos - http://www.robots.ox.ac.uk/~mobile/MOOS/wiki/pmwiki.php
find_package(MOOS 10.0)

message("+++++++++++++++++++++++++++++++++++++++++")
message("FIND_PACKAGE CONSIDERED:" ${MOOS_CONSIDERED_CONFIGS})
message("+++++++++++++++++++++++++++++++++++++++++")

if(NOT DEFINED MOOS_LIBRARIES)
  	message("Defining MOOS_LIBRARIES")
	set(MOOS_LIBRARIES MOOS)
endif()

message("+++++++++++++++++++++++++++++++++++++++++")
message("MOOS_INCLUDE_DIRS:" ${MOOS_INCLUDE_DIRS})
message("MOOS_LIBRARIES:   " ${MOOS_LIBRARIES})
message("+++++++++++++++++++++++++++++++++++++++++")

#===============================================================================
# FINDING MOOSGeodesy' HEADERS AND LIBRARIES...
#===============================================================================
# moos - http://www.robots.ox.ac.uk/~mobile/MOOS/wiki/pmwiki.php
find_package(MOOSGeodesy)

#link_directories(${MOOSGeodesy_LIBRARY_PATH})

message("+++++++++++++++++++++++++++++++++++++++++")
message("MOOSGeodesy_INCLUDE_DIRS:" ${MOOSGeodesy_INCLUDE_DIRS})
message("MOOSGeodesy_LIBRARY_PATH:" ${MOOSGeodesy_LIBRARY_PATH})
message("+++++++++++++++++++++++++++++++++++++++++")

# Some more packages
find_package( PCL 1.7.0 REQUIRED )    

set(CMAKE_PREFIX_PATH $ENV{JSON_HOME};$ENV{JSON_HOME}/build)
find_package(nlohmann_json 3.5.0 CONFIG REQUIRED)

add_executable(${EXEC_NAME}
    main.cpp
    Cloud.cpp
    Cloud_Info.cpp)

target_include_directories(${EXEC_NAME} PUBLIC
	./
    #$ENV{JSON_HOME}/src
    $ENV{MOOSIVP_SOURCE_TREE_BASE}/include/ivp
    ${PCL_INCLUDE_DIRS}
    ${MOOS_INCLUDE_DIRS}
    ${MOOSGeodesy_INCLUDE_DIRS})

TARGET_LINK_LIBRARIES(${EXEC_NAME}
    ${PCL_LIBRARIES}
    ${MOOSGeodesy_LIBRARIES}
    ${MOOS_LIBRARIES}
    $ENV{MOOSIVP_SOURCE_TREE_BASE}/lib/libmbutil.a
    nlohmann_json::nlohmann_json
    m)
