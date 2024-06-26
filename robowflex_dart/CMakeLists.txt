cmake_minimum_required(VERSION 2.8.3)
project(robowflex_dart)

set(LIBRARY_NAME ${PROJECT_NAME})

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_ROOT_DIR}/cmake/Modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")


include(CompileOptions)
include(HelperFunctions)

find_package(DART 6 QUIET
  COMPONENTS utils utils-urdf gui-osg
  OPTIONAL_COMPONENTS
  CONFIG)


if (DART_LIBRARIES)

  find_package(Boost REQUIRED COMPONENTS filesystem system program_options)
  include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
  list(APPEND LIBRARIES ${Boost_LIBRARIES})

  find_package(Eigen3 REQUIRED)
  include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
  list(APPEND LIBRARIES ${EIGEN3_LIBRARIES})

  find_package(ompl REQUIRED PATHS /usr/local/share/ompl/cmake)
  if (ompl_LIBRARIES)
    include_directories(SYSTEM ${ompl_INCLUDE_DIRS})
    list(APPEND LIBRARIES ${ompl_LIBRARIES})
  else()
    include_directories(SYSTEM ${OMPL_INCLUDE_DIRS})
    list(APPEND LIBRARIES ${OMPL_LIBRARIES})
  endif (ompl_LIBRARIES)

  find_package(TinyXML2 REQUIRED)
  include_directories(SYSTEM ${TINYXML2_INCLUDE_DIRS})
  list(APPEND LIBRARIES ${TINYXML2_LIBRARIES})

  find_package(Threads REQUIRED)
  list(APPEND LIBRARIES ${CMAKE_THREAD_LIBS_INIT})

  include_directories(SYSTEM ${DART_INCLUDE_DIRS})
  list(APPEND LIBRARIES ${DART_LIBRARIES})

  find_package(catkin REQUIRED COMPONENTS
    robowflex_library
    )



  catkin_package(
    LIBRARIES ${LIBRARY_NAME}
    CATKIN_DEPENDS
    robowflex_library
    DEPENDS
    INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
    )

  include_directories(SYSTEM ${CMAKE_CURRENT_LIST_DIR}/include ${catkin_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS})

  add_library(${LIBRARY_NAME}
    src/robot.cpp
    src/io.cpp
    src/space.cpp
    src/world.cpp
    src/acm.cpp
    src/structure.cpp
    src/tsr.cpp
    src/planning.cpp
    src/gui.cpp
    src/planner.cpp
    src/joints/joint.cpp
    src/joints/rnjoint.cpp
    src/joints/so2joint.cpp
    src/joints/so3joint.cpp
          src/conversion_functions.cpp
          src/Object.cpp
          src/planningFunctions.cpp
          src/point_collector.cpp
          src/quaternion_factory.cpp
          src/urdf_read.cpp
          src/ActionRobot.cpp

    )
  set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
  target_link_libraries(${LIBRARY_NAME} PUBLIC ${LIBRARIES})


  add_script(solve_puzzle)
  add_script(solve_puzzle_animation)
  add_script(maze_with_fetch)
  add_script(benchmark_main)
  add_script(benchmark_room)
  add_script(solve_escape_room)
  add_script(solve_room_debug)
  add_script(validate_pose)
  add_script(escape_room)


  install_scripts()
  install_library()
else()
  message("`DART` not found, not compiling robowflex_dart library or executables.")
endif()
