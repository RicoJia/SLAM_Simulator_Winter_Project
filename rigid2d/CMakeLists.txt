cmake_minimum_required(VERSION 2.8.3)
project(rigid2d)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-Wall -Werror)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        nav_msgs
        geometry_msgs
)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed  <test_depend>rosunit</test_depend>
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp tf nav_msgs geometry_msgs
#        DEPENDS system_lib
)

#if (CATKIN_ENABLE_TESTING)
#    catkin_add_gtest(Transform2D_test test/Transform2D_test.test.cpp)
#endif()
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include    #This is crucial as it tells catkin to search for .h file in your include!!
  ${catkin_INCLUDE_DIRS}     #[QUESTION]: Not sure what this is??
)


## Declare a C++ library
add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}.cpp
        src/diff_drive.cpp
        src/waypoints.cpp
        src/fake_diff_encoders.cpp
 )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_executable(${PROJECT_NAME}_Odometer_node src/odometer.cpp)
add_executable(${PROJECT_NAME}_Fake_Diff_Encoders_node src/fake_diff_encoders.cpp)
#add_executable(${PROJECT_NAME}_Real_World_node src/real_world.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_Odometer_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_Fake_Diff_Encoders_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)
#
#target_link_libraries(${PROJECT_NAME}_Real_World_node
#        ${catkin_LIBRARIES}
#        ${PROJECT_NAME}
#        )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_Odometer_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME}_Fake_Diff_Encoders_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

#install(TARGETS ${PROJECT_NAME}_Real_World_node
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )
## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EX\CLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES include/${PROJECT_NAME}/${PROJECT_NAME}.hpp
        include/${PROJECT_NAME}/fake_diff_encoders.hpp
        include/${PROJECT_NAME}/diff_drive.hpp
        include/${PROJECT_NAME}/waypoints.hpp
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rigid2d.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}_FakeDiffEncoders_test test/FakeDiffEncoders_test.cpp)
    target_link_libraries(${PROJECT_NAME}_FakeDiffEncoders_test ${PROJECT_NAME} ${catkin_Libraries} gtest_main)      #don't forget your own library - ${PROJECT_NAME}

    catkin_add_gtest(${PROJECT_NAME}_Transform2D_test test/Transform2D_test.cpp)
    target_link_libraries(${PROJECT_NAME}_Transform2D_test ${PROJECT_NAME} ${catkin_Libraries} gtest_main)      #don't forget your own library - ${PROJECT_NAME}

    catkin_add_gtest(${PROJECT_NAME}_DiffDrive_test test/DiffDrive_test.cpp)
    target_link_libraries(${PROJECT_NAME}_DiffDrive_test ${PROJECT_NAME} ${catkin_Libraries} gtest_main)      #don't forget your own library - ${PROJECT_NAME}

    catkin_add_gtest(${PROJECT_NAME}_Waypoints_test test/Waypoints_test.cpp)
    target_link_libraries(${PROJECT_NAME}_Waypoints_test ${PROJECT_NAME} ${catkin_Libraries} gtest_main)      #don't forget your own library - ${PROJECT_NAME}
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
