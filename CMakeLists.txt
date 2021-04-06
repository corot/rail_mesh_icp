cmake_minimum_required(VERSION 2.8.3)
project(rail_mesh_icp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED 1.8 REQUIRED COMPONENTS common io)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  actionlib
  pcl_ros
  pcl_conversions
  tf2_ros
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_service_files(
  DIRECTORY srv
  FILES
  ICPMatch.srv
  TemplateMatch.srv
)


add_action_files(
  DIRECTORY action
  FILES
  MatchTemplate.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LINK_LIBS
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    stdc++fs
)

# compiles cpp nodes
#add_executable(icp_matcher_node src/icp_matcher_node.cpp src/ICPMatching.cpp)
#add_dependencies(icp_matcher_node ${catkin_EXPORTED_TARGETS} ${PCL_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(icp_matcher_node ${LINK_LIBS})

add_executable(template_matcher_node src/template_matcher_node.cpp src/TemplateMatching.cpp src/ICPMatching.cpp)
add_dependencies(template_matcher_node ${catkin_EXPORTED_TARGETS} ${PCL_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template_matcher_node ${LINK_LIBS})

add_executable(mesh_sampler_node tools/mesh_sampler_node.cpp)
add_dependencies(mesh_sampler_node ${catkin_EXPORTED_TARGETS} ${PCL_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(mesh_sampler_node ${LINK_LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(TARGETS template_matcher_node mesh_sampler_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
