cmake_minimum_required(VERSION 2.8.3)
project(pepper_nullemo)


find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy 
  sensor_msgs 
  std_msgs 
  geometry_msgs
  visualization_msgs
  message_generation
)


# Directories --------------------------------------------------------------------------------------------------------

include_directories(include ${catkin_INCLUDE_DIRS} filtrjac)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(orocos_kdl REQUIRED)
include_directories(orocos_kdl)


#include_directories(./ReflexxesTypeII/include)
# set ( REFLEXXES_SOURCES		ReflexxesTypeII/src/ReflexxesAPI.cpp
# 				ReflexxesTypeII/src/TypeIIRMLCalculatePositionalExtrems.cpp
# 				ReflexxesTypeII/src/TypeIIRMLDecisions.cpp
# 				ReflexxesTypeII/src/TypeIIRMLDecisionTree1A.cpp
# 				ReflexxesTypeII/src/TypeIIRMLDecisionTree1B.cpp
# 				ReflexxesTypeII/src/TypeIIRMLDecisionTree1C.cpp
# 				ReflexxesTypeII/src/TypeIIRMLDecisionTree2.cpp
# 				ReflexxesTypeII/src/TypeIIRMLFallBackStrategy.cpp
# 				ReflexxesTypeII/src/TypeIIRMLIsPhaseSynchronizationPossible.cpp
# 				ReflexxesTypeII/src/TypeIIRMLPolynomial.cpp
# 				ReflexxesTypeII/src/TypeIIRMLPosition.cpp
# 				ReflexxesTypeII/src/TypeIIRMLQuicksort.cpp
# 				ReflexxesTypeII/src/TypeIIRMLSetupModifiedSelectionVector.cpp
# 				ReflexxesTypeII/src/TypeIIRMLStep1.cpp
# 				ReflexxesTypeII/src/TypeIIRMLStep1IntermediateProfiles.cpp
# 				ReflexxesTypeII/src/TypeIIRMLStep1Profiles.cpp
# 				ReflexxesTypeII/src/TypeIIRMLStep2.cpp
# 				ReflexxesTypeII/src/TypeIIRMLStep2IntermediateProfiles.cpp
# 				ReflexxesTypeII/src/TypeIIRMLStep2PhaseSynchronization.cpp
# 				ReflexxesTypeII/src/TypeIIRMLStep2Profiles.cpp
# 				ReflexxesTypeII/src/TypeIIRMLStep2WithoutSynchronization.cpp
# 				ReflexxesTypeII/src/TypeIIRMLStep3.cpp
# 				ReflexxesTypeII/src/TypeIIRMLVelocityCalculatePositionalExtrems.cpp
# 				ReflexxesTypeII/src/TypeIIRMLVelocity.cpp
# 				ReflexxesTypeII/src/TypeIIRMLVelocityFallBackStrategy.cpp
# 				ReflexxesTypeII/src/TypeIIRMLVelocityIsPhaseSynchronizationPossible.cpp
# 				ReflexxesTypeII/src/TypeIIRMLVelocityMethods.cpp
# 				ReflexxesTypeII/src/TypeIIRMLVelocitySetupPhaseSyncSelectionVector.cpp
# )


# ROS ----------------------------------------------------------------------------------------------------------------

add_message_files(
  FILES
  geometry_msgs_pose_vector.msg
)

add_service_files(
  FILES
  pepper_rnd_cfg_gen.srv
  pepper_rnd_pose_gen.srv
  pepper_trajectory_selection.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package( 
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime sensor_msgs geometry_msgs std_msgs visualization_msgs
  DEPENDS ${orocos_kdl_LIBRARIES}
)


# Libraries -----------------------------------------------------------------------------------------------------------------------------

add_library(pepper_nullemo_aux src/pepper_nullemo_aux.cpp)

add_library(pepper_chain_init src/pepper_chain_init.cpp)

add_library(pepper_ik_algs src/pepper_ik_algs.cpp)
target_link_libraries(pepper_ik_algs pepper_nullemo_aux)

add_library(pepper_ik_algs_full src/pepper_ik_algs_full.cpp)
target_link_libraries(pepper_ik_algs_full pepper_nullemo_aux)

add_library(pepper_core src/pepper_core.cpp)
target_link_libraries(pepper_core pepper_chain_init pepper_nullemo_aux ${orocos_kdl_LIBRARIES})

add_library(pepper_nullemo_tasks src/pepper_nullemo_tasks.cpp)
target_link_libraries(pepper_nullemo_tasks pepper_core pepper_nullemo_aux)

add_library(pepper_emotional src/pepper_emotional.cpp)

add_library(filtrjac filtrjac/filtrjac.cpp)
#add_library( libreflexxes ${REFLEXXES_SOURCES} )

# Executables ---------------------------------------------------------------------------------------------------------------------------

#   pepper_nullemo
add_executable(pepper_nullemo src/pepper_nullemo.cpp)
add_dependencies(pepper_nullemo ${PROJECT_NAME} pepper_nullemo_generate_messages_cpp)
target_link_libraries(pepper_nullemo ${catkin_LIBRARIES} pepper_core pepper_chain_init pepper_ik_algs ${orocos_kdl_LIBRARIES})

#   pepper_random_config_generator
add_executable(pepper_random_config_generator src/pepper_random_config_generator.cpp)
add_dependencies(pepper_random_config_generator ${PROJECT_NAME} pepper_nullemo_generate_messages_cpp)
target_link_libraries(pepper_random_config_generator ${catkin_LIBRARIES} pepper_core)

#   pepper_random_pose_generator
add_executable(pepper_random_pose_generator src/pepper_random_pose_generator.cpp)
add_dependencies(pepper_random_pose_generator ${PROJECT_NAME} pepper_nullemo_generate_messages_cpp)
target_link_libraries(pepper_random_pose_generator ${catkin_LIBRARIES} pepper_core pepper_chain_init ${orocos_kdl_LIBRARIES})

#   pepper_poses_to_rviz
add_executable(pepper_poses_to_rviz src/pepper_poses_to_rviz.cpp)
add_dependencies(pepper_poses_to_rviz ${PROJECT_NAME} pepper_nullemo_generate_messages_cpp)
target_link_libraries(pepper_poses_to_rviz ${catkin_LIBRARIES} pepper_core)

#   pepper_offset_pose_from_config_generator.cpp
add_executable(pepper_offset_pose_from_config_generator src/pepper_offset_pose_from_config_generator.cpp)
add_dependencies(pepper_offset_pose_from_config_generator ${PROJECT_NAME} pepper_nullemo_generate_messages_cpp)
target_link_libraries(pepper_offset_pose_from_config_generator ${catkin_LIBRARIES} pepper_core pepper_chain_init ${orocos_kdl_LIBRARIES})

#   pepper_cartesian_trajectories_generator
add_executable(pepper_cartesian_trajectories_generator src/pepper_cartesian_trajectories_generator.cpp)
add_dependencies(pepper_cartesian_trajectories_generator ${PROJECT_NAME} pepper_nullemo_generate_messages_cpp)
target_link_libraries(pepper_cartesian_trajectories_generator ${catkin_LIBRARIES} pepper_core pepper_chain_init ${orocos_kdl_LIBRARIES})

#   pepper_arm_config_listener
add_executable(pepper_arm_config_listener src/pepper_arm_config_listener.cpp)
add_dependencies(pepper_arm_config_listener ${PROJECT_NAME} pepper_nullemo_generate_messages_cpp)
target_link_libraries(pepper_arm_config_listener ${catkin_LIBRARIES} pepper_core)

#   pepper_config_tester
add_executable(pepper_config_tester src/pepper_config_tester.cpp)
add_dependencies(pepper_config_tester ${PROJECT_NAME} pepper_nullemo_generate_messages_cpp)
target_link_libraries(pepper_config_tester ${catkin_LIBRARIES} pepper_core)

#   pepper_nullemo_full
add_executable(pepper_nullemo_full src/pepper_nullemo_full.cpp)
add_dependencies(pepper_nullemo_full ${PROJECT_NAME} pepper_nullemo_generate_messages_cpp)
target_link_libraries(pepper_nullemo_full ${catkin_LIBRARIES} pepper_core pepper_chain_init pepper_ik_algs_full ${orocos_kdl_LIBRARIES} pepper_nullemo_tasks pepper_emotional filtrjac)# libreflexxes)

#   reflexxes_smooth
# add_executable(reflexxes_pepper_smooth src/pepper_nullemo.cpp)
# add_dependencies(reflexxes_pepper_smooth ${PROJECT_NAME} pepper_nullemo_generate_messages_cpp)
# target_link_libraries(reflexxes_pepper_smooth ${catkin_LIBRARIES} pepper_core)