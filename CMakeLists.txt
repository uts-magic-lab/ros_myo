cmake_minimum_required(VERSION 2.8.3)
project(ros_myo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   MyoArm.msg
   EmgArray.msg
   MyoPose.msg
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package()


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/myo-rawNode.py
  scripts/bluetooth.py
  scripts/common.py
  scripts/emg_ascii_graph.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark other files for installation (e.g. launch and bag files, etc.)
foreach (dir launch)
  install(DIRECTORY ${dir}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
