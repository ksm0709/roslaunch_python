cmake_minimum_required(VERSION 2.8.3)
project(roslaunch_python)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  message_generation
)

catkin_python_setup()

add_message_files(
   FILES
   LaunchInterface.msg
 )

add_service_files(
   FILES
   LaunchAdd.srv
   LaunchStart.srv
   LaunchStop.srv
   LaunchCheckStatus.srv
 )

generate_messages(DEPENDENCIES std_msgs std_srvs)

#catkin_package(
#  LIBRARIES roslaunch_python
#  CATKIN_DEPENDS 
#    rospy
#    std_msgs
#    std_srvs
#    message_runtime
#)


install(PROGRAMS
   src/roslaunch_python/roslaunch_server.py
   src/roslaunch_python/roslaunch_example.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
