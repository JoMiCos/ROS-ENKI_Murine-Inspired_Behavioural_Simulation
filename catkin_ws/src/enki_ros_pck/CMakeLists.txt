cmake_minimum_required(VERSION 3.0)

project(enki_ros_pck)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  message_generation
  
)

add_message_files(
  FILES
  Sight.msg
)

generate_messages(
      DEPENDENCIES
      std_msgs
        roscpp
) 


catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(enki_LIBRARY /usr/local/lib/libenki.a)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt5 COMPONENTS OpenGL REQUIRED)

set(SOURCES run.cpp Racer.cpp bandpass.cpp ReversalSignals.cpp)

include_directories(/usr/local/include/enki)


add_executable(robot ${SOURCES})
target_link_libraries(robot ${catkin_LIBRARIES} ${enki_LIBRARY} Qt5::OpenGL Qt5::Widgets GL)
add_dependencies(robot enki_ros_pck_gencpp)

#catkin_install_python(PROGRAMS py_plot/topic_plot.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

