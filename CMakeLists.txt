cmake_minimum_required(VERSION 3.0.2)
project(projeto_agrobot_uwb)

find_package(catkin REQUIRED COMPONENTS
  rospy
  urdf
  xacro
  gazebo_ros
  robot_localization
  tf2_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  robot_state_publisher
  rviz
  turtlebot3_description
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/teleop_teclado.py
  scripts/ekf_localizacao_uwb.py
  scripts/bag_para_csv.py
  scripts/identity.py
  scripts/localizacao_trilateracao.py
  scripts/exemplo_trilateracao.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds urdf config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
