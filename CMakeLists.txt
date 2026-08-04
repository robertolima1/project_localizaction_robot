cmake_minimum_required(VERSION 3.0.2)
project(projeto_agrobot_uwb)

find_package(catkin REQUIRED COMPONENTS
  rospy
  urdf
  xacro
  gazebo_ros
  gazebo_msgs
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
  scripts/ruido_uwb.py
  scripts/spawn_campo.py
  scripts/perfil_recursos.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# layout_ancoras.py não é um node (nunca roda via rosrun/roslaunch) — é um
# módulo importado pelos scripts acima. install(FILES ...), não PROGRAMS,
# pra cair no mesmo diretório (necessário em install space; em devel space
# os scripts resolvem o import via os.path.realpath(__file__), que segue o
# symlink até scripts/ na árvore fonte).
install(FILES scripts/layout_ancoras.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds urdf config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
