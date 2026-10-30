cmake_minimum_required(VERSION 3.0.2)
project(cage_simulation)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/mock_mavros_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch worlds models config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS
  scripts/start_full_simulation.sh
  scripts/generate_circular_cage.py
  scripts/generate_fish_farm.py
  scripts/generate_water_surface.py
  scripts/generate_uav_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
