cmake_minimum_required(VERSION 3.0.2)
project(cage_site_db)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/cage_site_db_server scripts/cage-site-db
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config www DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
