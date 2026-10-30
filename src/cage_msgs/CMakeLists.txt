cmake_minimum_required(VERSION 3.0.2)
project(cage_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES
  CageGeometry.msg
  CageGrid.msg
  CellTargetArray.msg
  CellTarget.msg
  FeedingTaskRequest.msg
  FeedingTaskStatus.msg
  MissionRequest.msg
  MissionWaypoint.msg
  MissionWaypointArray.msg
  MissionState.msg
  FeederCommand.msg)
add_service_files(FILES
  RunFeeder.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
