cmake_minimum_required(VERSION 3.0.2)
project(ooxx)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ooxx_node.py
  scripts/target_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/ooxx_cleanup.sh scripts/ooxx_logrun.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/search_fsm.py
  scripts/mission_state.py
  scripts/config_loader.py
  scripts/depth_projection.py
  scripts/frontier_navigator.py
  scripts/boustrophedon_planner.py
  scripts/occupancy_grid.py
  scripts/grid_planner.py
  scripts/local_planner.py
  scripts/global_planner.py
  scripts/perimeter_controller.py
  scripts/nav_feedback.py
  scripts/pose_estimator.py
  scripts/move_controller.py
  scripts/lidar_utils.py
  scripts/perception/__init__.py
  scripts/perception/base.py
  scripts/perception/classical.py
  scripts/perception/topic.py
  scripts/tracking/__init__.py
  scripts/tracking/base.py
  scripts/tracking/stub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
