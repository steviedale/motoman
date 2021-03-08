search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=motoman_ar1440e.srdf
robot_name_in_srdf=motoman_ar1440e
moveit_config_pkg=motoman_ar1440e_moveit_config
robot_name=motoman_ar1440e
planning_group_name=manipulator
ikfast_plugin_pkg=motoman_ar1440e_manipulator_ikfast_plugin
base_link_name=base_link
eef_link_name=tool0
ikfast_output_path=/home/stevie/workspaces/arccs/arccs_ext_ws/src/motoman/motoman_ar1440e_manipulator_ikfast_plugin/src/motoman_ar1440e_manipulator_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
