#include <mav_exploration/mav_exploration_node.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "mav_exploration");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub = nh.advertise < trajectory_msgs::MultiDOFJointTrajectory
      > (mav_msgs::default_topics::COMMAND_TRAJECTORY, 5);
  ROS_INFO("Started exploration");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  double dt = 1.0;
  std::string ns = ros::this_node::getName();
  if (!ros::param::get(ns + "/nbvp/dt", dt)) {
    ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s",
              (ns + "/nbvp/dt").c_str());
    return -1;
  }

  static int n_seq = 0;

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;

  // Wait for 6 seconds to let the Gazebo GUI show up.
  ros::Duration(6.0).sleep();

  // This is the initialization motion, necessary that the known free space allows the planning
  // of initial paths.
  double x_target, y_target, z_target;
  ROS_INFO("Starting the planner: Performing initialization motion");
  nh.param<double>("wp_x", x_target, 0.0);
  nh.param<double>("wp_y", y_target, 0.0);
  nh.param<double>("wp_z", z_target, 1.0);
  trajectory_point.position_W.x() = x_target - 0.3;
  trajectory_point.position_W.y() = y_target;
  trajectory_point.position_W.z() = z_target;


  for (double i = 0; i <= 1.0; i = i + 0.1) {
    trajectory_point.position_W.z() = z_target * 0.25 + z_target * 0.15 * i;
    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();
    n_seq++;
    tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0);
    trajectory_point.setFromYaw(tf::getYaw(quat));
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(samples_array);
    ros::Duration(0.5).sleep();
  }

  for (double i = 0; i <= 1.0; i = i + 0.1) {
    trajectory_point.position_W.x() = x_target - 0.3;
    trajectory_point.position_W.y() = y_target;
    trajectory_point.position_W.z() = z_target * 0.4 + i * z_target * 0.6;
    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();
    n_seq++;
    tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0);
    trajectory_point.setFromYaw(tf::getYaw(quat));
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(samples_array);
    ros::Duration(0.5).sleep();
  }

  for (double i = 0; i <= 1.0; i = i + 0.1) {
    trajectory_point.position_W.x() = x_target - i * x_target - 0.3;
    trajectory_point.position_W.y() = y_target;
    trajectory_point.position_W.z() = z_target;
    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();
    n_seq++;
    tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0);
    trajectory_point.setFromYaw(tf::getYaw(quat));
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(samples_array);
    ros::Duration(0.5).sleep();
  }

  for (double i = 0; i <= 1.0; i = i + 0.1) {
    trajectory_point.position_W.x() = - 0.3;
    trajectory_point.position_W.y() = y_target;
    trajectory_point.position_W.z() = z_target;
    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();
    n_seq++;
    tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), -M_PI * i);
    trajectory_point.setFromYaw(tf::getYaw(quat));
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(samples_array);
    ros::Duration(1.0).sleep();
  }


  trajectory_point.position_W.x() -= 0.5;
  trajectory_point.position_W.y() -= 0.5;
  samples_array.header.seq = n_seq;
  samples_array.header.stamp = ros::Time::now();
  samples_array.points.clear();
  n_seq++;
  mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
  samples_array.points.push_back(trajectory_point_msg);
  trajectory_pub.publish(samples_array);
  ros::Duration(1.0).sleep();

  // Start planning: The planner is called and the computed path sent to the controller.
  int iteration = 0;
  while (ros::ok()) {
    ROS_INFO_THROTTLE(0.5, "Planning iteration %i", iteration);
    nbvplanner::nbvp_srv planSrv;
    planSrv.request.header.stamp = ros::Time::now();
    planSrv.request.header.seq = iteration;
    planSrv.request.header.frame_id = "world";
    if (ros::service::call("nbvplanner", planSrv)) {
      n_seq++;
      if (planSrv.response.path.size() == 0) {
        ros::Duration(1.0).sleep();
      }
      for (int i = 0; i < planSrv.response.path.size(); i++) {
        samples_array.header.seq = n_seq;
        samples_array.header.stamp = ros::Time::now();
        samples_array.header.frame_id = "world";
        samples_array.points.clear();
        tf::Pose pose;
        tf::poseMsgToTF(planSrv.response.path[i], pose);
        double yaw = tf::getYaw(pose.getRotation());
        trajectory_point.position_W.x() = planSrv.response.path[i].position.x;
        trajectory_point.position_W.y() = planSrv.response.path[i].position.y;
        // Add offset to account for constant tracking error of controller
        trajectory_point.position_W.z() = planSrv.response.path[i].position.z + 0.25;
        tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);
        trajectory_point.setFromYaw(tf::getYaw(quat));
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
        samples_array.points.push_back(trajectory_point_msg);
        trajectory_pub.publish(samples_array);
        ros::Duration(dt).sleep();
      }
    } else {
      ROS_WARN_THROTTLE(1, "Planner not reachable");
      ros::Duration(1.0).sleep();
    }
    iteration++;
  }

  // {
  //   //Landing sequence
  //   ROS_WARN_THROTTLE(1, "Landing now!");
  //   n_seq++;
  //   samples_array.header.seq = n_seq;
  //   samples_array.header.stamp = ros::Time::now();
  //   samples_array.points.clear();
  //   n_seq++;
  //   trajectory_point.position_W.x() = 0;
  //   trajectory_point.position_W.y() = 0;
  //   trajectory_point.position_W.z() = 0.1;
  //   tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0);
  //   trajectory_point.setFromYaw(tf::getYaw(quat));
  //   mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
  //   samples_array.points.push_back(trajectory_point_msg);
  //   trajectory_pub.publish(samples_array);
  //   ros::Duration(1.0).sleep();
  // }

}

