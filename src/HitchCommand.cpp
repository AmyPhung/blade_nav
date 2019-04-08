#include "HitchCommand.h"

HitchCommand::HitchCommand()
: rate(ros::Rate(2))
// hitch path subscriber
, hitch_path_sub(n.subscribe("/blade_nav/hitch_path", 1, &HitchCommand::HitchCommand::hitchCmdCB, this))
// pose subscriber
, pose_sub(n.subscribe("/odometry/filtered", 1, &HitchCommand::HitchCommand::odomCB, this))
// hitch command publisher
, hitch_cmd_pub(n.advertise<geometry_msgs::Point>("cmd_hitch", 1)) {

}

void HitchCommand::hitchCmdCB(const blade_nav::HitchPath& msg) {
  // Hitch command callback function
  // Only runs once TODO: Use waypoints to stop sending hitch commands

  hitchpath_message = msg;
  int n = msg.poses.size();

  // Make array from hitch path message
  kd_node_t * arr;
  arr = pathToArray(msg);

  // Create kd tree using formatted array and keep track of the root node
  KdTree kd;
  kd_node_t * root;
  root = kd.make_tree(arr, n, 0, 2);

  while (ros::ok()) {
    // Initialize needed pieces of kd tree
    struct kd_node_t position_node = {{odom_message.pose.pose.position.x,
                                       odom_message.pose.pose.position.y}};
    struct kd_node_t *nearest_node;
    double best_dist;
    kd.visited = 0;
    nearest_node = 0;

    // Find nearest node to current position
    kd.nearest(root, &position_node, 0, 2, &nearest_node, &best_dist);

    // Publish hitch command stored in nearest node
    geometry_msgs::Point hitch_cmd;
    hitch_cmd.z = nearest_node->value;
    setHitchCmd(hitch_cmd);

    // Give time for other topics to update
    ros::spinOnce();
  }
}

void HitchCommand::odomCB(const nav_msgs::Odometry& msg) {
  odom_message = msg;
}

void HitchCommand::setHitchCmd(geometry_msgs::Point hitch_cmd) {
  hitch_cmd_pub.publish(hitch_cmd);
}

struct kd_node_t * HitchCommand::pathToArray(blade_nav::HitchPath path) {
  // Convert from HitchPath message to formtted array
  // Format of output array: {{{x1,y1}, hitch_value1}, {{x2,y2}, hitch_value2}, ...}

  int num_pts = path.poses.size();
  struct kd_node_t *arr = new struct kd_node_t[num_pts];

  for (int i=0; i<num_pts; i++) {
     arr[i] = {{path.poses[i].pose.position.x, path.poses[i].pose.position.y},
                path.poses[i].hitch.z};
  }

  return arr;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "HitchCommand");
  HitchCommand hitchcommand;
  ros::spin();
}
