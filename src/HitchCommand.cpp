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
  hitchpath_message = msg;
  int n = msg.poses.size();

  kd_node_t * arr;
  arr = pathToArray(msg);

  KdTree kd;
  kd_node_t * root;
  root = kd.make_tree(arr, n, 0, 2);

  geometry_msgs::Point hitch_cmd;

  while (ros::ok()) {
    struct kd_node_t position_node = {{odom_message.pose.pose.position.x,
                                       odom_message.pose.pose.position.y}};
    struct kd_node_t *nearest_node;
    double best_dist;

    kd.visited = 0;
    nearest_node = 0;
    kd.nearest(root, &position_node, 0, 2, &nearest_node, &best_dist);

    std::cout<<"odom!" << odom_message.pose.pose.position.x<<std::endl;
    std::cout<<"best value!" <<nearest_node->value<<std::endl;

    geometry_msgs::Point hitch_cmd;
    hitch_cmd.z = nearest_node->value;
    setHitchCmd(hitch_cmd);

    ros::spinOnce();
  }
}

void HitchCommand::odomCB(const nav_msgs::Odometry& msg) {
  odom_message = msg;
  std::cout<<odom_message.pose.pose.position.x<<std::endl;
}

void HitchCommand::setHitchCmd(geometry_msgs::Point hitch_cmd) {
  hitch_cmd_pub.publish(hitch_cmd);
}

struct kd_node_t * HitchCommand::pathToArray(blade_nav::HitchPath path) {
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
