#ifndef HITCHCOMMAND_H
#define HITCHCOMMAND_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <blade_nav/HitchPath.h>
#include <blade_nav/HitchPoseStamped.h>
#include "KdTree.h"

class HitchCommand {
  public:
    explicit HitchCommand();
    KdTree kd;

  private:
    ros::NodeHandle n;
    ros::Subscriber hitch_path_sub;
    ros::Subscriber pose_sub;
    ros::Publisher hitch_cmd_pub;
    blade_nav::HitchPath hitchpath_message;
    nav_msgs::Odometry odom_message;
    ros::Rate rate;

    void hitchCmdCB(const blade_nav::HitchPath& msg);
    void odomCB(const nav_msgs::Odometry& msg);
    void setHitchCmd(geometry_msgs::Pose hitch_cmd);
    struct kd_node_t * pathToArray(blade_nav::HitchPath path);

};

#endif
