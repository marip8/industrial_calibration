/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/server/simple_action_server.h>
#include <industrial_extrinsic_cal/robot_joint_values_triggerAction.h>  // one of the ros action messages
#include <industrial_extrinsic_cal/robot_pose_triggerAction.h>          // the other ros action message

const static double PLANNING_TIME = 10.0f;
const static int PLANNING_ATTEMPTS = 30;

class ServersNode
{
protected:
  ros::NodeHandle nh_;

public:
  typedef actionlib::SimpleActionServer<industrial_extrinsic_cal::robot_joint_values_triggerAction> JointValuesServer;
  typedef actionlib::SimpleActionServer<industrial_extrinsic_cal::robot_pose_triggerAction> PoseServer;

  ServersNode(const std::string& name,
              const std::string& move_group,
              const std::string& pose_reference_frame,
              const std::string& ee_link)
    : joint_value_server_(nh_, name + "_joint_values", boost::bind(&ServersNode::jointValueCallBack, this, _1), false)
    , pose_server_(nh_, name + "_pose", boost::bind(&ServersNode::poseCallBack, this, _1), false)
    , action_name_(name)
  {
    joint_value_server_.start();
    pose_server_.start();

    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(move_group));
    move_group_->setPlanningTime(PLANNING_TIME);                     // give it 10 seconds to plan
    move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);              // Allow parallel planner to hybridize this many plans
    move_group_->setPoseReferenceFrame(pose_reference_frame);
    move_group_->setEndEffectorLink(ee_link);

    //move_group_->setPlannerId("RRTConnectkConfigDefault");  // use this planner
  }

  ~ServersNode()
  {
//    delete (move_group_);
  }

  void jointValueCallBack(const industrial_extrinsic_cal::robot_joint_values_triggerGoalConstPtr& goal)
  {
    // TODO send both values and names and make sure they match. This is critical or else robots will crash into stuff
    std::vector<std::string> joint_names = move_group_->getJointNames();
    std::string joint_name_list;
    for(const std::string& joint_name : joint_names)
    {
      joint_name_list += joint_name + ", ";
    }
    ROS_INFO("%lu variables: %s", joint_names.size(), joint_name_list.c_str());

    move_group_->setStartStateToCurrentState();
    move_group_->setJointValueTarget(goal->joint_values);
    if (move_group_->move())
    {
      sleep(1);
      joint_value_server_.setSucceeded();
    }
    else
    {
      ROS_ERROR("move in jointValueCallBack failed. Receive %lu joint names.", joint_names.size());

      joint_value_server_.setAborted();
    }
  }

  void poseCallBack(const industrial_extrinsic_cal::robot_pose_triggerGoalConstPtr& goal)
  {
    move_group_->setStartStateToCurrentState();

    geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();
    ROS_ERROR("starting pose = %lf  %lf  %lf  %lf  %lf  %lf  %lf", current_pose.pose.position.x,
              current_pose.pose.position.y, current_pose.pose.position.z, current_pose.pose.orientation.x,
              current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
//    geometry_msgs::Pose target_pose = goal->pose;
    std::vector<geometry_msgs::Pose> poses;
//    target_pose.position.x = goal->pose.position.x;
//    target_pose.position.y = goal->pose.position.y;
//    target_pose.position.z = goal->pose.position.z;
//    target_pose.orientation.x = goal->pose.orientation.x;
//    target_pose.orientation.y = goal->pose.orientation.y;
//    target_pose.orientation.z = goal->pose.orientation.z;
//    target_pose.orientation.w = goal->pose.orientation.w;
//    ROS_ERROR("target   pose = %lf  %lf  %lf  %lf  %lf  %lf  %lf", target_pose.position.x, target_pose.position.y,
//              target_pose.position.z, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z,
//              target_pose.orientation.w);

//    poses.push_back(target_pose);
    poses.push_back(goal->pose);
    move_group_->setPoseTargets(poses, move_group_->getEndEffectorLink());

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan))
    {
      if (move_group_->execute(plan))
      {
        pose_server_.setSucceeded();
      }
      else
      {
        ROS_ERROR("execute in poseCallBack failed");
      }
    }
    else
    {
      ROS_ERROR("plan in poseCallBack failed");
    }
  }

private:
  JointValuesServer joint_value_server_;
  PoseServer pose_server_;
  std::string action_name_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosRobotActionTriggerServer");

  ros::NodeHandle pnh("~");

  std::string move_group;
  if(!pnh.getParam("move_group", move_group))
  {
    ROS_ERROR("Failed to get 'move_group' parameter");
    return -1;
  }

  std::string pose_reference_frame;
  if(!pnh.getParam("pose_reference_frame", pose_reference_frame))
  {
    ROS_ERROR("Failed to get 'pose_reference_frame' parameter");
    return -1;
  }

  std::string ee_link;
  if(!pnh.getParam("ee_link", ee_link))
  {
    ROS_ERROR("Failed to get 'ee_link' parameter");
    return -1;
  }

  ServersNode Servers(ros::this_node::getName(), move_group, pose_reference_frame, ee_link);
  ros::spin();
  return 0;
}
