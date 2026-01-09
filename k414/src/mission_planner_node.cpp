#include "ros/ros.h" 
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

#include <queue>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MissionPlanner
{
public:
    std::queue<geometry_msgs::Pose> goal_que_;

    MoveBaseClient ac_;

    int waypoint_counter = 1;

    const int state_check_interval = 15;

    MissionPlanner() : ac_("move_base", true)
    {   
        geometry_msgs::Pose pose;

        pose.position.x = 2.76;
        pose.position.y = -3.04;
        pose.orientation.z = -0.99;
        pose.orientation.w = 0.035;
        goal_que_.push(pose);

        pose.position.x = -2.82;
        pose.position.y = -5.99;
        pose.orientation.z = -0.99;
        pose.orientation.w = 0.013;
        goal_que_.push(pose);

        pose.position.x = -3.90;
        pose.position.y = -3.20;
        pose.orientation.z = 0.94;
        pose.orientation.w = 0.32;
        goal_que_.push(pose);

        pose.position.x = -5.75;
        pose.position.y = 6.02;
        pose.orientation.z = 0.02;
        pose.orientation.w = 0.99;
        goal_que_.push(pose);

        pose.position.x = -1.83;
        pose.position.y = 5.12;
        pose.orientation.z = -0.33;
        pose.orientation.w = 0.94;
        goal_que_.push(pose);

        pose.position.x = 6.04;
        pose.position.y = 0.98;
        pose.orientation.z = -0.88;
        pose.orientation.w = 0.46;
        goal_que_.push(pose);

        pose.position.x = 2.57;
        pose.position.y = -3.08;
        pose.orientation.z = 0.99;
        pose.orientation.w = 0.0;
        goal_que_.push(pose);

        pose.position.x = 2.68;
        pose.position.y = -6.13;
        pose.orientation.z = 0.99;
        pose.orientation.w = 0.06;
        goal_que_.push(pose);

        pose.position.x = 0;
        pose.position.y = 0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        goal_que_.push(pose);

        Run();
    }

    void Run()
    {
        ac_.waitForServer();
        ROS_INFO("[Mission Planner] Mission Planner Established");
        SendGoalFromQue();
        
        while(true)
        {
            ac_.waitForResult(ros::Duration(state_check_interval));
            if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
            {
                ROS_INFO("[Mission Planner] Goal Reached");
                if(!SendGoalFromQue())
                {
                    ROS_INFO("[Mission Planner] Mission Accomplished!");
                    break;
                }
            }
            ros::Duration(0.5).sleep();  
        }
    }

    bool SendGoalFromQue()
    {
        if(!goal_que_.empty())
        {
            ROS_INFO("[Mission Planner] Ready to send waypoint: %d", waypoint_counter);
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = goal_que_.front();
            // ac_.sendGoal(goal,
            // boost::bind(&MissionPlanner::DoneCb, this, _1, _2),
            // boost::bind(&MissionPlanner::ActiveCb, this),
            // boost::bind(&MissionPlanner::FeedbackCb, this, _1));
            ac_.sendGoal(goal);
            goal_que_.pop();
            waypoint_counter++;
            return true;
        }
        return false;
    }

    void DoneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) 
    {
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) 
        {
            ROS_INFO("[Mission Planner] Goal Reached");
            if(!SendGoalFromQue())
            {
                ROS_INFO("[Mission Planner] Mission Accomplished!");
            }
        }
        else
        {
            ROS_WARN("[Mission Planner] Mission did not succeed at waypoint: %d", waypoint_counter);
        }
    }

    void ActiveCb() 
    {
        ROS_INFO("[Mission Planner] Move Base just went active");
    }

    void FeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
    {
        // ROS_INFO("[Mission Planner] Current Position: [X: %f, Y: %f]", 
        //         feedback->base_position.pose.position.x, 
        //         feedback->base_position.pose.position.y);
    }
};


void pose_cb(const geometry_msgs::Pose& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.header.stamp = ros::Time::now();
    pose.pose = msg;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "mission_planner_node");
    MissionPlanner mp;

    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/initalpose", 1000);
    ros::Subscriber sub = nh.subscribe("gt_odom", 1000, pose_cb);

    ros::spin();
    return 0;
}