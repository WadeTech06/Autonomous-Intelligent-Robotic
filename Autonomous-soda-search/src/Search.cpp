#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void Search_detectionCB(const std_msgs::String::ConstPtr &msg)
{
    if (strcmp(msg->data.c_str(), "Found") == 0)
    {
        ROS_INFO("Found Object");
        ROS_INFO("Stopping...");

        //tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);

        while (!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        ac.cancelAllGoals ();
        // ac.waitForResult();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Searcher_listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("ASS_Detection", 1000, Search_detectionCB);

    ros::spin();

    return 0;
}
