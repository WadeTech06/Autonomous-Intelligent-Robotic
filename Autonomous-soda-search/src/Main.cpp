#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float x[4], y[4];
bool isFound;

ros::Publisher query_pub;
ros::Publisher cmd_vel_pub_;
ros::Subscriber detection_sub;

void ASS_isFoundCB(const std_msgs::String::ConstPtr &msg)
{
    if (strcmp(msg->data.c_str(), "Found") == 0)
    {
        isFound = true;
    }
}

void Search(int searchCode)
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    //Pepsi
    x[0] = 0.971981048584;
    y[0] = 1.34851241112;
    //Coke
    x[1] = 0.918219804764;
    y[1] = -2.77944302559;
    //Sprite
    x[2] = 8.33976745605;
    y[2] = 0.577239751816;

    goal.target_pose.pose.position.x = x[searchCode];
    goal.target_pose.pose.position.y = y[searchCode];
    goal.target_pose.pose.orientation.w = 1;
    ac.sendGoal(goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Navigated to known location");

        int scanCounter = 0;
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(1);
        while (ros::Time::now() - start_time < ros::Duration(60.0))
        {
            if (isFound == false)
            {
                ROS_INFO("Scanning area %d", scanCounter);
                geometry_msgs::Twist move;
                //velocity controls
                move.linear.x = 0; //speed value m/s
                move.angular.z = 0.20;
                cmd_vel_pub_.publish(move);
            }
            scanCounter++;

            ros::spinOnce();
            rate.sleep();
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Searcher");
    ros::NodeHandle n;

    query_pub = n.advertise<std_msgs::String>("ASS_Query", 1000);
    detection_sub = n.subscribe("ASS_Detection", 1000, ASS_isFoundCB);
    cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);

    std_msgs::String msg;

    isFound = false;

    int searchCode;
    do
    {
        cout << "--------------------------------------\n";
        cout << "What would you like me to find?\n";
        cout << "0 - Pepsi\n";
        cout << "1 - Coke\n";
        cout << "2 - Sprite\n";
        cout << "-1 - to Quit\n";
        cout << "--------------------------------------\n";
        cout << "Enter Value: ";
        cin >> searchCode;

        switch (searchCode)
        {
        case 0:
        {
            cout << "Searching for Pepsi\n";
            msg.data = "Pepsi";
            query_pub.publish(msg);
            Search(searchCode);
            break;
        }
        case 1:
        {
            cout << "Searching for Coke\n";
            msg.data = "Coke";
            query_pub.publish(msg);
            Search(searchCode);
            break;
        }
        case 2:
        {
            cout << "Searching for Sprite\n";
            msg.data = "Sprite";
            query_pub.publish(msg);
            Search(searchCode);
            break;
        }
        case -1:
        {
            cout << "Have a nice day!\n";
            break;
        }
        default:
        {
            cout << "Item not found\n";
            break;
        }
        }
    } while (searchCode != -1);

    return 0;
}
