#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>

void objectCd(const std_msgs::Float32MultiArray & msg);
void queryCd(const std_msgs::String::ConstPtr& msg);
void EvaluateDetection ();

const int ID_MONSTER = 8;
const int ID_PEPSI = 18;

ros::Publisher detection_pub;

int query;
float timeSinceLastDetection;
clock_t lastDetection;
int detectionCount;

void objectCd(const std_msgs::Float32MultiArray & msg)
{
	if (query != -1)
	{
		for(unsigned int i=0; i<msg.data.size(); i+=12)
		{
			// Retrieve the ID first.
			int id = (int)msg.data[i];

			// If the ID is a match of the queried can, continue processing.
			if (id == query)
			{
				// Retrieve the height, width, and corner locations.
				float objectWidth = msg.data[i+1];
				float objectHeight = msg.data[i+2];
				cv::Mat cvHomography(3, 3, CV_32F);
				cvHomography.at<float>(0,0) = msg.data[i+3];
				cvHomography.at<float>(1,0) = msg.data[i+4];
				cvHomography.at<float>(2,0) = msg.data[i+5];
				cvHomography.at<float>(0,1) = msg.data[i+6];
				cvHomography.at<float>(1,1) = msg.data[i+7];
				cvHomography.at<float>(2,1) = msg.data[i+8];
				cvHomography.at<float>(0,2) = msg.data[i+9];
				cvHomography.at<float>(1,2) = msg.data[i+10];
				cvHomography.at<float>(2,2) = msg.data[i+11];
				std::vector<cv::Point2f> inPts, outPts;
				inPts.push_back(cv::Point2f(0,0));
				inPts.push_back(cv::Point2f(objectWidth,0));
				inPts.push_back(cv::Point2f(0,objectHeight));
				inPts.push_back(cv::Point2f(objectWidth,objectHeight));
				cv::perspectiveTransform(inPts, outPts, cvHomography);

				// Object detected, call our evaluation function to continue processing.
				EvaluateDetection ();
				break;
			}
		}
	}
}

void queryCd(const std_msgs::String::ConstPtr& msg)
{
	//ROS_INFO("%s", msg->data.c_str);
	if (strcmp (msg->data.c_str (), "Pepsi") == 0)
	{
		ROS_INFO ("Pepsi queried!");
		query = ID_PEPSI;
	}
	else
	{
		ROS_INFO ("Query Stopped");
		query = -1;
	}

	timeSinceLastDetection = 0;
	detectionCount = 0;
}

void EvaluateDetection ()
{
	// If this is our first detection, just set the clock
	if (detectionCount == 0)
		lastDetection = clock ();
	else	// Otherwise, evaluate the time since the last detection and determine if it's acceptable.
	{
		timeSinceLastDetection = float (clock () - lastDetection);
		if (timeSinceLastDetection > 1000)
			detectionCount = 0;

		lastDetection = clock ();
	}

	detectionCount++;
	if (detectionCount > 1)
	{
		ROS_INFO ("ASS_ImageDetection: Item found. Waiting for new query.");
		// Notify the search node that we have located the item.
		std_msgs::String msg;
		msg.data = "Found";
		detection_pub.publish (msg);
		query = -1;
	}		
}

int main (int argc, char** argv)
{
	ros::init (argc, argv, "ASS_ImageDetection");
	ros::NodeHandle nh;
	
    	ros::Subscriber object_sub;
	ros::Subscriber query_sub;
    	object_sub = nh.subscribe("objects", 1, objectCd);
	query_sub = nh.subscribe ("ASS_Query", 1, queryCd);

	detection_pub = nh.advertise<std_msgs::String>("ASS_Detection", 1000);


	query = -1;
	timeSinceLastDetection = 0;
	detectionCount = 0;

	ROS_INFO ("ASS_ImageDetection: Initiated. Waiting for new query.");

	ros::spin ();

	return 0;
}
