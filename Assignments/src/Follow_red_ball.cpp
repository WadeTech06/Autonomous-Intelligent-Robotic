#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber depth_sub;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher turtle_pub;

  int Latitude, Longitude;
  int cRow, cCol;

public:
  ImageConverter() : it_(nh_)
  {
    // Subscribe to input video feed  and depth video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 10,
                               &ImageConverter::imageCb, this);
    depth_sub = it_.subscribe("/camera/depth/image", 10,
                              &ImageConverter::depthCb, this);

    image_pub_ = it_.advertise("/image_converter/output_video", 10);
    turtle_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &image_msg)
  {
    cv_bridge::CvImagePtr cv_ptr_image;
    try
    {
      cv_ptr_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    int row = 0;
    int col = 0;
    int dis_min = 1000000;
    int lowLimit,highLimit;
    for (int i = 0; i < cv_ptr_image->image.rows; i++)
    {
      for (int j = 0; j < cv_ptr_image->image.cols; j++)
      {
        int b = cv_ptr_image->image.at<cv::Vec3b>(i, j).val[0];
        int g = cv_ptr_image->image.at<cv::Vec3b>(i, j).val[1];
        int r = cv_ptr_image->image.at<cv::Vec3b>(i, j).val[2];

        int dis = (r - 255) * (r - 255) + g * g + b * b;

        if (dis < dis_min)
        {
          dis_min = dis;
          row = i;
          col = j;
        }
      }
    }

    cRow = row;
    cCol = col;
    lowLimit =image_msg->width *.4;
    highLimit = image_msg->width * .6;
    if (col < lowLimit)
      Latitude = -1;
    else if (col > highLimit)
      Latitude = 1;
    else
      Latitude = 0;

    ROS_INFO("B value: %d", cv_ptr_image->image.at<cv::Vec3b>(0, 0).val[0]);
    ROS_INFO("G value: %d", cv_ptr_image->image.at<cv::Vec3b>(0, 0).val[1]);
    ROS_INFO("R value: %d", cv_ptr_image->image.at<cv::Vec3b>(0, 0).val[2]);
    ROS_INFO("Height: %d", image_msg->height);
    ROS_INFO("Width: %d", image_msg->width);
    ROS_INFO_STREAM("Encoding: " << image_msg->encoding);

    ROS_INFO("Row: %d", row);
    ROS_INFO("Col: %d", col);

    cv::circle(cv_ptr_image->image, cv::Point(col, row), 10, CV_RGB(255, 0, 0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr_image->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr_image->toImageMsg());
    Update();
  }

  void depthCb(const sensor_msgs::Image::ConstPtr &depth_msg)
  {
    cv_bridge::CvImagePtr cv_ptr_depth;
    try
    {
      cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
      float distance = cv_ptr_depth->image.at<float>(cRow, cCol);
      ROS_INFO("Distance: %f", cv_ptr_depth->image.at<float>(cRow, cCol));

      if (distance < 1.0)
        Longitude = -1;
      else if (distance > 1.25)
        Longitude = 1;
      else
        Longitude = 0;

      Update();
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void Update()
  {
    geometry_msgs::Twist move_msg;

    move_msg.linear.x = 0;
    move_msg.linear.y = 0;
    move_msg.linear.z = 0;
    move_msg.angular.x = 0;
    move_msg.angular.y = 0;
    move_msg.angular.z = 0;

    std::string output = "/ LATITUDE";
    switch (Latitude)
    {
    case 0:
      output += "STABLE";
      break;
    case -1:
      output += "LEFT";
      move_msg.angular.z = .25;
      break;
    case 1:
      output += "RIGHT";
      move_msg.angular.z = -.25;
      break;
    }
    output += " / LONGITUDE ";
    switch (Longitude)
    {
    case 0:
      output += "STABLE";
      break;
    case -1:
      output += "BACKWARDS";
      move_msg.linear.x = -.125;
      break;
    case 1:
      output += "FORWARDS";
      move_msg.linear.x = .125;
      break;
    }

    turtle_pub.publish(move_msg);
    ROS_INFO("%s", output.c_str());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
