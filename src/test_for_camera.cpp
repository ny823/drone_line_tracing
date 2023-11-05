#include <boost_sml/sml.hpp>
#include <cassert>
#include <iostream> //used for testing
#include "ros/ros.h"
#include "mavros_msgs/CommandSetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTakeoffLocal.h"
#include "mavros_msgs/SetTFListen.h"
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <unistd.h>
#include "../include/color.hpp"
#include "../include/PID.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "../include/rec_line_new.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "std_msgs/String.h"
#include "linetracing/CustomMsg.h"
using namespace cv;
using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "img_publisher");
  //ros::NodeHandle nh;
  ros::NodeHandle n;
  //image_transport::ImageTransport it(nh);
  //image_transport::Publisher pub = it.advertise("camera/image", 1);
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher pub = n.advertise<linetracing::CustomMsg>("cumtom_msg_topic", 1000);

  cv::VideoCapture cap(0);
  cap.set(CAP_PROP_FRAME_WIDTH, 640);//宽度
  cap.set(CAP_PROP_FRAME_HEIGHT, 480);//高度
  cv::Mat img;
  ros::Rate loop_rate(30);
  bool isappbarcode = false; 
  bool isappqrcode = false;
  bool isappline;
  int qrnum = 1;
  int barnum = 1;
  while (ros::ok()) 
	{
		cap >> img;
		//bool isapp;
		std::vector<cv::Point> vec_re;
		vec_re = get_line_erect(img,isappline,isappbarcode,isappqrcode,qrnum,barnum);
		cout <<"is"<< vec_re << endl;
		cout <<"bool"<< isappline << endl;
    linetracing::CustomMsg msg2;
    msg2.field1=vec_re[0].x;
    msg2.field2=vec_re[0].y;
    msg2.field3=vec_re[1].x;
    msg2.field4=vec_re[1].y;
    msg2.field5=vec_re[2].x;
    msg2.field6=isappline;
    pub.publish(msg2);
	
		ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
