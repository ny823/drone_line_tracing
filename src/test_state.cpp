#include <iostream> //used for testing
#include "ros/ros.h"
#include "../include/state_machine_old.hpp"
//#include "../include/locate_elg.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>
#include "../include/PID.hpp"
#include "tf/transform_datatypes.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <tf/tf.h>
#include "linetracing/CustomMsg.h"
namespace sml = boost::sml;
using namespace message_filters;
using namespace sml;
using namespace std;
//float para0 = 0.0;
//float para1 = 0.0;
//float para2 = 0.0;
//float para3 = 0.0; 
//bool isapear = false;
//åŒç›®
/*
cv::Mat left_image;
cv::Mat right_image;

void image_callback(const sensor_msgs::ImageConstPtr &left_img, const sensor_msgs::ImageConstPtr &right_img)
{
    cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::TYPE_8UC1);
    left_image = cv_ptr1->image;
    cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::TYPE_8UC1);
    right_image = cv_ptr2->image;
}
*/


void read_yaml(const YAML::Node &yaml, tf::Vector3 &setposition1, tf::Vector3 &setposition2, tf::Vector3 &setposition3, tf::Vector3 &setposition4)
{
    double px1 = yaml["px1"].as<double>();
    double py1 = yaml["py1"].as<double>();
    double pz1 = yaml["pz1"].as<double>();
    setposition1 = tf::Vector3(px1, py1, pz1);//½×¶Î1

    double px2 = yaml["px2"].as<double>();
    double py2 = yaml["py2"].as<double>();
    double pz2 = yaml["pz2"].as<double>();
    setposition2 = tf::Vector3(px2, py2, pz2);//½×¶Î2

    double px3 = yaml["px3"].as<double>();
    double py3 = yaml["py3"].as<double>();
    double pz3 = yaml["pz3"].as<double>();
    setposition3 = tf::Vector3(px3, py3, pz3);//½×¶Î3

    double px4 = yaml["px4"].as<double>();
    double py4 = yaml["py4"].as<double>();
    double pz4 = yaml["pz4"].as<double>();
    setposition4 = tf::Vector3(px4, py4, pz4);//½×¶Î4
}

void read_PID(const YAML::Node &pid_yaml, PIDController &pid, int i)
{
    if (i = 0) // x,y
    {
        pid.Kd = pid_yaml["Kd"].as<double>();
        pid.Ki = pid_yaml["Ki"].as<double>();
        pid.Kp = pid_yaml["Kp"].as<double>();
        pid.limMax = pid_yaml["limMax"].as<double>();
        pid.limMin = pid_yaml["limMin"].as<double>();
        pid.limMaxInt = pid_yaml["limMaxInt"].as<double>();
        pid.limMinInt = pid_yaml["limMinInt"].as<double>();
        pid.T = pid_yaml["T"].as<double>();
    }
    if (i = 1) // z yaw
    {
        pid.Kd = pid_yaml["Kd_"].as<double>();
        pid.Ki = pid_yaml["Ki_"].as<double>();
        pid.Kp = pid_yaml["Kp_"].as<double>();
        pid.limMax = pid_yaml["limMax_"].as<double>();
        pid.limMin = pid_yaml["limMin_"].as<double>();
        pid.limMaxInt = pid_yaml["limMaxInt_"].as<double>();
        pid.limMinInt = pid_yaml["limMinInt_"].as<double>();
        pid.T = pid_yaml["T"].as<double>();
    }
}

linetracing::CustomMsg cu_msg;
void chatterCallback(const linetracing::CustomMsg::ConstPtr &msg /*float &para1, float &para2, float &para3, bool &isapear*/)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
    //para0 = msg->field1;
    //para1 = msg->field2; 
    //para2 = msg->field3;
    //para3 = msg->field4;
    //isapear = msg->field5;
    //ROS_INFO("Subcribe Person Info :name:%f  age:%f  sex:%f", msg->field1 , msg->field2 ,msg->field3);
    cu_msg = *msg;
    // cout << "ok" << endl;
    // cout << cu_msg.field1 << endl;
    // cout << cu_msg.field2 << endl;
    // cout << cu_msg.field3 << endl;
    // cout << cu_msg.field4 << endl;
    // cout << cu_msg.field5 << endl;
}
//geometry_msgs::TwistStamped vs;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_fly");
    drone::dependencies d;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<linetracing::CustomMsg>("cumtom_msg_topic", 1000, chatterCallback);
    ros::Rate rate(15);
    d.n = nh;
    sml::sm<drone::icarus> sm{d, rate};
    YAML::Node message = YAML::LoadFile("/home/liuqing/acfly_ws11/src/linetracing/config/position.yaml");
    ros::Publisher vel_sp_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    tf::Vector3 po1;
    tf::Vector3 po2;
    tf::Vector3 po3;
    tf::Vector3 po4;
    read_yaml(message, po1, po2, po3, po4);
    //float para0 = cu_msg.field1;
    //float para1 = cu_msg.field2;
    //float para2 = cu_msg.field3;
    //float para3 = cu_msg.field4; 
    //bool isapear = cu_msg.field5;
    float para0;
    float para1;
    float para2;
    float para3; 
    bool isapear;
    // ROS_INFO(para0);
    // ROS_INFO(para1);
    // ROS_INFO(para2);
    // ROS_INFO(para3);
    // ROS_INFO(isapear);
    // cout << para0 << endl;
    // cout << para1 << endl;
    // cout << para2 << endl;
    // cout << para3 << endl;
    // cout << isapear << endl;

    //read_yaml(message, po2);
    //read_yaml(message, po3);
    //read_yaml(message, po4);
    // read_pid
    PIDController mypid_x;
    PIDController mypid_y;
    PIDController mypid_z;
    PIDController mypid_z_angular;
    YAML::Node message1 = YAML::LoadFile("/home/liuqing/acfly_ws11/src/linetracing/config/msg.yaml");
    read_PID(message1, mypid_x, 0);
    read_PID(message1, mypid_y, 0);
    read_PID(message1, mypid_z, 1);
    read_PID(message1, mypid_z_angular, 1);
    double coff = message1["coff"].as<double>();

    drone::TFtarget target;
    bool is_detected;
    geometry_msgs::Quaternion rotate;
    tf::Quaternion tf_rotate; 
    sm.process_event(drone::release{});
    target.SetTarget(0, 0, 0, 1, 0, 0, 1.1);
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        ROS_INFO("take_of");
    }
    target.SetTarget(0, 0, 0, 1, 1, 0, 1.1);
    // target.SetPosition_yaml(po2);
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        ROS_INFO("moving_to_gan");
    }
    //??????,????????
    PIDController_Init(mypid_x);
    PIDController_Init(mypid_y);
    target.SetTarget(0, 0, 0, 1, 1, -2, 1.1);
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        ROS_INFO("line_tracing");
        para1 = cu_msg.field2;
        last_request = ros::Time::now();
        if(para1 < 200 || para1 >280)
        {
            mavros_msgs::PositionTarget vsT_body_axis;
            vsT_body_axis.x = PIDController_Update(mypid_x, para1, 240, coff);
            //vsT_body_axis.y = PIDController_Update(mypid_y, para1, 320, coff);
            vel_sp_pub.publish(vsT_body_axis);
            //vsT_body_axis.z = PIDController_Update(mypid_z, transform.getOrigin().z(), 1.0, 1);
            // while( ros::Time::now() - last_request < ros::Duration(3.0))
            // {
            //     vsT.type_mask=4039+24;
            //     vsT_body_axis.type_mask=4039+24;
            //     vsT.velocity.x = -0.1;
            //     vsT.velocity.y = 0;
            //     vsT.velocity.z = 0;

            // }
        }
    }
    //´ýÌí¼Ó ÈÆ¸Ë
    // sm.process_event(drone::moveTo{target})
    // target.SetTarget(0, 0, 0, 1, 1, -1, 1.1);
    // //target.SetPosition_yaml(po3);
    // sm.process_event(drone::moveTo{target});
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     ROS_INFO("33333333333333333333333333333333333333333");
    // }
    target.SetTarget(0, 0, 0, 1, 1, -2, -0.1);
    //target.SetPosition_yaml(po4);
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        ROS_INFO("landing");
    }
    sm.process_event(drone::lock{});
    ros::spin();
    return 0;
}
