//#include <iostream> //used for testing
#include "ros/ros.h"
#include "../include/state_machine_old.hpp"
//#include "../include/locate_elg.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "tf/transform_datatypes.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <tf/tf.h>
namespace sml = boost::sml;
using namespace message_filters;


//双目
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


int main(int argc, char **argv)
{
    using namespace sml;
    ros::init(argc, argv, "simple_fly");
    drone::dependencies d;
    ros::NodeHandle nh;
    double angle;
//双目
/*
    Eigen::Matrix<double, 3, 3> K_left, K_right;
    K_left << 427.2486928456254, 0, 425.06874497136005, 0, 427.74610622571066, 236.16152744508838, 0, 0, 1;
    K_right << 425.81737868037914, 0, 424.41982264160583, 0, 426.2683663190262, 235.4487746155112, 0, 0, 1;
    message_filters::Subscriber<sensor_msgs::Image> sub_left_image(nh, "/camera/infra1/image_rect_raw", 2000, ros::TransportHints().tcpNoDelay());

    message_filters::Subscriber<sensor_msgs::Image> sub_right_image(nh, "/camera/infra2/image_rect_raw", 2000, ros::TransportHints().tcpNoDelay());

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), sub_left_image, sub_right_image);
    // 指定一个回调函数，就可以实现两个话题数据的同步获取
    sync.registerCallback(boost::bind(&image_callback, _1, _2));
*/


//测试起飞
    ros::Rate rate(10);
    d.n = nh;
    sml::sm<drone::icarus> sm{d, rate};
    sm.process_event(drone::release{});
    drone::TFtarget target;

    bool is_detected;
    geometry_msgs::Quaternion rotate;
    tf::Quaternion tf_rotate;
    std::vector<cv::Point3d> normal;

//从起点飞到杆A正前方。。。。根据情况修改目标位置


//不改变姿态
    target.SetPosition(0, 0, 1.1);
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
    }
//改变姿态
    target.SetTarget(0, 0, 0, 1, 0, 0, 1.1);
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
    }



//识别杆A相对于相机中心位置，pid修正目标位置1



//巡线，直接设置向右飞，可操作难点：相对于线的距离
/*检测黄色条形码并存照*/
/*检测杆B，检测到后执行下一步操作*/
//识别杆B相对于相机中心位置，pid修正目标位置1


//向前飞一段距离

//向左飞巡线
/*检测杆A，检测到后执行下一步操作*/
//识别杆A相对于相机中心位置，pid修正目标位置1
//向后飞一段距离
/*识别起点，pid修正降落位置*/


target.SetPosition(0, 0, 1.1);
//示例计算目标位置及位姿
/*
    normal = zuanquan(left_image, right_image, K_left, K_right, is_detected);
    ROS_INFO("1");
    while (!is_detected)
    {
        normal = zuanquan(left_image, right_image, K_left, K_right, is_detected);
        ros::Duration(0.5).sleep();
        ROS_INFO("detecting!");
    }
    angle = -atan(normal[0].x / normal[0].z);
    double a_b = qumo(normal[0]);
    
    cv::Point3d target_pos = cv::Point3d(normal[1].x - normal[0].x / a_b * 0.5, 1.1, normal[1].z - normal[0].z / a_b * 0.5);
    rotate = tf::createQuaternionMsgFromYaw(angle);
    tf::quaternionMsgToTF(rotate, tf_rotate);
    tf::Quaternion new_rotate = tf::Quaternion(0, 0, 0, 1) * tf_rotate;
    target.SetPosition(target_pos.z, -target_pos.x, 1.1);
    ROS_INFO("x:%f, y:%f",target_pos.z, -target_pos.x);
    



//先调整位姿
 
    target.rotation = new_rotate.normalize();
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
    }
    ros::Duration(10).sleep();


    target_pos = cv::Point3d(normal[1].x + normal[0].x / a_b * 0.5, 1.1, normal[1].z + normal[0].z / a_b * 0.5);


*/

/*
   cv::Point3d target_pos = (0,0,1.1);
  tf::Quaternion new_rotate = tf::Quaternion(0, 0, 0, 1) * tf_rotate;

//再飞向计算目标
    target.SetPosition(target_pos.z, -target_pos.x, 1.1);
    //target.rotation = new_rotate.normalize();
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
    }











//降落
    target.SetPosition(target_pos.z, -target_pos.x, -0.2);
    target.rotation = new_rotate.normalize();
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
    }

*/
//结束操作

    sm.process_event(drone::lock{});
    return 0;
}
