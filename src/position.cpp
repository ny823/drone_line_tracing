#include "ros/ros.h"
#include "mavros_msgs/CommandSetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTakeoffLocal.h"
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>

mavros_msgs::PositionTarget vsT;
mavros_msgs::PositionTarget vsT_body_axis;
//uint FRAME_LOCAL_NED = 1;


geometry_msgs::TwistStamped vs;
geometry_msgs::TwistStamped vs_body_axis;
mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_fly");
  ros::NodeHandle n;
  // ros::NodeHandle nh;

  //ros::Publisher vel_sp_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
  //ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
  ros::Publisher vel_sp_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
  ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
  ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
  ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  ros::Rate rate(20.0);

  vsT.coordinate_frame = 1;

  vsT_body_axis.coordinate_frame = 1;

//uint16_t type_mask=4039;

  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::CommandSetMode offb_set_mode;
  // srv_setmode.request.base_mode = 0;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  set_mode_client.call(offb_set_mode);

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  arming_client.call(arm_cmd);

  ros::Time last_request = ros::Time::now();
  while (ros::ok() && (!current_state.armed || current_state.mode != "OFFBOARD"))
  {
    if (ros::Time::now() - last_request < ros::Duration(10.0))
    {

      ros::spinOnce();
      rate.sleep();
    }
    else
    {
      ROS_INFO("failed");
      return 1;
    }
  }
  /*
  vs.twist.linear.x = 0.0;
  vs.twist.linear.y = 0.0;
  vs.twist.linear.z = 0.0;
  vs.twist.angular.x = 0.0;
  vs.twist.angular.y = 0.0;
  vs.twist.angular.z = 0.0;

  vs_body_axis.twist.linear.x = 0.0;
  vs_body_axis.twist.linear.y = 0.0;
  vs_body_axis.twist.linear.z = 0.0;
  vs_body_axis.twist.angular.x = 0.0;
  vs_body_axis.twist.angular.y = 0.0;
  vs_body_axis.twist.angular.z = 0.0;
  */
  vsT.velocity.x = 0.0;
  vsT.velocity.y = 0.0;
  vsT.velocity.z = 0.0;


  vsT_body_axis.velocity.x = 0.0;
  vsT_body_axis.velocity.y = 0.0;
  vsT_body_axis.velocity.z = 0.0;

  last_request = ros::Time::now();
  while (ros::ok())
  {
    if (ros::Time::now() - last_request < ros::Duration(5.0))
    {
      vsT.type_mask=4039+24;
      vsT_body_axis.type_mask=4039+24;
      vsT.velocity.x = 0;
      vsT.velocity.y = 0;
      vsT.velocity.z = 0;
      ROS_INFO("1111");
    }
/*    
    if (ros::Time::now() - last_request >= ros::Duration(5.0) && ros::Time::now() - last_request < ros::Duration(10.0))
    {
      vsT.twist.linear.x = 0;
      vsT.twist.linear.y = 0;
      vsT.twist.linear.z = 0.2;
      vsT.twist.angular.x = 0;
      vsT.twist.angular.y = 0;
      vsT.twist.angular.z = 0;
      ROS_INFO("222");
    }
*/
    if (ros::Time::now() - last_request >= ros::Duration(5.0) && ros::Time::now() - last_request < ros::Duration(10.0))
    {
        vsT.type_mask=4039;
      vsT_body_axis.type_mask=4039;
      vsT.velocity.x = 0;
      vsT.velocity.y = 0;
      vsT.velocity.z = 0.15;
      ROS_INFO("222");
    }
  ros::Rate rate(20.0);



    if (ros::Time::now() - last_request >= ros::Duration(10.0) && ros::Time::now() - last_request < ros::Duration(15.0))
    {
      vsT.type_mask=4039+24;
      vsT_body_axis.type_mask=4039+24;
      vsT.velocity.x = 0;
      vsT.velocity.y = 0;
      vsT.velocity.z = 0;
      ROS_INFO("333");

    }
    //ROS_INFO("333");
    if (ros::Time::now() - last_request >= ros::Duration(15.0) && ros::Time::now() - last_request < ros::Duration(20.0))
    {
      vsT.type_mask=4039;
      vsT_body_axis.type_mask=4039;
      vsT.velocity.x = 0.15;
      vsT.velocity.y = 0;
      vsT.velocity.z = 0;
      ROS_INFO("4444");
    }
    
    if (ros::Time::now() - last_request >= ros::Duration(20.0) && ros::Time::now() - last_request < ros::Duration(25.0))
    {
          vsT.type_mask=4039+24;
      vsT_body_axis.type_mask=4039+24;
      vsT.velocity.x = 0;
      vsT.velocity.y = 0;
      vsT.velocity.z = 0;
      ROS_INFO("555");
    }
    
    if (ros::Time::now() - last_request >= ros::Duration(25.0) && ros::Time::now() - last_request < ros::Duration(30.0))
    {
      vsT.type_mask=4039;
      vsT_body_axis.type_mask=4039;
      vsT.velocity.x = 0;
      vsT.velocity.y = -0.15;
      vsT.velocity.z = 0;
      ROS_INFO("666");
    }
    

    if (ros::Time::now() - last_request >= ros::Duration(30.0) && ros::Time::now() - last_request < ros::Duration(35.0))
    {
          vsT.type_mask=4039+24;
      vsT_body_axis.type_mask=4039+24;
      vsT.velocity.x = 0;
      vsT.velocity.y = 0;
      vsT.velocity.z = 0;
      ROS_INFO("7777");
    }
    
    if (ros::Time::now() - last_request >= ros::Duration(35.0) && ros::Time::now() - last_request < ros::Duration(45.0))
    {
      vsT.type_mask=4039;
      vsT_body_axis.type_mask=4039;
      vsT.velocity.x = 0;
      vsT.velocity.y = 0;
      vsT.velocity.z = -0.15;
      ROS_INFO("8888");
    }
    
    if (ros::Time::now() - last_request >= ros::Duration(45.0))
    {
      arm_cmd.request.value = false;
      arming_client.call(arm_cmd);
      break;
    }
    vsT_body_axis.velocity.x = vsT.velocity.x;
    vsT_body_axis.velocity.y = vsT.velocity.y;
    vsT_body_axis.velocity.z = vsT.velocity.z;
    //vsT_body_axis.twist.angular.x = vsT.twist.angular.x;
    //vsT_body_axis.twist.angular.y = vsT.twist.angular.y;
    //vsT_body_axis.twist.angular.z = vsT.twist.angular.z;
    vsT_body_axis.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vsT_body_axis);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
