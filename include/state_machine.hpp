#include <boost_sml/sml.hpp>

#include "ros/ros.h"
#include "mavros_msgs/CommandSetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTakeoffLocal.h"
#include "mavros_msgs/SetTFListen.h"
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>

#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#define SphereDis 0.03

namespace sml = boost::sml;

namespace drone
{   //TF定位、目标设置
    struct TFtarget
    {
        tf::Vector3 position = tf::Vector3(0, 0, 0);
        tf::Quaternion rotation = tf::Quaternion(0, 0, 0, 1);
        void SetPosition(double x, double y, double z)
        {
            position = tf::Vector3(x, y, z);
            rotation = tf::Quaternion(0, 0, 0, 1);
        }
        void SetTarget(double p, double i, double j, double k, double x, double y, double z)
        {
            position = tf::Vector3(x, y, z);
            rotation = tf::Quaternion(p, i, j, k);
        }
        
    };
    //速度控制用
    struct VsTarget
    {
        geometry_msgs::TwistStamped vs;
        double runtime=0.0;
        void SetVsTarget(double x_l, double y_l, double z_l,double x_a, double y_a, double z_a,double time)//设置各向速度、角速度，运动时间
        {
            vs.twist.linear.x = x_l;
            vs.twist.linear.y = y_l;
            vs.twist.linear.z = z_l;
            vs.twist.angular.x = x_a;
            vs.twist.angular.y = y_a;
            vs.twist.angular.z = z_a;
            vs.header.stamp = ros::Time::now();
            this->runtime=time;
        }
    };
    //基础参数和操作函数
    struct dependencies
    {
        mavros_msgs::State current_state;
        ros::NodeHandle n;
        ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, [&](const mavros_msgs::State::ConstPtr &msg)
                                                                    { current_state = *msg; });
        ros::Publisher vel_pub;
        ros::Publisher target_pub;


        ros::ServiceClient client;
        tf::TransformBroadcaster broadcaster;
        tf::TransformListener listener;
        tf::StampedTransform transform;
        TFtarget _target;
        VsTarget vs_command;

        bool velCommandReady=true;
        //offboard外部模式设定
        void Set2Offboard()
        {
            mavros_msgs::CommandSetMode offb_set_mode;
            offb_set_mode.request.base_mode = 0;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            client = n.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
            client.call(offb_set_mode);
        }
        //设置解锁状态
        void Set2Armed(bool open)
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = open;
            client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
            client.call(arm_cmd);
        }

        //启用速度控制
        void SetVsCommand()
        {
            vel_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);//发布到mavros控制速度插件
        }

        /*设置TF监听状态(位置控制时)
        void openTFlisten(bool open)
        {
            client = n.serviceClient<mavros_msgs::SetTFListen>("/mavros/setpoint_position/set_tf_listen");
            mavros_msgs::SetTFListen tf_listen;
            tf_listen.request.value = open;
            client.call(tf_listen);
        }
        
        void getTransform(std::string target, std::string base)
        {

            try
            {
                listener.lookupTransform(target, base, ros::Time(0), transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }

        bool is_close(tf::Vector3 current_pos, tf::Vector3 target_pos, double threshold)
        {
            tfScalar dis = tf::tfDistance(current_pos, target_pos);
            if (dis <= threshold)
                return true;
            else
                return false;
        }
        bool is_rotated(tf::Quaternion current_quat, tf::Quaternion target_quat, double threshold)
        {
            tfScalar angle = current_quat.angleShortestPath(target_quat);
            if (angle <= threshold)
                return true;
            else
                return false;
        }
        */
    };

    // event
    struct release
    {
    };
    struct lock
    {
    };
    struct switchTFmode
    {
        int mode = 0;
    };
    struct moveByVel
    {
        VsTarget vs_target;
    };
    struct moveByVelonly
    {
        VsTarget vs_target;
    };
    struct moveEnd
    {
        //在带时间的速度控制中手动表示此次运动结束，回到准备状态
    };
    struct moveTo
    {
        TFtarget target;
    };
    struct tickOnce
    {
        TFtarget target;
    };

    struct icarus
    {
        auto operator()() const
        {
            using namespace sml;

            // guard
            auto is_init = [](const dependencies &d, ros::Rate _rate)
            {
                ros::spinOnce();
                _rate.sleep();
                if (ros::ok())
                {
                    while (!d.current_state.connected)
                    {
                        ros::spinOnce();
                        _rate.sleep();
                    }
                    return true;
                }
                else
                {
                    ROS_INFO("not connected");
                    return false;
                }
            };

            auto is_valid = [](dependencies &d, ros::Rate _rate)
            {
                ros::spinOnce();
                _rate.sleep();
                if (ros::ok())
                {
                    while (d.current_state.mode != "OFFBOARD")
                    {
                        ros::spinOnce();
                        _rate.sleep();
                        d.Set2Offboard();
                        d.Set2Armed(true);
                        //d.openTFlisten(true);
                    }
                    return true;
                }
                else
                {
                    return false;
                }
            };

            /*
            auto is_arrive = [](dependencies &d)
            {
                d.getTransform("map", "base_link");
                if (d.is_close(tf::Vector3(d.transform.getOrigin().x(), d.transform.getOrigin().y(), d.transform.getOrigin().z()),
                               d._target.position, SphereDis) &&
                    d.is_rotated(d.transform.getRotation(), d._target.rotation, 2))
                    return true;
                else
                    return false;
            };
            */


            // action
            auto unlock = [](dependencies &d)
            {
                d.Set2Offboard();
                d.Set2Armed(true);
                d.SetVsCommand();
                //d.openTFlisten(true);
            };
            auto delay = [](double time)
            {
                ros::Duration(time).sleep();
            };
            //带运动时间的速度控制，在事件中传入速度信息和运动时间
            auto runByVel=[](dependencies &d, ros::Rate _rate)
            {   //设置速度的时间戳记为开始时间，运行时间内不断发送速度控制信息
                d.velCommandReady=false;
                while(ros::Time::now() - d.vs_command.vs.header.stamp < ros::Duration(d.vs_command.runtime))
                {
                        d.vel_pub.publish(d.vs_command.vs);
                        _rate.sleep();
                        ros::spinOnce();
                };
                //运行时间结束，停止移动
                d.vs_command.SetVsTarget(0,0,0,0,0,0,0);
                for(int i=0;i<10;i++)
                    d.vel_pub.publish(d.vs_command.vs);
                ros::spinOnce();
                d.velCommandReady=true;
            };
            //仅发布速度信息，不控制运动时间,事件中传入速度信息即可
            auto runVelonly=[](dependencies &d, ros::Rate _rate)
            {   
                d.vel_pub.publish(d.vs_command.vs);
                _rate.sleep();
                ros::spinOnce();
            };
            //速度控制运动停止
            auto stop_runvel=[](dependencies &d)
            {
                d.vs_command.SetVsTarget(0,0,0,0,0,0,0);
                for(int i=0;i<10;i++)
                    d.vel_pub.publish(d.vs_command.vs);
            };

            /*
            auto setTFtarget = [](dependencies &d)
            {
                d.broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(d._target.rotation, d._target.position),
                        ros::Time::now(), "map", "target_position"));
            };
            */


            return make_transition_table(
                *"idle"_s + event<release>[is_init] / unlock = "ready"_s,
                // "moving"_s + "endmov"_e[is_valid] / [] {} = "ready"_s,
                /*"ready"_s + event<moveByVel>[is_valid] / ([](auto const &e, dependencies &d)
                                                       { d.vs_command = e.vs_target; },
                                                       runByVel) = "moving"_s,
                */
                //"moving"_s + event<tickOnce>[is_arrive] / [] {} = "ready"_s,
                //"moving"_s + event<tickOnce>[!is_arrive && is_valid] / setTFtarget = "moving"_s,
                //有运行时间速度控制
                "ready"_s + event<moveByVel>[is_valid] / ([](auto const &e, dependencies &d)
                                                       { d.vs_command = e.vs_target; },
                                                       runByVel) = "moving"_s,

                "moving"_s + event<tickOnce>[is_valid] / runByVel = "moving"_s,
                
                //无运行时间速度控制
                "ready"_s + event<moveByVelonly>[is_valid] / ([](auto const &e, dependencies &d)
                                                       { d.vs_command = e.vs_target; },
                                                       runVelonly) = "moving"_s,
                //无运行时间速度控制中途修改速度
                "moving"_s + event<moveByVelonly>[is_valid] / ([](auto const &e, dependencies &d)
                                                       { d.vs_command = e.vs_target; },
                                                       runVelonly) = "moving"_s,

                
                "tracking"_s + event<moveByVelonly>[is_valid] / ([](auto const &e, dependencies &d)
                                                       { d.vs_command = e.vs_target; },
                                                       runVelonly) = "tracking"_s,

                "tracking"_s + event<tickOnce>[is_valid] / runVelonly = "tracking"_s,

                //结束运动，速度归零
                "moving"_s + event<moveEnd> / stop_runvel = "ready"_s,

                "ready"_s + event<lock> / [] {} = X

            ); 
        }
    };
};
