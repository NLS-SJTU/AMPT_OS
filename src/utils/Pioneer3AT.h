#ifndef _PIONEER3AT_H
#define _PIONEER3AT_H

#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

class Pioneer3AT
{
public:
    Pioneer3AT(ros::NodeHandle &_n):nh(_n)
    {
        initPublisherSubscriber();
    }
    ~Pioneer3AT(){}

    //input should be m/s and rad/s
    void move(float forwardx, float rotatez){
        geometry_msgs::Twist _vel;
        _vel.linear.x = forwardx;
        _vel.linear.y = 0;
        _vel.linear.z = 0;
        _vel.angular.x = 0;
        _vel.angular.y = 0;
        _vel.angular.z = rotatez;

        vel_pub.publish(_vel);
    }
    //enable motor and stop
    void takeOff(){
        std_srvs::Empty _srv;
        enable_srv.call(_srv);
        move(0, 0);
    }
    //stop and disable motor
    void land(){
        move(0, 0);
        std_srvs::Empty _srv;
        disable_srv.call(_srv);
    }
private:
    ros::NodeHandle nh;

    ros::Publisher vel_pub;

    ros::ServiceClient enable_srv;
    ros::ServiceClient disable_srv;

    void initPublisherSubscriber(){
        //publisher
        vel_pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);

        //subscriber

        //service client
        enable_srv = nh.serviceClient<std_srvs::Empty>("RosAria/enable_motors");
        disable_srv = nh.serviceClient<std_srvs::Empty>("RosAria/disable_motors");
    }
};

#endif
